/*
 * hw::Compositor implementation - see hw/compositor.h for the rationale.
 */
#include "compositor.h"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/dma-heap.h>
#include <linux/dma-buf.h>

#include "im2d.hpp"

namespace hw {
namespace {

#define COMPE(...) do { fprintf(stderr, "[E] compositor: " __VA_ARGS__); fflush(stderr); } while (0)

int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR);
    return r;
}

RgaSURF_FORMAT rga_format(PixelFormat f)
{
    switch (f) {
    case PixelFormat::Gray8:    return RK_FORMAT_YCbCr_400;
    case PixelFormat::NV12:     return RK_FORMAT_YCbCr_420_SP;
    case PixelFormat::BGRX8888: return RK_FORMAT_BGRX_8888;
    case PixelFormat::RGBA8888: return RK_FORMAT_RGBA_8888;
    }
    return RK_FORMAT_YCbCr_400;
}

rga_buffer_t wrap(rga_buffer_handle_t h, const ImageDesc &img, PixelFormat as)
{
    /* RGA counts stride in pixels, ImageDesc in bytes */
    const int wstride = img.stride / bytes_per_pixel(as);
    return wrapbuffer_handle_t(h, img.width, img.height, wstride, img.height,
                               rga_format(as));
}

} // namespace

Compositor::~Compositor()
{
    close();
}

/* ------------------------------------------------------------------ */
/* buffer registration                                                 */
/* ------------------------------------------------------------------ */
void *Compositor::handle_for(const ImageDesc &img)
{
    for (int i = 0; i < m_slots_used; i++)
        if (m_slots[i].fd == img.dma_fd)
            return m_slots[i].handle;

    if (m_slots_used >= (int)(sizeof m_slots / sizeof m_slots[0])) {
        COMPE("buffer registry full\n");
        return nullptr;
    }
    rga_buffer_handle_t h = importbuffer_fd(img.dma_fd, img.size);
    if (!h) {
        COMPE("importbuffer_fd(%d) failed\n", img.dma_fd);
        return nullptr;
    }
    m_slots[m_slots_used] = { img.dma_fd, (void *)(uintptr_t)h };
    m_slots_used++;
    return (void *)(uintptr_t)h;
}

/* ------------------------------------------------------------------ */
/* lifecycle                                                           */
/* ------------------------------------------------------------------ */
bool Compositor::open(const Config &cfg)
{
    close();
    m_cfg = cfg;

    /* NV12 scratch at full destination size: pass 1 writes only the luma
     * rows of cfg.video into it, so bars and chroma stay as initialised */
    m_scratch_size = (size_t)cfg.width * cfg.height * 3 / 2;

    int heap = ::open("/dev/dma_heap/system", O_RDWR);
    if (heap < 0) {
        COMPE("open dma_heap: %s\n", strerror(errno));
        return false;
    }
    struct dma_heap_allocation_data ad = {};
    ad.len = m_scratch_size;
    ad.fd_flags = O_RDWR | O_CLOEXEC;
    int r = xioctl(heap, DMA_HEAP_IOCTL_ALLOC, &ad);
    ::close(heap);
    if (r < 0) {
        COMPE("DMA_HEAP_IOCTL_ALLOC: %s\n", strerror(errno));
        return false;
    }
    m_scratch_fd = ad.fd;
    m_scratch_va = mmap(nullptr, m_scratch_size, PROT_READ | PROT_WRITE,
                        MAP_SHARED, m_scratch_fd, 0);
    if (m_scratch_va == MAP_FAILED) {
        COMPE("mmap scratch: %s\n", strerror(errno));
        m_scratch_va = nullptr;
        close();
        return false;
    }

    ImageDesc scratch;
    scratch.dma_fd = m_scratch_fd;
    scratch.va = m_scratch_va;
    scratch.size = m_scratch_size;
    scratch.width = cfg.width;
    scratch.height = cfg.height;
    scratch.stride = cfg.width;
    scratch.format = PixelFormat::NV12;
    m_scratch_handle = handle_for(scratch);
    if (!m_scratch_handle) {
        close();
        return false;
    }

    const size_t y_len = (size_t)cfg.width * cfg.height;
    struct dma_buf_sync s = {};
    s.flags = DMA_BUF_SYNC_RW | DMA_BUF_SYNC_START;
    xioctl(m_scratch_fd, DMA_BUF_IOCTL_SYNC, &s);
    memset(m_scratch_va, 0, y_len);                                 /* bars */
    memset((uint8_t *)m_scratch_va + y_len, 128, y_len / 2);      /* chroma */
    s.flags = DMA_BUF_SYNC_RW | DMA_BUF_SYNC_END;
    xioctl(m_scratch_fd, DMA_BUF_IOCTL_SYNC, &s);
    return true;
}

void Compositor::close()
{
    for (int i = 0; i < m_slots_used; i++)
        releasebuffer_handle((rga_buffer_handle_t)(uintptr_t)m_slots[i].handle);
    m_slots_used = 0;
    m_scratch_handle = nullptr;

    if (m_scratch_va) munmap(m_scratch_va, m_scratch_size);
    if (m_scratch_fd >= 0) ::close(m_scratch_fd);
    m_scratch_va = nullptr;
    m_scratch_fd = -1;
    m_scratch_size = 0;
    m_overlay = ImageDesc();
}

bool Compositor::set_overlay(const ImageDesc &overlay)
{
    if (!overlay.valid()) {
        m_overlay = ImageDesc();
        return true;
    }
    if (!handle_for(overlay))
        return false;
    m_overlay = overlay;
    return true;
}

/* ------------------------------------------------------------------ */
/* render                                                              */
/* ------------------------------------------------------------------ */
bool Compositor::render(const ImageDesc &video, const ImageDesc &dst)
{
    if (m_scratch_fd < 0 || !video.valid() || !dst.valid()) return false;

    rga_buffer_handle_t vh = (rga_buffer_handle_t)(uintptr_t)handle_for(video);
    rga_buffer_handle_t dh = (rga_buffer_handle_t)(uintptr_t)handle_for(dst);
    if (!vh || !dh) return false;

    rga_buffer_t src = wrap(vh, video, video.format);
    rga_buffer_t out = wrap(dh, dst, dst.format);

    ImageDesc scratch_img;
    scratch_img.width = m_cfg.width;
    scratch_img.height = m_cfg.height;
    scratch_img.stride = m_cfg.width;
    const rga_buffer_handle_t sh = (rga_buffer_handle_t)(uintptr_t)m_scratch_handle;

    /* pass 1: scale + letterbox into the luma plane only */
    rga_buffer_t luma = wrap(sh, scratch_img, PixelFormat::Gray8);
    const im_rect vrect = { m_cfg.video.x, m_cfg.video.y,
                            m_cfg.video.w, m_cfg.video.h };
    IM_STATUS st = improcess(src, luma, {}, {}, vrect, {}, -1, nullptr, nullptr, IM_SYNC);
    if (st != IM_STATUS_SUCCESS) {
        COMPE("scale: %s\n", imStrError(st));
        return false;
    }

    /* pass 2: full-frame YUV -> RGB */
    rga_buffer_t yuv = wrap(sh, scratch_img, PixelFormat::NV12);
    imsetColorSpace(&yuv, (IM_COLOR_SPACE_MODE)IM_YUV_BT601_FULL_RANGE);
    st = improcess(yuv, out, {}, {}, {}, {}, -1, nullptr, nullptr, IM_SYNC);
    if (st != IM_STATUS_SUCCESS) {
        COMPE("convert: %s\n", imStrError(st));
        return false;
    }

    /* pass 3: overlay */
    if (m_overlay.valid()) {
        rga_buffer_handle_t oh = (rga_buffer_handle_t)(uintptr_t)handle_for(m_overlay);
        rga_buffer_t ovl = wrap(oh, m_overlay, m_overlay.format);
        st = imblend(ovl, out, IM_ALPHA_BLEND_SRC_OVER, 1);
        if (st != IM_STATUS_SUCCESS) {
            COMPE("blend: %s\n", imStrError(st));
            return false;
        }
    }
    return true;
}

} // namespace hw
