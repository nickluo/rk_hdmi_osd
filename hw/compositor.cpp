/*
 * hw::Compositor implementation - see hw/compositor.h for the rationale.
 */
#include "compositor.h"

#include <cstdio>
#include <cstring>
#include <cstdint>
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
    case PixelFormat::Y10:      /* never wrapped: render() down-converts Y10 */
    case PixelFormat::NV12:     return RK_FORMAT_YCbCr_420_SP;
    case PixelFormat::BGRX8888: return RK_FORMAT_BGRX_8888;
    case PixelFormat::RGBA8888: return RK_FORMAT_RGBA_8888;
    }
    return RK_FORMAT_YCbCr_400;
}

/* rkcif (RK3576) writes V4L2 'Y10 ' as a continuous little-endian bit
 * stream: within every byte the LSB comes first, each sample consumes 10
 * bits. Per 5-byte group (4 samples) that reduces to:
 *   p0 = b0      | (b1 & 0x03) << 8
 *   p1 = b1 >> 2 | (b2 & 0x0f) << 6
 *   p2 = b2 >> 4 | (b3 & 0x3f) << 4
 *   p3 = b3 >> 6 |  b4         << 2
 * Verified on hardware by decoding a live frame (recognisable scene,
 * smooth gradients; the MIPI-style "2-bit tail byte" layout decodes to
 * noise). Row stride is ALIGN(w*10/8, 256); the caller advances by it. */
void y10_row_to_y8(const uint8_t *src, uint8_t *dst, int n)
{
    for (int g = n / 4; g > 0; g--, src += 5, dst += 4) {
        const uint32_t p0 = src[0] | ((src[1] & 0x03u) << 8);
        const uint32_t p1 = (src[1] >> 2) | ((src[2] & 0x0fu) << 6);
        const uint32_t p2 = (src[2] >> 4) | ((src[3] & 0x3fu) << 4);
        const uint32_t p3 = (src[3] >> 6) | ((uint32_t)src[4] << 2);
        dst[0] = (uint8_t)(p0 >> 2);
        dst[1] = (uint8_t)(p1 >> 2);
        dst[2] = (uint8_t)(p2 >> 2);
        dst[3] = (uint8_t)(p3 >> 2);
    }
    for (int i = n & ~3; i < n; i++)    /* tail when width%4 != 0 */
        dst[i] = (uint8_t)(src[i] >> 2);
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

    if (m_y8_va) munmap(m_y8_va, m_y8_size);
    if (m_y8_fd >= 0) ::close(m_y8_fd);
    m_y8_va = nullptr;
    m_y8_fd = -1;
    m_y8_size = 0;
    m_y8_w = m_y8_h = m_y8_stride = 0;

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
/* Y10 -> Gray8 down-conversion (CPU; RGA has no 10-bit luma input)    */
/* ------------------------------------------------------------------ */
bool Compositor::ensure_y8_scratch_(int width, int height)
{
    if (m_y8_fd >= 0 && m_y8_w == width && m_y8_h == height)
        return true;
    if (m_y8_va) munmap(m_y8_va, m_y8_size);
    if (m_y8_fd >= 0) ::close(m_y8_fd);
    m_y8_va = nullptr;
    m_y8_fd = -1;

    m_y8_w = width;
    m_y8_h = height;
    m_y8_stride = (width + 15) & ~15;    /* RGA wants an aligned stride */
    m_y8_size = (size_t)m_y8_stride * m_y8_h;

    int heap = ::open("/dev/dma_heap/system", O_RDWR);
    if (heap < 0) {
        COMPE("open dma_heap: %s\n", strerror(errno));
        return false;
    }
    struct dma_heap_allocation_data ad = {};
    ad.len = m_y8_size;
    ad.fd_flags = O_RDWR | O_CLOEXEC;
    int r = xioctl(heap, DMA_HEAP_IOCTL_ALLOC, &ad);
    ::close(heap);
    if (r < 0) {
        COMPE("DMA_HEAP_IOCTL_ALLOC y8: %s\n", strerror(errno));
        return false;
    }
    m_y8_fd = ad.fd;
    m_y8_va = mmap(nullptr, m_y8_size, PROT_READ | PROT_WRITE,
                   MAP_SHARED, m_y8_fd, 0);
    if (m_y8_va == MAP_FAILED) {
        COMPE("mmap y8: %s\n", strerror(errno));
        m_y8_va = nullptr;
        return false;
    }

    ImageDesc y8;
    y8.dma_fd = m_y8_fd;
    y8.va = m_y8_va;
    y8.size = m_y8_size;
    y8.width = m_y8_w;
    y8.height = m_y8_h;
    y8.stride = m_y8_stride;
    y8.format = PixelFormat::Gray8;
    if (!handle_for(y8))
        return false;
    return true;
}

void Compositor::y10_to_y8_(const ImageDesc &src)
{
    struct dma_buf_sync s = {};

    /* the DMA engine wrote the Y10 buffer; make the CPU view coherent */
    s.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
    xioctl(src.dma_fd, DMA_BUF_IOCTL_SYNC, &s);
    s.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
    xioctl(m_y8_fd, DMA_BUF_IOCTL_SYNC, &s);

    const int w = m_y8_w;
    const uint8_t *srow = (const uint8_t *)src.va;
    uint8_t *drow = (uint8_t *)m_y8_va;
    for (int y = 0; y < m_y8_h; y++) {
        y10_row_to_y8(srow, drow, w);
        srow += src.stride;
        drow += m_y8_stride;
    }

    s.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    xioctl(src.dma_fd, DMA_BUF_IOCTL_SYNC, &s);
    s.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
    xioctl(m_y8_fd, DMA_BUF_IOCTL_SYNC, &s);
}

/* ------------------------------------------------------------------ */
/* render                                                              */
/* ------------------------------------------------------------------ */
bool Compositor::render(const ImageDesc &video, const ImageDesc &dst)
{
    if (m_scratch_fd < 0 || !video.valid() || !dst.valid()) return false;

    /* RGA cannot ingest 10-bit luma: down-convert once per frame on the
     * CPU into the internal Gray8 scratch and run the usual passes on it */
    ImageDesc video_src = video;
    if (video.format == PixelFormat::Y10) {
        if (!ensure_y8_scratch_(video.width, video.height))
            return false;
        y10_to_y8_(video);
        video_src.dma_fd = m_y8_fd;
        video_src.va = m_y8_va;
        video_src.size = m_y8_size;
        video_src.stride = m_y8_stride;
        video_src.format = PixelFormat::Gray8;
    }

    rga_buffer_handle_t vh = (rga_buffer_handle_t)(uintptr_t)handle_for(video_src);
    rga_buffer_handle_t dh = (rga_buffer_handle_t)(uintptr_t)handle_for(dst);
    if (!vh || !dh) return false;

    rga_buffer_t src = wrap(vh, video_src, video_src.format);
    rga_buffer_t out = wrap(dh, dst, dst.format);

    ImageDesc scratch_img;
    scratch_img.width = m_cfg.width;
    scratch_img.height = m_cfg.height;
    scratch_img.stride = m_cfg.width;
    const rga_buffer_handle_t sh = (rga_buffer_handle_t)(uintptr_t)m_scratch_handle;

    /* pass 1: scale + letterbox.
     * NV12 input (rkisp path): scale both planes into the letterbox rect
     * of the scratch; bars stay as initialised. Mono/gray input: Y400
     * writes only the luma rows, the chroma plane is pre-filled neutral */
    const im_rect vrect = { m_cfg.video.x, m_cfg.video.y,
                            m_cfg.video.w, m_cfg.video.h };
    IM_STATUS st;
    if (video_src.format == PixelFormat::NV12) {
        rga_buffer_t yuv = wrap(sh, scratch_img, PixelFormat::NV12);
        st = improcess(src, yuv, {}, {}, {}, vrect, -1, nullptr, nullptr, IM_SYNC);
    } else {
        rga_buffer_t luma = wrap(sh, scratch_img, PixelFormat::Gray8);
        st = improcess(src, luma, {}, {}, vrect, {}, -1, nullptr, nullptr, IM_SYNC);
    }
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
