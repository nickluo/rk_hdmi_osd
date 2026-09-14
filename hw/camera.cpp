/*
 * hw::MipiCamera implementation - see hw/camera.h for the API contract.
 */
#include "camera.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

namespace hw {
namespace {

#define CAME(...) do { fprintf(stderr, "[E] camera: " __VA_ARGS__); fflush(stderr); } while (0)

int xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do { r = ioctl(fd, req, arg); } while (r < 0 && errno == EINTR);
    return r;
}

/* every VIDIOC_*BUF call on an MPLANE device needs its own plane array */
struct PlaneBuf {
    struct v4l2_plane planes[1] = {};
    struct v4l2_buffer b = {};
    explicit PlaneBuf(int index = -1)
    {
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.memory = V4L2_MEMORY_MMAP;
        b.length = 1;
        b.m.planes = planes;
        if (index >= 0) b.index = index;
    }
};

} // namespace

/* ------------------------------------------------------------------ */
/* Frame                                                               */
/* ------------------------------------------------------------------ */
CameraBase::Frame &CameraBase::Frame::operator=(Frame &&o) noexcept
{
    if (this != &o) {
        release();
        m_cam = o.m_cam;
        m_index = o.m_index;
        m_image = o.m_image;
        o.m_cam = nullptr;
        o.m_index = -1;
        o.m_image = nullptr;
    }
    return *this;
}

const ImageDesc &CameraBase::Frame::image() const
{
    static const ImageDesc empty;
    return m_image ? *m_image : empty;
}

void CameraBase::Frame::release()
{
    if (!m_cam) return;
    m_cam->requeue(m_index);
    m_cam = nullptr;
    m_index = -1;
    m_image = nullptr;
}

/* ------------------------------------------------------------------ */
/* lifecycle                                                           */
/* ------------------------------------------------------------------ */
MipiCamera::~MipiCamera()
{
    close();
}

bool MipiCamera::open(const Config &cfg)
{
    close();

    if (!configure_subdev(cfg))
        return false;

    m_fd = ::open(cfg.video_dev, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (m_fd < 0) {
        CAME("open %s: %s\n", cfg.video_dev, strerror(errno));
        return false;
    }
    if (!set_format(cfg) || !setup_buffers(cfg.buffers) || !stream_on()) {
        close();
        return false;
    }
    fprintf(stdout, "[I] camera: GREY %dx%d stride=%d bufs=%d\n",
            m_w, m_h, stride(), m_nbufs);
    fflush(stdout);
    return true;
}

bool MipiCamera::configure_subdev(const Config &cfg)
{
    char cmd[512];
    snprintf(cmd, sizeof cmd,
             "media-ctl -d %s --set-v4l2 '\"%s\":0[fmt:Y8_1X8/%dx%d@1/%d field:none]'",
             cfg.media_dev, cfg.entity, cfg.width, cfg.height, cfg.fps);
    if (system(cmd) != 0) {
        CAME("media-ctl failed\n");
        return false;
    }
    return true;
}

bool MipiCamera::set_format(const Config &cfg)
{
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = cfg.width;
    fmt.fmt.pix_mp.height = cfg.height;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix_mp.num_planes = 1;
    if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0) {
        CAME("S_FMT: %s\n", strerror(errno));
        return false;
    }
    m_w = fmt.fmt.pix_mp.width;
    m_h = fmt.fmt.pix_mp.height;
    return true;
}

bool MipiCamera::setup_buffers(int count)
{
    struct v4l2_requestbuffers req = {};
    req.count = count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(m_fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        CAME("REQBUFS: %s\n", strerror(errno));
        return false;
    }
    m_nbufs = req.count;
    m_bufs = new ImageDesc[m_nbufs];

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(m_fd, VIDIOC_G_FMT, &fmt) < 0) {
        CAME("G_FMT: %s\n", strerror(errno));
        return false;
    }
    const int stride_bytes = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;

    for (int i = 0; i < m_nbufs; i++) {
        PlaneBuf q(i);
        if (xioctl(m_fd, VIDIOC_QUERYBUF, &q.b) < 0) {
            CAME("QUERYBUF %d: %s\n", i, strerror(errno));
            return false;
        }
        ImageDesc &d = m_bufs[i];
        d.size = q.planes[0].length;
        void *va = mmap(nullptr, d.size, PROT_READ | PROT_WRITE,
                        MAP_SHARED, m_fd, q.planes[0].m.mem_offset);
        if (va == MAP_FAILED) {
            CAME("mmap %d: %s\n", i, strerror(errno));
            return false;
        }
        d.va = va;
        d.width = m_w;
        d.height = m_h;
        d.stride = stride_bytes;
        d.format = PixelFormat::Gray8;

        struct v4l2_exportbuffer e = {};
        e.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        e.index = i;
        e.flags = O_CLOEXEC;
        if (xioctl(m_fd, VIDIOC_EXPBUF, &e) < 0) {
            CAME("EXPBUF %d: %s\n", i, strerror(errno));
            return false;
        }
        d.dma_fd = e.fd;
    }
    return true;
}

bool MipiCamera::stream_on()
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(m_fd, VIDIOC_STREAMON, &type) < 0) {
        CAME("STREAMON: %s\n", strerror(errno));
        return false;
    }
    m_streaming = true;
    for (int i = 0; i < m_nbufs; i++) {
        PlaneBuf q(i);
        if (xioctl(m_fd, VIDIOC_QBUF, &q.b) < 0) {
            CAME("QBUF %d: %s\n", i, strerror(errno));
            return false;
        }
    }
    return true;
}

void MipiCamera::close()
{
    if (m_fd >= 0 && m_streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        xioctl(m_fd, VIDIOC_STREAMOFF, &type);
    }
    m_streaming = false;

    if (m_bufs) {
        for (int i = 0; i < m_nbufs; i++) {
            ImageDesc &d = m_bufs[i];
            if (d.dma_fd >= 0) ::close(d.dma_fd);
            if (d.va) munmap(d.va, d.size);
        }
        delete[] m_bufs;
        m_bufs = nullptr;
    }
    m_nbufs = 0;
    if (m_fd >= 0) ::close(m_fd);
    m_fd = -1;
}

bool MipiCamera::get_image_size(int &width, int &height) const
{
    if (m_fd < 0) return false;
    width = m_w;
    height = m_h;
    return true;
}

/* ------------------------------------------------------------------ */
/* capture                                                             */
/* ------------------------------------------------------------------ */
CameraBase::Frame MipiCamera::capture(int timeout_ms)
{
    if (m_fd < 0) return Frame();

    struct pollfd p = { m_fd, POLLIN, 0 };
    if (poll(&p, 1, timeout_ms) <= 0) {
        CAME("poll timeout\n");
        return Frame();
    }
    PlaneBuf q;
    if (xioctl(m_fd, VIDIOC_DQBUF, &q.b) < 0) {
        CAME("DQBUF: %s\n", strerror(errno));
        return Frame();
    }
    const int index = (int)q.b.index;
    return make_frame(index, &m_bufs[index]);
}

void MipiCamera::requeue(int index)
{
    if (m_fd < 0 || index < 0 || index >= m_nbufs) return;
    PlaneBuf q(index);
    if (xioctl(m_fd, VIDIOC_QBUF, &q.b) < 0)
        CAME("QBUF %d: %s\n", index, strerror(errno));
}

} // namespace hw
