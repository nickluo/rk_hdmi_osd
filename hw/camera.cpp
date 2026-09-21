/*
 * hw::MipiCamera implementation - see hw/camera.h for the API contract.
 */
#include "camera.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
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

/* ------------------------------------------------------------------ */
/* SC233HGS pipeline discovery                                         */
/*                                                                     */
/* Each rkcif MIPI host owns one media device whose graph runs          */
/* sensor -> dphy -> csi2 -> stream_cif_mipi_idN. Scanning /dev/media*  */
/* for the sensor entity of the wanted module both picks the right      */
/* media device and yields the id0 capture node of that graph.          */
/* ------------------------------------------------------------------ */
bool mediactl_text(const char *media_dev, std::string *out)
{
    char cmd[128];
    snprintf(cmd, sizeof cmd, "media-ctl -d %s -p 2>/dev/null", media_dev);
    FILE *f = popen(cmd, "r");
    if (!f)
        return false;
    char buf[4096];
    size_t n;
    out->clear();
    while ((n = fread(buf, 1, sizeof buf, f)) > 0)
        out->append(buf, n);
    pclose(f);
    return !out->empty();
}

/* "- entity 22: m00_b_sc233hgs 0-0030 (1 pad, 1 link)" -> full entity
 * name between ": " and " (" */
bool entity_name_at(const std::string &text, size_t pos,
                    char *name, size_t namesz)
{
    size_t b = text.find(": ", pos);
    if (b == std::string::npos)
        return false;
    b += 2;
    size_t e = text.find(" (", b);
    if (e == std::string::npos)
        return false;
    if (e - b >= namesz)
        return false;
    memcpy(name, text.data() + b, e - b);
    name[e - b] = '\0';
    return true;
}

/* the /dev/videoN node name is the "device node name" line of the block
 * that follows the entity header line */
bool devnode_after(const std::string &text, size_t pos,
                   char *dev, size_t devsz)
{
    size_t d = text.find("device node name ", pos);
    size_t blk_end = text.find("- entity ", pos + 1);
    if (d == std::string::npos)
        return false;
    if (blk_end != std::string::npos && d > blk_end)
        return false;    /* belongs to the next entity */
    size_t b = d + strlen("device node name ");
    size_t e = text.find('\n', b);
    if (e == std::string::npos || e - b >= devsz)
        return false;
    memcpy(dev, text.data() + b, e - b);
    dev[e - b] = '\0';
    return true;
}

bool discover_sc233hgs(int camera_index, const char *media_hint,
                       char *entity, size_t esz,
                       char *video_dev, size_t vsz,
                       char *media_dev, size_t msz)
{
    char prefix[40];
    snprintf(prefix, sizeof prefix, "m%02d_b_sc233hgs", camera_index);

    for (int i = 0; i < 8; i++) {
        char md[16];
        if (media_hint && media_hint[0])
            snprintf(md, sizeof md, "%s", media_hint);
        else
            snprintf(md, sizeof md, "/dev/media%d", i);

        std::string text;
        if (!mediactl_text(md, &text))
            continue;

        /* sensor entity of this module in this graph? */
        size_t pos = 0, sensor = std::string::npos;
        while ((pos = text.find("- entity ", pos)) != std::string::npos) {
            size_t nl = text.find('\n', pos);
            if (nl == std::string::npos)
                break;
            if (text.find(prefix, pos) != std::string::npos &&
                text.find(prefix, pos) < nl) {
                sensor = pos;
                break;
            }
            pos = nl;
        }
        if (sensor == std::string::npos)
            continue;
        if (!entity_name_at(text, sensor, entity, esz))
            continue;

        /* capture node: stream_cif_mipi_id0 of the same graph */
        pos = 0;
        char node[32] = "";
        while ((pos = text.find("stream_cif_mipi_id0", pos)) !=
               std::string::npos) {
            size_t hdr = text.rfind("- entity ", pos);
            size_t hdr_nl = (hdr == std::string::npos)
                          ? std::string::npos : text.find('\n', hdr);
            char node_entity[64];
            if (hdr != std::string::npos && pos < hdr_nl &&
                entity_name_at(text, hdr, node_entity, sizeof node_entity) &&
                strcmp(node_entity, "stream_cif_mipi_id0") == 0 &&
                devnode_after(text, hdr_nl, node, sizeof node))
                break;
            pos += strlen("stream_cif_mipi_id0");
        }
        if (!node[0]) {
            CAME("%s: no stream_cif_mipi_id0 node in %s\n", entity, md);
            continue;
        }

        snprintf(media_dev, msz, "%s", md);
        snprintf(video_dev, vsz, "%s", node);
        return true;
    }
    return false;
}

/* rkisp pipeline: /dev/mediaN with driver "rkisp-vir{idx}" contains the
 * rkisp_mainpath NV12 capture node fed by the same sensor */
bool discover_rkisp(int camera_index, char *video_dev, size_t vsz,
                    char *media_dev, size_t msz)
{
    char driver[24];
    snprintf(driver, sizeof driver, "rkisp-vir%d", camera_index);

    for (int i = 0; i < 8; i++) {
        char md[16];
        snprintf(md, sizeof md, "/dev/media%d", i);
        std::string text;
        if (!mediactl_text(md, &text))
            continue;
        if (text.find(driver) == std::string::npos)
            continue;

        size_t pos = 0;
        char node[32] = "";
        while ((pos = text.find("rkisp_mainpath", pos)) !=
               std::string::npos) {
            size_t hdr = text.rfind("- entity ", pos);
            size_t hdr_nl = (hdr == std::string::npos)
                          ? std::string::npos : text.find('\n', hdr);
            char ent[64];
            if (hdr != std::string::npos && pos < hdr_nl &&
                entity_name_at(text, hdr, ent, sizeof ent) &&
                strcmp(ent, "rkisp_mainpath") == 0 &&
                devnode_after(text, hdr_nl, node, sizeof node))
                break;
            pos += strlen("rkisp_mainpath");
        }
        if (!node[0])
            continue;
        snprintf(media_dev, msz, "%s", md);
        snprintf(video_dev, vsz, "%s", node);
        return true;
    }
    return false;
}

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
        sof_raw_ns = o.sof_raw_ns;
        epoch_ns = o.epoch_ns;
        sequence = o.sequence;
        o.m_cam = nullptr;
        o.m_index = -1;
        o.m_image = nullptr;
        o.sof_raw_ns = o.epoch_ns = 0;
        o.sequence = 0;
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
    sof_raw_ns = epoch_ns = 0;
    sequence = 0;
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

    m_fd = ::open(m_video_dev, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (m_fd < 0) {
        CAME("open %s: %s\n", m_video_dev, strerror(errno));
        return false;
    }
    if (!set_format(cfg) || !setup_buffers(cfg.buffers) || !stream_on()) {
        close();
        return false;
    }

    if (!m_clock.start())
        CAME("clock not running: frame epoch_ns will stay 0\n");

    fprintf(stdout,
            "[I] camera: %c%c%c%c %dx%d stride=%d bufs=%d via %s (%s)\n",
            m_fourcc & 0xff, (m_fourcc >> 8) & 0xff,
            (m_fourcc >> 16) & 0xff, (m_fourcc >> 24) & 0xff,
            m_w, m_h, stride(), m_nbufs, m_video_dev, m_entity);
    fflush(stdout);
    return true;
}

bool MipiCamera::configure_subdev(const Config &cfg)
{
    const char *mbus;
    char cmd[512];

    if (cfg.sensor == Sensor::Ar0234M) {
        mbus = "Y8_1X8";
        m_fourcc = V4L2_PIX_FMT_GREY;
        m_pf = PixelFormat::Gray8;
        snprintf(m_media_dev, sizeof m_media_dev, "%s",
                 cfg.media_dev ? cfg.media_dev : "/dev/media0");
        snprintf(m_entity, sizeof m_entity, "%s",
                 cfg.entity ? cfg.entity : "m00_b_mvcam 4-003b");
        snprintf(m_video_dev, sizeof m_video_dev, "%s",
                 cfg.video_dev ? cfg.video_dev : "/dev/video0");
    } else if (cfg.sensor == Sensor::Sc233hgsIsp) {
        /* NV12 from the rkisp mainpath: formats propagate through the
         * cif -> sditf -> isp chain at stream-on, no media-ctl needed */
        mbus = nullptr;
        m_fourcc = V4L2_PIX_FMT_NV12;
        m_pf = PixelFormat::NV12;
        snprintf(m_entity, sizeof m_entity, "rkisp_mainpath");
        if (cfg.video_dev) {
            snprintf(m_video_dev, sizeof m_video_dev, "%s", cfg.video_dev);
            snprintf(m_media_dev, sizeof m_media_dev, "%s",
                     cfg.media_dev ? cfg.media_dev : "/dev/media0");
        } else if (!discover_rkisp(cfg.camera_index,
                                   m_video_dev, sizeof m_video_dev,
                                   m_media_dev, sizeof m_media_dev)) {
            CAME("rkisp-vir%d mainpath not found in any media graph\n",
                 cfg.camera_index);
            return false;
        }
        return true;    /* no sensor-pad format to set */
    } else {
        mbus = "Y10_1X10";
        m_fourcc = V4L2_PIX_FMT_Y10;
        m_pf = PixelFormat::Y10;
        if (cfg.entity && cfg.video_dev) {
            snprintf(m_entity, sizeof m_entity, "%s", cfg.entity);
            snprintf(m_video_dev, sizeof m_video_dev, "%s", cfg.video_dev);
            snprintf(m_media_dev, sizeof m_media_dev, "%s",
                     cfg.media_dev ? cfg.media_dev : "/dev/media0");
        } else if (!discover_sc233hgs(cfg.camera_index, cfg.media_dev,
                                      m_entity, sizeof m_entity,
                                      m_video_dev, sizeof m_video_dev,
                                      m_media_dev, sizeof m_media_dev)) {
            CAME("sc233hgs module %d not found in any media graph\n",
                 cfg.camera_index);
            return false;
        }
    }

    snprintf(cmd, sizeof cmd,
             "media-ctl -d %s --set-v4l2 '\"%s\":0[fmt:%s/%dx%d@1/%d field:none]'",
             m_media_dev, m_entity, mbus, cfg.width, cfg.height, cfg.fps);
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
    fmt.fmt.pix_mp.pixelformat = m_fourcc;
    fmt.fmt.pix_mp.num_planes = 1;
    if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0) {
        CAME("S_FMT %c%c%c%c: %s\n",
             m_fourcc & 0xff, (m_fourcc >> 8) & 0xff,
             (m_fourcc >> 16) & 0xff, (m_fourcc >> 24) & 0xff,
             strerror(errno));
        return false;
    }
    if (fmt.fmt.pix_mp.pixelformat != m_fourcc) {
        uint32_t f = fmt.fmt.pix_mp.pixelformat;
        CAME("driver refused Y10 format, negotiated %c%c%c%c\n",
             f & 0xff, (f >> 8) & 0xff, (f >> 16) & 0xff, (f >> 24) & 0xff);
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
        d.format = m_pf;

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
    m_clock.stop();

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
    m_ts_warned = false;
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

    /* SOF timestamp as stamped by the rkcif frame-start interrupt:
     * CLOCK_MONOTONIC_RAW domain, usec timeval widened to ns. */
    const uint64_t sof_ns =
        (uint64_t)q.b.timestamp.tv_sec * 1000000000ull +
        (uint64_t)q.b.timestamp.tv_usec * 1000ull;
    uint64_t epoch_ns = 0;
    if (sof_ns)
        epoch_ns = (uint64_t)m_clock.to_epoch_ns((int64_t)sof_ns);

    if (!m_ts_warned &&
        !(q.b.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)) {
        CAME("driver buffer flags 0x%x lack TIMESTAMP_MONOTONIC; "
             "timestamps may not be SOF/CLOCK_MONOTONIC_RAW\n", q.b.flags);
        m_ts_warned = true;
    }

    return make_frame(index, &m_bufs[index], sof_ns, epoch_ns, q.b.sequence);
}

void MipiCamera::requeue(int index)
{
    if (m_fd < 0 || index < 0 || index >= m_nbufs) return;
    PlaneBuf q(index);
    if (xioctl(m_fd, VIDIOC_QBUF, &q.b) < 0)
        CAME("QBUF %d: %s\n", index, strerror(errno));
}

} // namespace hw
