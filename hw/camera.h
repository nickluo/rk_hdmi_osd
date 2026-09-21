/*
 * hw::CameraBase - capture interface; hw::MipiCamera - MIPI CSI (rkcif)
 * implementation using V4L2 MMAP buffers exported as dmabuf.
 *
 * Frames are handed out as borrowed dmabuf descriptors, never copied. No
 * vendor 2D-engine type appears here: what consumes the dmabuf is the
 * consumer's business.
 *
 * Three capture paths are known:
 *   Ar0234M     - VEYE RAW-MIPI-AR0234M behind the vendor "mvcam" subdev,
 *                 Y8_1X8 on the wire, captured as V4L2_PIX_FMT_GREY.
 *   Sc233hgs    - in-tree Rockchip driver (HZ-RK3576 BSP), mono 2-lane mode
 *                 1920x1200@60, direct rkcif capture as V4L2_PIX_FMT_Y10
 *                 (LE packed bitstream). The rkcif driver stamps every
 *                 buffer's SOF in CLOCK_MONOTONIC_RAW.
 *   Sc233hgsIsp - same sensor through the rkisp pipeline (rkisp-virN,
 *                 /dev/mediaN -> rkisp_mainpath), output NV12: the ISP
 *                 converts the mono RAW to YUV, which removes the CPU
 *                 Y10 unpacking entirely. NOTE: rkisp stamps buffers at
 *                 frame END in CLOCK_MONOTONIC (identical to
 *                 MONOTONIC_RAW while no NTP daemon runs).
 *
 * Every dequeued Frame carries its timestamp in both domains: the raw
 * value the driver stamped (sof_raw_ns) and its CLOCK_REALTIME mapping
 * (epoch_ns), produced by hw::Sof2Epoch. All-zero timestamps mean the
 * driver delivered none.
 */
#pragma once

#include <cstdint>

#include "image.h"
#include "sof2epoch.h"

namespace hw {

class CameraBase {
public:
    enum class Sensor { Ar0234M, Sc233hgs, Sc233hgsIsp };

    struct Config {
        Sensor sensor = Sensor::Sc233hgs;
        /* SC233HGS module selector: 0 = m00_b_sc233hgs (i2c0),
         *                            1 = m01_b_sc233hgs (i2c4) */
        int camera_index = 0;

        /* NULL = derived from the sensor: Ar0234M keeps the fixed mvcam
         * defaults, Sc233hgs resolves both from the media graph (the
         * media device containing the sensor entity, and the
         * stream_cif_mipi_id0 node of that graph) */
        const char *video_dev = nullptr;
        const char *media_dev = nullptr;
        const char *entity    = nullptr;

        int width  = 1920;
        int height = 1200;
        int fps    = 60;
        int buffers = 4;
    };

    /* Borrowed capture buffer, returned to the driver queue on destruction
     * so an early break out of the render loop cannot starve the pipeline.
     * Call release() to hand it back as soon as the consumer is done.
     *
     * Timestamps (valid after capture()):
     *   sof_raw_ns - frame start as stamped by the rkcif SOF interrupt,
     *                CLOCK_MONOTONIC_RAW domain, nanoseconds (the V4L2
     *                usec timeval is widened losslessly)
     *   epoch_ns   - sof_raw_ns mapped to CLOCK_REALTIME; 0 when the
     *                clock is not running or the driver sent no timestamp
     *   sequence   - driver frame sequence number */
    class Frame {
    public:
        Frame() = default;
        ~Frame() { release(); }
        Frame(Frame &&o) noexcept
            : sof_raw_ns(o.sof_raw_ns), epoch_ns(o.epoch_ns),
              sequence(o.sequence),
              m_cam(o.m_cam), m_index(o.m_index), m_image(o.m_image)
        { o.m_cam = nullptr; o.m_index = -1; o.m_image = nullptr;
          o.sof_raw_ns = o.epoch_ns = 0; o.sequence = 0; }
        Frame &operator=(Frame &&o) noexcept;
        Frame(const Frame &) = delete;
        Frame &operator=(const Frame &) = delete;

        bool valid() const { return m_cam != nullptr; }
        int  index() const { return m_index; }
        const ImageDesc &image() const;
        void release();

        uint64_t sof_raw_ns = 0;
        uint64_t epoch_ns = 0;
        uint32_t sequence = 0;

    private:
        friend class CameraBase;
        Frame(CameraBase *cam, int index, const ImageDesc *img)
            : m_cam(cam), m_index(index), m_image(img) {}

        CameraBase *m_cam = nullptr;
        int m_index = -1;
        const ImageDesc *m_image = nullptr;
    };

    virtual ~CameraBase() = default;

    virtual bool open(const Config &cfg) = 0;
    virtual void close() = 0;
    virtual bool is_running() const = 0;

    /* blocks up to timeout_ms; an invalid Frame means timeout or error */
    virtual Frame capture(int timeout_ms = 1000) = 0;

    virtual bool get_image_size(int &width, int &height) const = 0;

protected:
    Frame make_frame(int index, const ImageDesc *img,
                     uint64_t sof_raw_ns, uint64_t epoch_ns, uint32_t sequence)
    {
        Frame f(this, index, img);
        f.sof_raw_ns = sof_raw_ns;
        f.epoch_ns = epoch_ns;
        f.sequence = sequence;
        return f;
    }

    virtual void requeue(int index) = 0;
};

class MipiCamera : public CameraBase {
public:
    MipiCamera() = default;
    ~MipiCamera() override;
    MipiCamera(const MipiCamera &) = delete;
    MipiCamera &operator=(const MipiCamera &) = delete;

    bool open(const Config &cfg) override;
    void close() override;
    bool is_running() const override { return m_streaming; }
    Frame capture(int timeout_ms = 1000) override;
    bool get_image_size(int &width, int &height) const override;

    /* the negotiated stride is 256-aligned and differs from the width */
    int stride() const { return m_nbufs ? m_bufs[0].stride : 0; }
    int buffer_count() const { return m_nbufs; }

    /* the SOF(CLOCK_MONOTONIC_RAW)->Epoch converter backing Frame::epoch_ns */
    const Sof2Epoch &clock() const { return m_clock; }
    Sof2Epoch &clock() { return m_clock; }

protected:
    void requeue(int index) override;

private:
    bool configure_subdev(const Config &cfg);
    bool set_format(const Config &cfg);
    bool setup_buffers(int count);
    bool stream_on();

    int m_fd = -1;
    int m_nbufs = 0;
    ImageDesc *m_bufs = nullptr;
    int m_w = 0, m_h = 0;
    bool m_streaming = false;
    uint32_t m_fourcc = 0;
    PixelFormat m_pf = PixelFormat::Gray8;
    char m_media_dev[64] = {};
    char m_entity[64] = {};
    char m_video_dev[32] = {};
    bool m_ts_warned = false;
    Sof2Epoch m_clock;
};

} // namespace hw
