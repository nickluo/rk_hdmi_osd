/*
 * hw::CameraBase - capture interface; hw::MipiCamera - MIPI CSI (rkcif)
 * implementation using V4L2 MMAP buffers exported as dmabuf.
 *
 * Frames are handed out as borrowed dmabuf descriptors, never copied. No
 * vendor 2D-engine type appears here: what consumes the dmabuf is the
 * consumer's business.
 */
#pragma once

#include "image.h"

namespace hw {

class CameraBase {
public:
    struct Config {
        const char *video_dev = "/dev/video0";
        const char *media_dev = "/dev/media0";
        const char *entity    = "m00_b_mvcam 4-003b";
        int width  = 1920;
        int height = 1200;
        int fps    = 60;
        int buffers = 4;
    };

    /* Borrowed capture buffer, returned to the driver queue on destruction
     * so an early break out of the render loop cannot starve the pipeline.
     * Call release() to hand it back as soon as the consumer is done. */
    class Frame {
    public:
        Frame() = default;
        ~Frame() { release(); }
        Frame(Frame &&o) noexcept
            : m_cam(o.m_cam), m_index(o.m_index), m_image(o.m_image)
        { o.m_cam = nullptr; o.m_index = -1; o.m_image = nullptr; }
        Frame &operator=(Frame &&o) noexcept;
        Frame(const Frame &) = delete;
        Frame &operator=(const Frame &) = delete;

        bool valid() const { return m_cam != nullptr; }
        int  index() const { return m_index; }
        const ImageDesc &image() const;
        void release();

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
    Frame make_frame(int index, const ImageDesc *img) { return Frame(this, index, img); }
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
};

} // namespace hw
