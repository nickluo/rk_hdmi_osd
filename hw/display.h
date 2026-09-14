/*
 * hw::HdmiDisplay - DRM/KMS output on the HDMI connector, double buffered
 * with page flips, plus an optional overlay plane.
 *
 * The primary plane holds video + HUD (composed by RGA into the back
 * buffer). The overlay plane carries the detection layer: the VOP blends it
 * at scanout with plane-global x per-pixel alpha, which is exact and costs
 * neither CPU nor RGA time - RGA2-E's own blend is unusable for alpha<255.
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <xf86drmMode.h>
#include "image.h"

namespace hw {

class HdmiDisplay {
public:
    struct Config {
        const char *dev = "/dev/dri/card0";
        int width  = 720;           /* NTSC progressive */
        int height = 480;
        bool overlay = true;        /* allocate the detection-layer plane */
        int  overlay_alpha = 235;   /* plane global alpha, set once */
    };

    HdmiDisplay() = default;
    ~HdmiDisplay();
    HdmiDisplay(const HdmiDisplay &) = delete;
    HdmiDisplay &operator=(const HdmiDisplay &) = delete;

    bool open(const Config &cfg);
    void close();
    bool valid() const { return m_fd >= 0; }

    int width()  const { return m_w; }
    int height() const { return m_h; }
    int pitch()  const { return m_pitch; }

    /* the buffer being composed this frame */
    ImageDesc back_image() const;
    void  *back_pixels() const { return m_fb[m_back].map; }
    size_t back_size()   const { return m_fb[m_back].map_size; }

    /* queue the back buffer for the next vblank, then swap */
    bool present();
    bool wait_flip(int timeout_ms);
    bool flip_pending() const { return m_flip_pending; }

    /* overlay plane buffer, for an OSD layer to draw into directly */
    bool has_overlay() const { return m_ovl_plane_id != 0; }
    ImageDesc overlay_image() const;

private:
    struct Framebuffer {
        uint32_t fb_id = 0, gem_handle = 0;
        int prime_fd = -1;
        uint32_t *map = nullptr;
        size_t map_size = 0;
    };

    bool pick_connector_and_mode();
    bool create_fb(Framebuffer &fb, uint32_t fourcc);
    void destroy_fb(Framebuffer &fb);
    bool setup_overlay(const Config &cfg);

    int m_fd = -1;
    uint32_t m_conn_id = 0, m_crtc_id = 0;
    int m_crtc_index = 0;
    drmModeModeInfo m_mode = {};

    Framebuffer m_fb[2];
    int m_pitch = 0, m_w = 0, m_h = 0;
    int m_back = 1;                 /* SetCrtc scans out m_fb[0] first */
    bool m_flip_pending = false;

    uint32_t m_ovl_plane_id = 0;
    Framebuffer m_ovl;
};

} // namespace hw
