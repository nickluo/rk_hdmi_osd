/*
 * osd::layout - Betaflight-style element layout engine on top of osd::core.
 *
 * Depends ONLY on osd::core (pure CPU rasteriser): buildable and testable
 * on any host, no libdrm/librga. The caller owns the layers and the frame
 * loop; Layout fills two canvases:
 *
 *   hud - HUD layer (alpha-binary, RGA-blended over the video on the
 *        primary plane): static frame + change-driven text slots
 *   det - detection layer (DRM overlay plane, VOP blends at scanout):
 *        BBoxRect boxes at image rate
 *
 * Cadence (mirrors the proven osd_demo loop at 60 fps):
 *   text slots   ~10 Hz, change-driven with static-base restore
 *   home arrow   ~20 Hz, region restore + redraw
 *   warnings     blink via Layer::blink_phase() (~3.3 Hz)
 *   boxes        every frame, exact-footprint clear -> draw -> record
 */
#pragma once

#include <vector>
#include "telemetry.h"
#include "osd/osd.h"

namespace osd {
namespace layout {

struct Config {
    const char *craft_name = "FPV";     /* ASCII only */
    bool crosshair   = true;            /* center reticle (static) */
    bool corner_brackets = true;        /* frame corner brackets (static) */
    bool flymode_el  = true;
    bool tracker_el  = true;            /* tracker status banner */
    bool timer_el    = true;            /* T+ mm:ss since arm */
    bool voltage_el  = true;            /* pack V + per-cell V */
    bool current_el  = true;
    bool mah_el      = true;            /* mAh drawn + usage bar */
    bool alt_el      = true;            /* baro altitude */
    bool speed_el    = true;            /* ground speed km/h */
    bool heading_el  = true;
    bool sats_el     = true;            /* "SAT 12 3D" */
    bool temp_el     = true;
    bool clock_el    = true;            /* wall clock from epoch_ns */
    bool frame_el    = true;            /* frame counter (debug, small font) */
    bool warnings_el = true;            /* FAILSAFE / CRITICAL / LOW BATT */
    bool armed_el    = true;            /* ARMED / DISARMED banner */
    bool horizon_el  = true;            /* pitch-ladder artificial horizon */
    bool message_el  = true;            /* system/custom message line */
    bool home_el     = true;            /* home arrow + distance */
};

/* camera-space -> canvas-space map (letterbox). Pure math so the bridge
 * stays geometry-free. Mirrors osd_demo's mapping:
 *   x_osd = video.x + x_cam * video.w / cam_w
 *   y_osd = video.y + y_cam * video.h / cam_h  (rounded) */
struct CanvasMap {
    int  cam_w = 1920, cam_h = 1200;
    Rect video{0, 14, 720, 450};        /* letterbox rect on the canvas */

    Rect operator()(const Rect &r) const
    {
        Rect o;
        o.x = video.x + (r.x * video.w + cam_w / 2) / cam_w;
        o.y = video.y + (r.y * video.h + cam_h / 2) / cam_h;
        o.w = (r.w * video.w + cam_w / 2) / cam_w;
        o.h = (r.h * video.h + cam_h / 2) / cam_h;
        if (o.w < 1) o.w = 1;
        if (o.h < 1) o.h = 1;
        return o;
    }
};

/* per-frame render context supplied by the caller's loop */
struct FrameCtx {
    uint32_t frame = 0;     /* layer frame counter (drives cadence + blink) */
    double   t_s = 0.0;     /* seconds since render start (animations) */
};

class Layout {
public:
    bool init(int canvas_w, int canvas_h, const Config &cfg = Config());

    /* draw the static elements once after Layer init; the caller then
     * snapshots with hud.set_base() (exactly like osd_demo) */
    void draw_static(osd::Layer &hud);

    /* dynamic elements for this frame. `boxes` are in CANVAS pixels
     * (apply CanvasMap before calling). Internal cadence per header. */
    void update(const Telemetry &t, const std::vector<osd::BBoxRect> &boxes,
                osd::Layer &hud, osd::Layer &det, const FrameCtx &f);

    int width()  const { return m_w; }
    int height() const { return m_h; }

private:
    /* change-driven text slot (ported from osd_demo): redraws only when the
     * string changes, restoring the static base underneath first */
    struct Slot {
        int x = 0, y = 0;
        char last[96] = {};
        int  last_w = 0;
    };
    void slot_draw(osd::Layer &l, Slot &s, const char *text, Color c,
                   bool small = false);
    void update_boxes(const std::vector<osd::BBoxRect> &boxes, osd::Layer &det);
    const char *tracker_text(int8_t state, Color &c) const;

    Config m_cfg;
    int m_w = 0, m_h = 0;

    Slot m_slot_mode, m_slot_tracker, m_slot_timer;
    Slot m_slot_batt, m_slot_cell, m_slot_cur, m_slot_mah;
    Slot m_slot_alt, m_slot_spd, m_slot_hdg, m_slot_sat, m_slot_tmp;
    Slot m_slot_home, m_slot_time, m_slot_frame, m_slot_warn;
    Slot m_slot_arm, m_slot_msg;

    Rect m_arrow_region;               /* home-arrow restore region */
    Rect m_horizon_region;             /* attitude-ladder restore region */
    int  m_bar_y = 0;                  /* battery usage bar row */
    float m_last_usage = -1.f;
    std::vector<Rect> m_prev_foot;     /* box clear footprints (last frame) */
    uint32_t m_last_trig_seq = 0;      /* trigger event detection */
    double   m_trig_until_s = -1.0;    /* trigger banner visible until */
};

} // namespace osd::layout
} // namespace osd
