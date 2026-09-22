/*
 * osd::layout implementation - see layout/layout.h.
 *
 * Element positions and the slot/restore/footprint mechanics are ported
 * from the proven osd_demo loop; this file adds the Betaflight element
 * vocabulary driven by Telemetry instead of simulated data.
 */
#include "layout.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <cmath>
#include <algorithm>

namespace osd {
namespace layout {

/* ------------------------------------------------------------------ */
/* change-driven text slot (ported from osd_demo, footprint-hardened)  */
/* ------------------------------------------------------------------ */
void Layout::slot_draw(osd::Layer &l, Slot &s, const char *text, Color c,
                       bool small)
{
    if (strcmp(text, s.last) == 0)
        return;
    TextStyle st;
    st.color = c;
    st.small = small;
    /* restore the union of old and new text so a shorter replacement
     * cannot leave the old tail behind (osd_demo relied on fixed widths) */
    const int new_w = l.text_width(text, st);
    const int clear_w = std::max(s.last_w, new_w);
    if (clear_w > 0)
        l.restore({s.x - 4, s.y - 4, clear_w + 8,
                   (small ? glyph_h_small() : glyph_h()) + 8});
    l.draw_text(s.x, s.y, text, st);
    s.last_w = new_w;
    snprintf(s.last, sizeof s.last, "%s", text);
}

/* ------------------------------------------------------------------ */
/* init / static layer                                                 */
/* ------------------------------------------------------------------ */
bool Layout::init(int canvas_w, int canvas_h, const Config &cfg)
{
    if (canvas_w <= 0 || canvas_h <= 0)
        return false;
    m_cfg = cfg;
    m_w = canvas_w;
    m_h = canvas_h;

    const int W = m_w, H = m_h;
    const int gw = glyph_w();               /* 16 */
    const int row = glyph_h() + 6;          /* 38: demo's right-column pitch */

    /* right column (right-aligned, widths kept stable via fixed formats) */
    m_slot_mode    = { W - 22 - 9  * gw, 10 };            /* "   OFFBOARD" */
    m_slot_tracker = { W - 22 - 12 * gw, 10 + row };      /* "TRK TRACKING"*/
    m_slot_timer   = { W - 22 - 8  * gw, 10 + 2 * row };  /* "T+12:34"     */
    m_slot_batt    = { W - 22 - 9  * gw, 10 + 4 * row };  /* "BAT 22.2V"   */
    m_slot_cell    = { W - 22 - 9  * gw, 10 + 5 * row };  /* "CELL 3.70"   */
    m_slot_cur     = { W - 22 - 9  * gw, 10 + 6 * row };  /* "CUR 04.3A"   */
    m_slot_mah     = { W - 22 - 9  * gw, 10 + 7 * row };  /* "MAH 00410"   */

    /* left telemetry column */
    m_slot_alt = { 22, 118 };
    m_slot_spd = { 22, 118 + row };
    m_slot_hdg = { 22, 118 + 2 * row };
    m_slot_sat = { 22, 118 + 3 * row };
    m_slot_tmp = { 22, 118 + 4 * row };

    /* home arrow cluster (top center) */
    m_slot_home    = { W / 2 - 3 * gw, 86 };
    m_arrow_region = { W / 2 - 30, 32, 60, 52 };

    /* bottom rows: clock/frame left, message/armed center, warning right */
    m_slot_time  = { 22, H - 2 * glyph_h() - 26 };
    m_slot_frame = { 22, H - glyph_h() - 18 };
    m_slot_msg   = { W / 2 - 10 * gw, H - 2 * glyph_h() - 26 };
    m_slot_arm   = { W / 2 - 4 * gw, H - glyph_h() - 18 };
    m_slot_warn  = { W - 22 - 8 * gw, H - glyph_h() - 18 };

    /* pitch-ladder restore footprint around the center reticle */
    m_horizon_region = osd::Layer::horizon_region(W / 2, H / 2);

    /* battery usage bar: clear of the TMP slot (ends y ~306) and the
     * clock row (starts y ~390); row pitch must exceed the 36px cell */
    m_bar_y = 330;
    m_last_usage = -1.f;
    m_prev_foot.clear();
    m_last_trig_seq = 0;
    m_trig_until_s = -1.0;
    return true;
}

void Layout::draw_static(osd::Layer &hud)
{
    const int W = m_w, H = m_h;

    if (m_cfg.corner_brackets) {
        const int m = 16, len = 30, t = 2;
        auto brk = [&](int x, int y, int sx, int sy) {
            hud.fill_rect({x, y, len * sx, t}, kWhite);
            hud.fill_rect({x, y, t, len * sy}, kWhite);
        };
        brk(m, m, 1, 1);
        brk(W - m - 1, m, -1, 1);
        brk(m, H - m - 1, 1, -1);
        brk(W - m - 1, H - m - 1, -1, -1);
    }
    if (m_cfg.crosshair)
        hud.draw_crosshair(W / 2, H / 2, kRed);

    TextStyle st;
    st.color = kWhite;
    hud.draw_text(22, 10, m_cfg.craft_name, st);
    st.color = kCyan;
    st.small = true;
    hud.draw_text(22, 10 + glyph_h() + 6, "RK3576 OSD NTSC", st);
}

/* ------------------------------------------------------------------ */
/* tracker banner                                                      */
/* ------------------------------------------------------------------ */
const char *Layout::tracker_text(int8_t state, Color &c) const
{
    switch (state) {
    case 0:  c = kWhite; return "TRK SCANNING";
    case 1:  c = kCyan;  return "TRK FOUND   ";
    case 2:  c = kGreen; return "TRK TRACKING";
    case 3:  c = kRed;   return "TARGET LOST ";
    default: c = kWhite; return "             ";
    }
}

/* ------------------------------------------------------------------ */
/* per-frame update                                                    */
/* ------------------------------------------------------------------ */
void Layout::update(const Telemetry &t, const std::vector<osd::BBoxRect> &boxes,
                    osd::Layer &hud, osd::Layer &det, const FrameCtx &f)
{
    /* trigger event: latch a 2 s banner (checked every frame, cheap) */
    if (t.trigger_seq != m_last_trig_seq) {
        m_last_trig_seq = t.trigger_seq;
        m_trig_until_s = f.t_s + 2.0;
    }

    /* ---- ~10 Hz text slots (change-driven) ---- */
    if ((f.frame % 6) == 0) {
        char buf[96];

        if (m_cfg.flymode_el) {
            const bool offboard = strcmp(t.flight_mode, "OFFBOARD") == 0;
            snprintf(buf, sizeof buf, "%9s", t.flight_mode);
            slot_draw(hud, m_slot_mode, buf, offboard ? kCyan : kAmber);
        }

        if (m_cfg.tracker_el) {
            Color tc = kWhite;
            const char *tt = tracker_text(t.tracker_state, tc);
            snprintf(buf, sizeof buf, "%12s", tt);
            slot_draw(hud, m_slot_tracker, buf, tc);
        }

        if (m_cfg.timer_el) {
            if (t.arm_s >= 0.f) {
                int sec = (int)t.arm_s;
                snprintf(buf, sizeof buf, "T+%02d:%02d", sec / 60, sec % 60);
            } else {
                snprintf(buf, sizeof buf, "T+--:--");
            }
            slot_draw(hud, m_slot_timer, buf, kCyan);
        }

        /* battery block: color follows the FC's own cell-health grading */
        if (m_cfg.voltage_el) {
            Color bc = t.bat_state == 3 ? kRed : t.bat_state == 2 ? kAmber : kWhite;
            snprintf(buf, sizeof buf, "BAT %04.1fV", (double)t.batt_v);
            slot_draw(hud, m_slot_batt, buf, bc);
            if (t.cells > 0) {
                snprintf(buf, sizeof buf, "CELL %4.2f", (double)t.batt_v / t.cells);
                slot_draw(hud, m_slot_cell, buf, bc);
            } else {
                slot_draw(hud, m_slot_cell, "", kWhite);
            }
        }
        if (m_cfg.current_el) {
            snprintf(buf, sizeof buf, "CUR %04.1fA", (double)t.current_a);
            slot_draw(hud, m_slot_cur, buf, kWhite);
        }
        if (m_cfg.mah_el) {
            int mah = (int)(t.mah_used + 0.5f);
            snprintf(buf, sizeof buf, "MAH %05d", mah);
            slot_draw(hud, m_slot_mah, buf, kWhite);
        }

        /* left telemetry column */
        if (m_cfg.alt_el) {
            snprintf(buf, sizeof buf, "ALT %04dM", (int)lroundf(t.baro_alt_m));
            slot_draw(hud, m_slot_alt, buf, kWhite);
        }
        if (m_cfg.speed_el) {
            snprintf(buf, sizeof buf, "SPD %03dKMH", (int)(t.gspeed_mps * 3.6f + 0.5f));
            slot_draw(hud, m_slot_spd, buf, kWhite);
        }
        if (m_cfg.heading_el) {
            int hdg = ((int)(t.heading_deg + 0.5f)) % 360;
            if (hdg < 0) hdg += 360;
            snprintf(buf, sizeof buf, "HDG %03d", hdg);
            slot_draw(hud, m_slot_hdg, buf, kWhite);
        }
        if (m_cfg.sats_el) {
            snprintf(buf, sizeof buf, "SAT %2d %s", (int)t.sats,
                     t.gps_fix ? "3D" : (t.sats ? "2D" : "--"));
            slot_draw(hud, m_slot_sat, buf, t.gps_fix ? kWhite : kAmber);
        }
        if (m_cfg.temp_el) {
            snprintf(buf, sizeof buf, "TMP %+03dC", (int)lroundf(t.temperature_c));
            slot_draw(hud, m_slot_tmp, buf, kWhite, /*small=*/true);
        }

        /* home distance under the arrow */
        if (m_cfg.home_el) {
            if (t.home_valid && t.gps_fix)
                snprintf(buf, sizeof buf, "%3dM",
                         std::min(999, (int)(t.home_dist_m + 0.5f)));
            else
                snprintf(buf, sizeof buf, "%3s", "--");
            slot_draw(hud, m_slot_home, buf, kAmber);
        }

        /* clock: the frame's own epoch (falls back to wall clock) */
        if (m_cfg.clock_el) {
            uint64_t ep = t.epoch_ns;
            if (!ep) {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ep = (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
            }
            time_t ep_s = (time_t)(ep / 1000000000ull);
            struct tm tm;
            localtime_r(&ep_s, &tm);
            snprintf(buf, sizeof buf, "%02d:%02d:%02d.%d",
                     tm.tm_hour, tm.tm_min, tm.tm_sec,
                     (int)((ep % 1000000000ull) / 100000000ull));
            slot_draw(hud, m_slot_time, buf, kAmber);
        }
        if (m_cfg.frame_el) {
            snprintf(buf, sizeof buf, "F%06u", f.frame);
            slot_draw(hud, m_slot_frame, buf, kWhite, /*small=*/true);
        }

        /* system / custom message line: error blinks, warn amber, info blue */
        if (m_cfg.message_el) {
            Color mc = kBlue;
            if (t.message_level == 3)      mc = kRed;
            else if (t.message_level == 2) mc = kAmber;
            else if (t.message_level == 1) mc = kBlue;
            const bool show = t.message[0] &&
                              (t.message_level != 3 || hud.blink_phase());
            snprintf(buf, sizeof buf, "%20.20s", show ? t.message : "");
            slot_draw(hud, m_slot_msg, buf, mc);
        }

        /* ARMED banner slot doubles as the trigger-event flash */
        if (m_cfg.armed_el) {
            if (f.t_s < m_trig_until_s && t.trigger_kind[0]) {
                if (strcmp(t.trigger_kind, "ON") == 0)
                    snprintf(buf, sizeof buf, "TRIG ON ");
                else if (strcmp(t.trigger_kind, "OFF") == 0)
                    snprintf(buf, sizeof buf, "TRIG OFF");
                else
                    snprintf(buf, sizeof buf, "MISSION ");
                slot_draw(hud, m_slot_arm, buf, kCyan);
            } else if (!t.armed) {
                slot_draw(hud, m_slot_arm, "DISARMED", kWhite);
            } else if (t.arm_s >= 0.f && t.arm_s < 3.f) {
                /* Betaflight-style: ARMED flashes briefly after arming */
                snprintf(buf, sizeof buf, "%6s", hud.blink_phase() ? "ARMED" : "");
                slot_draw(hud, m_slot_arm, buf, kGreen);
            } else {
                slot_draw(hud, m_slot_arm, "", kWhite);
            }
        }

        /* blinking warning (Betaflight OSD_WARNINGS style) */
        if (m_cfg.warnings_el) {
            Color wc = kRed;
            const char *w = "";
            if (t.failsafe)
                w = "FAILSAFE";
            else if (t.bat_state == 3)
                w = "CRITICAL";
            else if (t.bat_state == 2)
                w = "LOW BATT";
            else if (!t.connected) {
                w = "NO LINK";
                wc = kAmber;
            }
            snprintf(buf, sizeof buf, "%8s", w);
            slot_draw(hud, m_slot_warn, (w[0] && hud.blink_phase()) ? buf : "", wc);
        }

        /* battery usage bar (change-driven, needs a known capacity).
         * The % text keeps its baked outline - no opaque plate: plates
         * read as opaque black boxes over the video. */
        if (m_cfg.mah_el && t.capacity_mah > 0.f && t.mah_used >= 0.f) {
            float frac = 1.f - t.mah_used / t.capacity_mah;
            if (frac < 0.f) frac = 0.f;
            if (frac > 1.f) frac = 1.f;
            if (fabsf(frac - m_last_usage) > 0.005f) {
                m_last_usage = frac;
                hud.restore({14, m_bar_y - 18, 258, 52});
                const Color bc = frac < 0.25f ? kRed : kAmber;
                hud.draw_bar(20, m_bar_y, 150, 12, frac, bc, kBlack);
                snprintf(buf, sizeof buf, "%3d%%", (int)(frac * 100));
                TextStyle s2;
                s2.color = kWhite;
                hud.draw_text(178, m_bar_y - 12, buf, s2);
            }
        }
    }

    /* ---- ~20 Hz home arrow + attitude ladder (restore + redraw) ---- */
    if ((f.frame % 3) == 0) {
        if (m_cfg.home_el) {
            hud.restore(m_arrow_region);
            if (t.home_valid && t.gps_fix) {
                /* Betaflight HOME_DIR: arrow points where home sits relative
                 * to the nose (0 = up = straight ahead) */
                float rel = t.home_bearing_deg - t.heading_deg;
                rel = fmodf(rel, 360.f);
                if (rel < 0.f) rel += 360.f;
                hud.draw_home_arrow(m_w / 2, 58, rel, kAmber, 22);
            }
        }
        if (m_cfg.horizon_el) {
            hud.restore(m_horizon_region);
            hud.draw_horizon(m_w / 2, m_h / 2, t.pitch_deg, t.roll_deg, kWhite);
        }
    }

    /* ---- detection boxes: every frame on the overlay layer ---- */
    update_boxes(boxes, det);
}

void Layout::update_boxes(const std::vector<osd::BBoxRect> &boxes,
                          osd::Layer &det)
{
    /* clear every old footprint BEFORE drawing any box: interleaving the
     * two would let target N's erase bite into target N-1's fresh paint */
    for (const Rect &r : m_prev_foot)
        det.clear(r);
    m_prev_foot.clear();
    m_prev_foot.reserve(boxes.size());

    for (const BBoxRect &in : boxes) {
        BBoxRect b = in;
        if (b.locked) {
            /* per-pixel alpha pulse: no per-frame plane ioctls */
            b.alpha = det.blink_phase() ? 255 : 190;
        } else if (b.alpha >= 255) {
            b.alpha = 160;      /* default comfortable translucency */
        }
        const Rect p = det.draw_bbox(b);
        m_prev_foot.push_back({p.x - 2, p.y - 2, p.w + 4, p.h + 4});
    }
}

} // namespace osd::layout
} // namespace osd
