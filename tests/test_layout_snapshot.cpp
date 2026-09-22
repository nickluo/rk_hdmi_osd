/* test_layout_snapshot: render the full layout from a synthetic Telemetry
 * stream on memory-backed layers (no dma-heap / DRM / RGA needed), dump
 * PPM snapshots for eyeballing and assert the mechanics:
 *
 *  1. battery slot paints opaque text
 *  2. warning slot blinks with hud.blink_phase() (CRITICAL phase)
 *  3. a slot repaints when its value changes (ALT digits)
 *  4. home arrow draws when home is valid, disappears when not
 *  5. mode banner renders OFFBOARD in cyan
 *  6. locked BBoxRect keeps its configured color
 *  7. box footprint clear: a moved-away box leaves no residue
 *
 * Usage: test_layout_snapshot [out_dir]
 */
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include "osd/osd.h"
#include "layout/layout.h"

static int g_fail = 0;

static void check(bool ok, const char *what, const char *detail)
{
    printf("%-4s %-38s %s\n", ok ? "PASS" : "FAIL", what, detail);
    if (!ok) g_fail++;
}

struct Px { uint8_t r, g, b, a; };

static Px px(const osd::Layer &l, int x, int y)
{
    const uint32_t v = ((const uint32_t *)l.pixels())[(size_t)y * l.width() + x];
    return {(uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF),
            (uint8_t)((v >> 16) & 0xFF), (uint8_t)(v >> 24)};
}

static int count_opaque(const osd::Layer &l, osd::Rect r)
{
    int n = 0;
    for (int y = r.y; y < r.y + r.h; y++)
        for (int x = r.x; x < r.x + r.w; x++)
            if (px(l, x, y).a != 0) n++;
    return n;
}

static uint64_t rgb_checksum(const osd::Layer &l, osd::Rect r)
{
    uint64_t s = 0;
    for (int y = r.y; y < r.y + r.h; y++)
        for (int x = r.x; x < r.x + r.w; x++) {
            const Px p = px(l, x, y);
            if (p.a) s += p.r + (uint64_t)p.g * 7 + (uint64_t)p.b * 13;
        }
    return s;
}

/* dump a layer composited over a solid background as binary PPM */
static void dump_ppm(const char *path, const osd::Layer &l,
                     uint8_t bg_r, uint8_t bg_g, uint8_t bg_b)
{
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", l.width(), l.height());
    for (int y = 0; y < l.height(); y++)
        for (int x = 0; x < l.width(); x++) {
            const Px p = px(l, x, y);
            const float a = p.a / 255.f;
            const uint8_t r = (uint8_t)(p.r * a + bg_r * (1 - a));
            const uint8_t g = (uint8_t)(p.g * a + bg_g * (1 - a));
            const uint8_t b = (uint8_t)(p.b * a + bg_b * (1 - a));
            const unsigned char rgb[3] = {r, g, b};
            fwrite(rgb, 1, 3, f);
        }
    fclose(f);
    printf("snapshot: %s\n", path);
}

static void fill_telemetry(osd::layout::Telemetry &t, bool critical)
{
    snprintf(t.craft, sizeof t.craft, "ROCKET");
    t.connected = true;
    t.armed = true;
    snprintf(t.flight_mode, sizeof t.flight_mode, "OFFBOARD");
    t.failsafe = false;
    t.batt_v = critical ? 20.1f : 22.2f;
    t.cells = 6;
    t.current_a = 4.3f;
    t.mah_used = critical ? 10400.f : 410.f;
    t.capacity_mah = 13000.f;
    t.bat_state = critical ? 3 : 1;
    t.heading_deg = critical ? 220.f : 214.f;
    t.pitch_deg = 3.f;
    t.roll_deg = -2.f;
    t.baro_alt_m = critical ? 95.f : 102.f;
    t.temperature_c = 31.f;
    t.gps_fix = true;
    t.sats = 12;
    t.gspeed_mps = 8.3f;
    t.course_deg = 210.f;
    t.lat_deg = 30.5;
    t.lon_deg = 114.4;
    t.home_valid = !critical;
    t.home_bearing_deg = 120.f;
    t.home_dist_m = 240.f;
    t.tracker_state = 2;
    t.trigger_seq = critical ? 1u : 0u;
    snprintf(t.trigger_kind, sizeof t.trigger_kind, "ON");
    snprintf(t.message, sizeof t.message, critical ? "LINK LOST 12S" : "");
    t.message_level = critical ? 3 : 0;
    t.epoch_ns = 1760000000ull * 1000000000ull;
    t.arm_s = 75.f;
}

/* two moving boxes, osd_demo style (canvas space already) */
static void sim_boxes(double t, std::vector<osd::BBoxRect> &out)
{
    out.clear();
    const bool lock = sinf(0.55f * (float)t) > 0.25f;
    osd::BBoxRect a;
    {
        const int bw = 116 + (int)(18.f * sinf(0.31f * (float)t));
        const int bh = 84 + (int)(14.f * sinf(0.24f * (float)t + 2.f));
        const int bx = 360 + (int)(250.f * sinf(0.42f * (float)t));
        const int by = 250 + (int)(110.f * sinf(0.9f * (float)t + 1.f));
        a.rect = {bx - bw / 2, by - bh / 2, bw, bh};
        a.label = "UAV";
        a.conf = 0.9f;
        a.color = osd::kRed;
        a.locked = lock;
        a.tag = lock ? "T01 LCK" : "T01";
    }
    out.push_back(a);

    osd::BBoxRect b;
    b.rect = {404, 154, 70, 52};
    b.label = "BIRD";
    b.conf = 0.6f;
    b.color = osd::kGreen;
    b.tag = "T02";
    out.push_back(b);
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : ".";
    const int W = 720, H = 480;

    osd::Layer hud, det;
    if (!hud.init_auto(W, H) || !det.init_auto(W, H)) {
        printf("layer init failed\n");
        return 1;
    }

    osd::layout::Config cfg;
    osd::layout::Layout layout;
    if (!layout.init(W, H, cfg)) { printf("layout init failed\n"); return 1; }
    layout.draw_static(hud);
    hud.set_base();

    std::vector<osd::BBoxRect> boxes;
    uint64_t alt_sum_a = 0, alt_sum_b = 0;
    int arrow_px_a = 0, arrow_px_b = 0;

    const osd::Rect alt_region{22 - 4, 118 - 4, 12 * osd::glyph_w() + 8,
                               osd::glyph_h() + 8};
    const osd::Rect arrow_region{W / 2 - 30, 32, 60, 52};
    const osd::Rect batt_region{W - 22 - 9 * osd::glyph_w() - 4, 162 - 4,
                                9 * osd::glyph_w() + 8, osd::glyph_h() + 8};
    /* text-only part of the warn slot: clear of the bottom-right bracket */
    const osd::Rect warn_text{W - 22 - 8 * osd::glyph_w(), H - osd::glyph_h() - 18,
                              7 * osd::glyph_w(), osd::glyph_h()};

    for (int f = 0; f < 180; f++) {
        const bool critical = f >= 120;
        const double t = f / 60.0;

        osd::layout::Telemetry tel;
        fill_telemetry(tel, critical);
        sim_boxes(t, boxes);

        hud.begin_frame();
        det.begin_frame();
        layout.update(tel, boxes, hud, det,
                      {hud.frame_count(), t});
        hud.end_frame();
        det.end_frame();

        /* the 10/20 Hz gates key off hud.frame_count() == f+1, so text
         * refreshes land on f = 5, 11, ... 113, 119, 125, ... and arrow
         * refreshes on f = 2, 5, ... 125, ...: sample just after those */
        if (f == 116) {                 /* content from the f=113 draw (A) */
            alt_sum_a = rgb_checksum(hud, alt_region);
            arrow_px_a = count_opaque(hud, arrow_region);
        }
        if (f == 127) {                 /* content from the f=125 draw (B) */
            alt_sum_b = rgb_checksum(hud, alt_region);
            arrow_px_b = count_opaque(hud, arrow_region);
        }

        if (f == 100 || f == 150) {
            char path[256];
            snprintf(path, sizeof path, "%s/layout_hud_%03d.ppm", dir, f);
            dump_ppm(path, hud, 100, 100, 100);
            snprintf(path, sizeof path, "%s/layout_det_%03d.ppm", dir, f);
            dump_ppm(path, det, 20, 20, 20);
        }
    }

    char d[160];

    /* 1. battery slot paints */
    const int batt_px = count_opaque(hud, batt_region);
    snprintf(d, sizeof d, "%d opaque px", batt_px);
    check(batt_px > 100, "battery slot paints", d);

    /* 2. warning blink: re-render frames until both blink phases are seen.
     * blink toggles every 18 frames and 18 is a multiple of the 6-frame
     * text cadence, so the slot always updates on the toggle frame. */
    auto render_one = [&](int frame, bool critical, int *warn_px) {
        osd::layout::Telemetry tel;
        fill_telemetry(tel, critical);
        std::vector<osd::BBoxRect> bx;
        sim_boxes(frame / 60.0, bx);
        hud.begin_frame();
        layout.update(tel, bx, hud, det, {hud.frame_count(), frame / 60.0});
        hud.end_frame();
        if (warn_px) *warn_px = count_opaque(hud, warn_text);
    };
    int warn_on = -1, warn_off = -1;
    int msg_on = -1;
    const osd::Rect msg_text{W / 2 - 10 * osd::glyph_w(), H - 2 * osd::glyph_h() - 26,
                             20 * osd::glyph_w(), osd::glyph_h()};
    for (int f = 180; f < 240 && (warn_on < 0 || warn_off < 0); f++) {
        int n;
        render_one(f, true, &n);
        if (n > 50 && warn_on < 0) {
            warn_on = n;
            msg_on = count_opaque(hud, msg_text);
        }
        if (n == 0 && warn_off < 0) warn_off = n;
    }
    snprintf(d, sizeof d, "on=%d off=%d", warn_on, warn_off);
    check(warn_on > 50 && warn_off == 0, "warning blinks (CRITICAL)", d);
    snprintf(d, sizeof d, "blink-on msg px=%d", msg_on);
    check(msg_on > 100, "error message line paints", d);

    /* 3. slot repaint on value change */
    snprintf(d, sizeof d, "sumA=%lu sumB=%lu",
             (unsigned long)alt_sum_a, (unsigned long)alt_sum_b);
    check(alt_sum_a != 0 && alt_sum_b != 0 && alt_sum_a != alt_sum_b,
          "ALT slot repaints on change", d);

    /* 4. home arrow appears/disappears */
    snprintf(d, sizeof d, "valid=%d invalid=%d", arrow_px_a, arrow_px_b);
    check(arrow_px_a > 30 && arrow_px_b == 0, "home arrow presence", d);

    /* 5. OFFBOARD banner in cyan */
    bool cyan_seen = false;
    for (int y = 6; y < 6 + osd::glyph_h() + 4 && !cyan_seen; y++)
        for (int x = W - 22 - 9 * osd::glyph_w(); x < W - 22 && !cyan_seen; x++) {
            const Px p = px(hud, x, y);
            if (p.a == 255 && p.b > 230 && p.g > 180 && p.r < 130) cyan_seen = true;
        }
    check(cyan_seen, "OFFBOARD banner cyan", cyan_seen ? "found" : "none");

    /* 6+7. box color + footprint clear (controlled two-frame sequence) */
    std::vector<osd::BBoxRect> fb;
    osd::BBoxRect box;
    box.rect = {300, 200, 100, 70};
    box.label = "UAV";
    box.conf = 0.9f;
    box.color = osd::kRed;
    box.locked = true;
    fb.push_back(box);
    osd::layout::Telemetry tel;
    fill_telemetry(tel, true);
    hud.begin_frame(); det.begin_frame();
    layout.update(tel, fb, hud, det, {hud.frame_count(), 4.0});
    hud.end_frame(); det.end_frame();
    const Px corner = px(det, box.rect.x + 1, box.rect.y + 1);
    snprintf(d, sizeof d, "got (%u,%u,%u,%u) want kRed rgb", corner.r,
             corner.g, corner.b, corner.a);
    check(corner.r == 255 && corner.g == 70 && corner.b == 70 && corner.a != 0,
          "locked box keeps color", d);

    fb[0].rect = {80, 60, 80, 50};          /* teleport far away */
    hud.begin_frame(); det.begin_frame();
    layout.update(tel, fb, hud, det, {hud.frame_count(), 4.05});
    hud.end_frame(); det.end_frame();
    const Px gone = px(det, 301, 201);
    snprintf(d, sizeof d, "old corner alpha=%u (want 0)", gone.a);
    check(gone.a == 0, "box footprint clears", d);

    /* 8. attitude ladder moves with pitch (replaces P/R numerics).
     * y-weighted hash over a strip right of center (static crosshair
     * excluded): pitch +8 vs +5 gives a different long/short-rung layout
     * at different heights; run 3 frames so the 20 Hz gate fires. */
    const osd::Rect lad_strip{W / 2 + 42, H / 2 - 95, 34, 190};
    auto lad_hash = [&]() {
        uint64_t h = 0;
        for (int y = lad_strip.y; y < lad_strip.y + lad_strip.h; y++)
            for (int x = lad_strip.x; x < lad_strip.x + lad_strip.w; x++)
                if (px(hud, x, y).a) h += (uint64_t)(y + 1);
        return h;
    };
    auto draw_ladder = [&](float pitch) {
        tel.pitch_deg = pitch;
        tel.roll_deg = 0.f;
        for (int i = 0; i < 3; i++) {         /* gate fires once in any 3 */
            hud.begin_frame();
            layout.update(tel, fb, hud, det, {hud.frame_count(), 4.1});
            hud.end_frame();
        }
    };
    draw_ladder(8.f);
    const uint64_t lad_a = lad_hash();
    draw_ladder(5.f);
    const uint64_t lad_b = lad_hash();
    snprintf(d, sizeof d, "p8=%lu p5=%lu", (unsigned long)lad_a,
             (unsigned long)lad_b);
    check(lad_a != 0 && lad_b != 0 && lad_a != lad_b,
          "horizon ladder tracks pitch", d);

    /* 9. small font: TMP slot paints 8x16 glyphs (content high <= 24 px) */
    const osd::Rect tmp_hi{22 - 4, 270 - 4, 80, 24};
    const osd::Rect tmp_lo{22, 270 + 24, 80, 10};
    const int hi = count_opaque(hud, tmp_hi), lo = count_opaque(hud, tmp_lo);
    snprintf(d, sizeof d, "hi=%d below=%d", hi, lo);
    check(hi > 30 && lo == 0, "small font (TMP) height", d);

    printf("\n%s (%d failures)\n", g_fail ? "LAYOUT TEST FAILED" : "LAYOUT TEST OK",
           g_fail);
    return g_fail ? 1 : 0;
}
