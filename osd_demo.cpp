/*
 * osd_demo - AR0234 -> HDMI NTSC pipeline driven by libosd.
 *
 * Demonstrates the library with Betaflight-style FPV elements plus AI
 * interception OSD: detection boxes (label + confidence + track id, threat
 * color coding, lock indicator), mode banner, warnings, home arrow,
 * telemetry, link/battery bars, timestamp, mission timer.
 *
 * Detection/telemetry data here is SIMULATED (two targets on smooth paths).
 * A real detector calls the same osd::Layer API; map camera-space coords to
 * the 720x480 OSD canvas with:
 *     x_osd = x_cam * 720 / 1920
 *     y_osd = y_cam * 450 / 1200 + 14        (letterbox rect y=14 h=450)
 *
 * Capture, composition and scanout live behind hw::CameraBase,
 * hw::Compositor and hw::HdmiDisplay - no 2D-engine or pixel-format
 * details leak in here.
 *
 * Build: make LIBRGA=/home/firefly/workspace/librga    (on board)
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <ctime>
#include <cmath>
#include <initializer_list>
#include <getopt.h>

#include <fcntl.h>
#include <unistd.h>
#include <sched.h>

#include "osd/osd.h"
#include "hw/camera.h"
#include "hw/display.h"
#include "hw/compositor.h"

#define LOGI(...) do { fprintf(stdout, "[I] " __VA_ARGS__); fflush(stdout); } while (0)
#define LOGE(...) do { fprintf(stderr, "[E] " __VA_ARGS__); fflush(stderr); } while (0)

static volatile sig_atomic_t g_run = 1;
static void on_sigint(int) { g_run = 0; }

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

/* the OSD canvas as a plain dmabuf image the hw layer can consume */
static hw::ImageDesc layer_image(const osd::Layer &l)
{
    hw::ImageDesc d;
    d.dma_fd = l.dma_fd();
    d.va = l.pixels();
    d.size = l.size();
    d.width = l.width();
    d.height = l.height();
    d.stride = l.width() * 4;
    d.format = hw::PixelFormat::RGBA8888;
    return d;
}

struct opts {
    hw::CameraBase::Config cam;
    hw::HdmiDisplay::Config disp;
    const char *out_dir = "/tmp";
    int frames = 0, snapframe = 120;
};

/* ------------------------------------------------------------------ */
/* OSD helpers for the simulated AI feed                              */
/* ------------------------------------------------------------------ */
struct SimTarget {
    osd::Rect rect;
    osd::Rect prev_rect;          /* including label plate + echo margin */
    float conf;
    const char *label;
    const char *tag;
    osd::Color color;
    bool locked;
};

/* change-driven text slot: redraws (restoring the static base underneath)
 * only when the text changes or the slot was damaged by a moving element */
struct TextSlot {
    int x, y;
    char last[96] = {};
    int last_w = 0;
};

static void slot_draw(osd::Layer &osd, TextSlot &s, const char *text, osd::Color c)
{
    if (strcmp(text, s.last) == 0) return;
    osd.restore({s.x - 4, s.y - 4, s.last_w + 8, osd::glyph_h() + 8});
    osd::TextStyle st;
    st.color = c;
    osd.draw_text(s.x, s.y, text, st);
    s.last_w = osd.text_width(text, st);
    snprintf(s.last, sizeof s.last, "%s", text);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    opts o;
    static const struct option longopts[] = {
        {"frames", required_argument, 0, 'n'},
        {"snapframe", required_argument, 0, 's'},
        {"outdir", required_argument, 0, 'o'},
        {0, 0, 0, 0},
    };
    int ch;
    while ((ch = getopt_long(argc, argv, "n:s:o:", longopts, NULL)) != -1) {
        switch (ch) {
        case 'n': o.frames = atoi(optarg); break;
        case 's': o.snapframe = atoi(optarg); break;
        case 'o': o.out_dir = optarg; break;
        default: return 1;
        }
    }

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);
    LOGI("=== osd_demo: AR0234 -> RGA -> libosd -> HDMI 720x480 NTSC ===\n");

    /* keep the big cores at full speed: RT kernel + interactive governor
     * otherwise park at 408MHz under this bursty load */
    for (const char *gov : {"/sys/devices/system/cpu/cpufreq/policy0/scaling_governor",
                            "/sys/devices/system/cpu/cpufreq/policy4/scaling_governor"}) {
        int fd = open(gov, O_WRONLY);
        if (fd >= 0) { if (write(fd, "performance", 11) < 0) {} close(fd); }
    }
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    for (int i = 4; i < 8; i++) CPU_SET(i, &cpus);
    sched_setaffinity(0, sizeof(cpus), &cpus);

    hw::MipiCamera cam;
    if (!cam.open(o.cam)) return 1;
    hw::HdmiDisplay disp;
    if (!disp.open(o.disp)) return 1;

    osd::Layer hud;
    if (!hud.init(disp.width(), disp.height())) { LOGE("osd init\n"); return 1; }
    LOGI("osd: %dx%d RGBA8888 dma-heap\n", hud.width(), hud.height());

    hw::Compositor comp;
    hw::Compositor::Config ccfg;
    ccfg.width = disp.width();
    ccfg.height = disp.height();
    ccfg.video = {0, 14, 720, 450};        /* 16:10 source letterboxed to 4:3 */
    if (!comp.open(ccfg)) { LOGE("compositor open\n"); return 1; }
    if (!comp.set_overlay(layer_image(hud))) { LOGE("compositor overlay\n"); return 1; }

    /* ---------------- static OSD elements ---------------- */
    osd::TextStyle st;
    const int W = disp.width(), H = disp.height();

    /* corner brackets */
    {
        const int m = 16, len = 30, t = 2;
        auto brk = [&](int x, int y, int sx, int sy) {
            hud.fill_rect({x, y, len * sx, t}, osd::kWhite);
            hud.fill_rect({x, y, t, len * sy}, osd::kWhite);
        };
        brk(m, m, 1, 1);
        brk(W - m - 1, m, -1, 1);
        brk(m, H - m - 1, 1, -1);
        brk(W - m - 1, H - m - 1, -1, -1);
    }
    hud.draw_crosshair(W / 2, H / 2, osd::kRed);

    st.color = osd::kWhite;
    hud.draw_text(22, 10, "AR0234 1920x1200@60", st);
    st.color = osd::kCyan;
    hud.draw_text(22, 10 + osd::glyph_h() + 6, "RK3576 RGA OSD NTSC", st);

    /* dynamic text slots */
    TextSlot slot_mode    { W - 22 - 9 * osd::glyph_w(), 10 };            /* right top */
    TextSlot slot_det     { W - 22 - 12 * osd::glyph_w(), 10 + osd::glyph_h() + 6 };
    TextSlot slot_alt     { 22, 118 };
    TextSlot slot_spd     { 22, 118 + 38 };
    TextSlot slot_hdg     { 22, 118 + 76 };
    TextSlot slot_home    { W / 2 - 3 * osd::glyph_w(), 86 };
    TextSlot slot_time    { 22, H - 2 * osd::glyph_h() - 26 };
    TextSlot slot_frame   { 22, H - osd::glyph_h() - 18 };
    TextSlot slot_warn    { W - 22 - 8 * osd::glyph_w(), H - osd::glyph_h() - 18 };
    TextSlot slot_timer   { W - 22 - 6 * osd::glyph_w(), 10 + 2 * (osd::glyph_h() + 6) };

    /* bars: link + battery, bottom left (kept clear of the timestamp row).
     * Row pitch must exceed the 36px glyph cell or the percent plates overlap. */
    const int bar_y = 300, bar_pitch = 40;
    float last_link = -1, last_batt = -1;

    SimTarget targets[2] = {};
    targets[0].label = "UAV";
    targets[0].color = osd::kRed;
    targets[1].label = "BIRD";
    targets[1].color = osd::kGreen;

    /* snapshot the static layer: changing HUD text restores its slot from
     * the base instead of erasing what was drawn there before */
    hud.set_base();
    const osd::Rect arrow_region{W / 2 - 30, 32, 60, 52};

    /* detection layer: attached to the overlay-plane dumb buffer; VOP
     * blends it translucently at scanout - CPU just draws opaque shapes */
    osd::Layer det;
    {
        const hw::ImageDesc ovl = disp.overlay_image();
        if (!det.attach(ovl.dma_fd, ovl.va, ovl.width, ovl.height, ovl.size)) {
            LOGE("det layer attach\n");
            return 1;
        }
    }

    double t0 = now_ms(), t_stat = t0;
    double acc_det = 0, acc_rga = 0, acc_flip = 0, acc_dq = 0;
    int frames = 0;
    LOGI("running...\n");

    while (g_run && (!o.frames || frames < o.frames)) {
        if (disp.flip_pending() && !disp.wait_flip(100)) break;

        double tphase = now_ms();
        hw::CameraBase::Frame frame = cam.capture();
        if (!frame.valid()) break;
        acc_dq += now_ms() - tphase;
        tphase = now_ms();

        const double t = frames / 60.0;

        /* ---- simulate detector output (2 targets, periodic lock) ---- */
        const bool lock = sinf(0.55f * t) > 0.25f;
        {
            SimTarget &a = targets[0];
            int bw = 116 + (int)(18.0f * sinf(0.31f * t));
            int bh = 84 + (int)(14.0f * sinf(0.24f * t + 2.0f));
            int bx = 360 + (int)(250.0f * sinf(0.42f * t));
            int by = 250 + (int)(110.0f * sinf(0.9f * t + 1.0f));
            a.rect = {bx - bw / 2, by - bh / 2, bw, bh};
            a.conf = 0.86f + 0.12f * (0.5f + 0.5f * sinf(0.8f * t + 3));
            a.locked = lock;
            a.tag = lock ? "T01 LCK" : "T01";
            a.color = lock ? osd::kAmber : osd::kRed;
        }
        {
            SimTarget &b = targets[1];
            int bw = 70, bh = 52;
            int bx = 470 + (int)(130.0f * sinf(0.13f * t + 3.0f));
            int by = 180 + (int)(88.0f * sinf(0.2f * t));
            b.rect = {bx - bw / 2, by - bh / 2, bw, bh};
            b.conf = 0.55f + 0.25f * (0.5f + 0.5f * sinf(0.37f * t + 1));
            b.locked = false;
            b.tag = "T02";
            b.color = osd::kGreen;
        }

        hud.begin_frame();

        /* ---- 10Hz dynamic text (change driven) ---- */
        if ((frames % 6) == 0) {
            char buf[96];
            const char *mode = lock ? "INTERCEPT" : "TRACK";
            snprintf(buf, sizeof buf, "%9s", mode);
            slot_draw(hud, slot_mode, buf, lock ? osd::kRed : osd::kAmber);

            snprintf(buf, sizeof buf, "DET 2 %4.1fms", 6.0 + 3.0 * (0.5 + 0.5 * sin(0.9 * t)));
            slot_draw(hud, slot_det, buf, osd::kWhite);

            snprintf(buf, sizeof buf, "ALT %04dM", 80 + (int)(40 * sin(0.2 * t)));
            slot_draw(hud, slot_alt, buf, osd::kWhite);
            snprintf(buf, sizeof buf, "SPD %03dKMH", 32 + (int)(18 * cos(0.15 * t)));
            slot_draw(hud, slot_spd, buf, osd::kWhite);
            snprintf(buf, sizeof buf, "HDG %03d", (int)(t * 7) % 360);
            slot_draw(hud, slot_hdg, buf, osd::kWhite);

            int hd = 40 + (int)(15 * sin(0.3 * t));
            snprintf(buf, sizeof buf, "%3dM", hd);
            slot_draw(hud, slot_home, buf, osd::kAmber);

            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            struct tm tm;
            localtime_r(&ts.tv_sec, &tm);
            snprintf(buf, sizeof buf, "%02d:%02d:%02d.%d",
                     tm.tm_hour, tm.tm_min, tm.tm_sec, (int)(ts.tv_nsec / 100000000UL));
            slot_draw(hud, slot_time, buf, osd::kAmber);

            double fps = frames > 0 ? frames * 1000.0 / (now_ms() - t0) : 0;
            snprintf(buf, sizeof buf, "F%06d %4.1ffps", frames, fps);
            slot_draw(hud, slot_frame, buf, osd::kWhite);

            int sec = (int)t;
            snprintf(buf, sizeof buf, "T+%02d:%02d", sec / 60, sec % 60);
            slot_draw(hud, slot_timer, buf, osd::kCyan);

            /* link / battery bars (also redrawn when damaged by a box) */
            float link = 0.62f + 0.30f * (0.5f + 0.5f * sin(0.7 * t));
            float batt = 1.0f - (float)t / 600.0f;
            if (fabsf(link - last_link) > 0.01f || fabsf(batt - last_batt) > 0.01f) {
                last_link = link; last_batt = batt;
                /* must stop short of the timestamp slot below: the bars are
                 * drawn after it, so an oversized restore erases it */
                hud.restore({16, bar_y - 16, 230, bar_pitch + 52});
                osd::TextStyle s2; s2.color = osd::kWhite;
                const struct { int y; float v; osd::Color c; } rows[2] = {
                    {bar_y, link, link < 0.65f ? osd::kRed : osd::kGreen},
                    {bar_y + bar_pitch, batt, batt < 0.25f ? osd::kRed : osd::kAmber},
                };
                for (const auto &r : rows) {
                    hud.draw_bar(20, r.y, 150, 12, r.v, r.c, osd::kBlack);
                    snprintf(buf, sizeof buf, "%3d%%", (int)(r.v * 100));
                    /* opaque: RGA imblend cannot be trusted with src alpha<255 */
                    hud.fill_rect({176, r.y - 11, 64, 34}, osd::kBlack);
                    hud.draw_text(178, r.y - 12, buf, s2);
                }
            }

            /* blinking warning (Betaflight OSD_WARNINGS style) */
            snprintf(buf, sizeof buf, "%8s", link < 0.65f ? "LOW LINK" : "");
            slot_draw(hud, slot_warn, buf, osd::kRed);

        }

        /* home arrow (rotates slowly in the sim) */
        if ((frames % 3) == 0) {
            hud.restore(arrow_region);
            hud.draw_home_arrow(W / 2, 58, fmodf(t * 11.0f, 360.0f), osd::kAmber, 22);
        }

        /* ---- detection layer: clear old footprints, draw new boxes;
         * the VOP overlay plane blends them translucently over the
         * video+HUD, so anything underneath shows through ---- */
        tphase = now_ms();
        /* per-pixel alpha pulse (no per-frame ioctl: plane alpha is fixed
         * at 235; pixel alpha scales it - normal 160 -> ~147 effective,
         * locked blinks 255/190 -> ~235/~175) */
        const uint8_t box_alpha = lock ? (det.blink_phase() ? 255 : 190) : 160;

        det.begin_frame();
        /* clear every old footprint BEFORE drawing any box: interleaving the
         * two would let target N's erase bite into target N-1's fresh paint */
        for (auto &tg : targets)
            if (tg.prev_rect.w > 0)
                det.clear(tg.prev_rect);
        for (auto &tg : targets) {
            osd::DetectBox db;
            db.rect = tg.rect;
            db.label = tg.label;
            db.tag = tg.tag;
            db.conf = tg.conf;
            db.color = tg.color;
            db.locked = tg.locked;
            db.plate_opaque = true;      /* plane alpha does the translucency */
            db.alpha = (tg.locked || tg.color.a != 0) ? box_alpha : 160;
            /* exact painted bounds (plate can be much wider than the box)
             * +2px margin for line stamps; this is the clear footprint for
             * the next frame - no more colored trails on movement */
            osd::Rect painted = det.draw_detect_box(db);
            tg.prev_rect = {painted.x - 2, painted.y - 2,
                            painted.w + 4, painted.h + 4};
        }
        det.end_frame();
        acc_det += now_ms() - tphase;

        hud.end_frame();       /* single cache clean, skipped when clean */

        /* ---- hardware composition: video letterboxed + HUD blended ---- */
        tphase = now_ms();
        if (!comp.render(frame.image(), disp.back_image())) break;
        frame.release();                 /* the engine is done reading it */
        acc_rga += now_ms() - tphase;

        if (o.snapframe >= 0 && frames == o.snapframe) {
            char path[256];
            snprintf(path, sizeof path, "%s/osddemo_snap_%06d.xrgb", o.out_dir, frames);
            FILE *f = fopen(path, "wb");
            if (f) {
                fwrite(disp.back_pixels(), 1,
                       (size_t)disp.pitch() * disp.height(), f);
                fclose(f);
                LOGI("snapshot: %s\n", path);
            }
            /* also dump the OSD canvas itself (RGBA, memory R,G,B,A) to
             * separate CPU-compositing issues from RGA blend rendering */
            snprintf(path, sizeof path, "%s/osddemo_canvas_%06d.rgba", o.out_dir, frames);
            f = fopen(path, "wb");
            if (f) {
                fwrite(hud.pixels(), 1, hud.size(), f);
                fclose(f);
                LOGI("canvas dump: %s\n", path);
            }
            /* overlay-plane buffer (RGBA, memory R,G,B,A): composite with the
             * primary dump offline (out = ovl*a + fb*(1-a), a = px*235/255/255)
             * to reproduce exactly what the VOP shows on screen */
            snprintf(path, sizeof path, "%s/osddemo_ovl_%06d.rgba", o.out_dir, frames);
            f = fopen(path, "wb");
            if (f) {
                const hw::ImageDesc ovl = disp.overlay_image();
                fwrite(ovl.va, 1, ovl.size, f);
                fclose(f);
                LOGI("ovl dump: %s\n", path);
            }
        }

        tphase = now_ms();
        if (!disp.present()) break;
        acc_flip += now_ms() - tphase;

        frames++;
        double now = now_ms();
        if (now - t_stat >= 5000) {
            double fps = frames * 1000.0 / (now - t0);
            LOGI("frames=%d fps=%.2f glyphs=%zu  dq=%.2fms det=%.2fms rga=%.2fms flip=%.2fms\n",
                 frames, fps, hud.glyph_cache_count(),
                 acc_dq / frames, acc_det / frames, acc_rga / frames, acc_flip / frames);
            t_stat = now;
        }
    }

    disp.wait_flip(200);
    double el = now_ms() - t0;
    LOGI("done: %d frames in %.2fs (%.2f fps), glyphs cached %zu\n",
         frames, el / 1000.0, frames / (el / 1000.0), hud.glyph_cache_count());

    hud.shutdown();
    det.shutdown();
    return 0;
}
