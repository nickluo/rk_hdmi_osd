/*
 * Pipeline implementation - the osd_demo render loop (ar0234_osd_hdmi/
 * apps/osd_demo.cpp) with simulated data replaced by the injected seams.
 */
#include "osd_bridge/pipeline.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <fcntl.h>
#include <unistd.h>
#include <sched.h>

namespace osd_bridge {
namespace {

double now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

hw::ImageDesc layer_image(const osd::Layer &l)
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

} // namespace

Pipeline::~Pipeline()
{
    stop();
}

/* ------------------------------------------------------------------ */
/* seams                                                               */
/* ------------------------------------------------------------------ */
void Pipeline::set_telemetry(const osd::layout::Telemetry &t)
{
    std::lock_guard<std::mutex> lk(m_tel_mtx);
    m_tel = t;
}

void Pipeline::set_boxes(const std::vector<osd::BBoxRect> &boxes)
{
    std::lock_guard<std::mutex> lk(m_box_mtx);
    m_boxes = boxes;
}

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */
osd::BBoxRect Pipeline::style_box(int8_t tracker_state, osd::BBoxRect b)
{
    switch (tracker_state) {
    case 0:  b.color = osd::kWhite; break;   /* Detecting  */
    case 1:  b.color = osd::kCyan;  break;   /* Found      */
    case 2:  b.color = osd::kGreen; b.locked = true; break; /* Tracking */
    case 3:  b.color = osd::kRed;   break;   /* Lost       */
    default: break;                          /* keep injected style */
    }
    return b;
}

float Pipeline::HeadingFromQuaternionDeg(double x, double y, double z, double w)
{
    /* FLU quaternion with ENU-convention yaw (as published by the bridge):
     * yaw_enu 0 = East, CCW positive. Compass heading: 0 = North, CW. */
    const double yaw_enu = std::atan2(2.0 * (w * z + x * y),
                                      1.0 - 2.0 * (y * y + z * z));
    double hdg = 90.0 - yaw_enu * 180.0 / M_PI;
    if (hdg < 0.0)   hdg += 360.0;
    if (hdg >= 360.0) hdg -= 360.0;
    return (float)hdg;
}

float Pipeline::BearingDeg(double lat1, double lon1, double lat2, double lon2)
{
    const double d2r = M_PI / 180.0;
    const double la1 = lat1 * d2r, la2 = lat2 * d2r;
    const double dlo = (lon2 - lon1) * d2r;
    const double y = std::sin(dlo) * std::cos(la2);
    const double x = std::cos(la1) * std::sin(la2) -
                     std::sin(la1) * std::cos(la2) * std::cos(dlo);
    double b = std::atan2(y, x) / d2r;
    if (b < 0.0)    b += 360.0;
    return (float)b;
}

double Pipeline::HaversineM(double lat1, double lon1, double lat2, double lon2)
{
    constexpr double R = 6371000.0;
    const double d2r = M_PI / 180.0;
    const double dla = (lat2 - lat1) * d2r;
    const double dlo = (lon2 - lon1) * d2r;
    const double a = std::sin(dla / 2) * std::sin(dla / 2) +
                     std::cos(lat1 * d2r) * std::cos(lat2 * d2r) *
                     std::sin(dlo / 2) * std::sin(dlo / 2);
    return 2.0 * R * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

float Pipeline::BaroAltM(float pressure_pa, double ground_pa)
{
    if (ground_pa <= 0.0 || pressure_pa <= 0.0)
        return 0.f;
    return (float)(44330.0 * (1.0 - std::pow((double)pressure_pa / ground_pa,
                                             0.190295)));
}

/* ------------------------------------------------------------------ */
/* lifecycle                                                           */
/* ------------------------------------------------------------------ */
bool Pipeline::start(const Config &cfg, LogFn log)
{
    if (m_run_flag.load())
        return true;
    m_cfg = cfg;
    m_log = std::move(log);
    m_sim_on.store(m_cfg.sim_boxes);
    m_run_flag.store(true);
    m_thread = std::thread(&Pipeline::run, this);
    return true;
}

void Pipeline::stop()
{
    if (!m_thread.joinable())
        return;
    m_run_flag.store(false);
    m_thread.join();
}

/* ------------------------------------------------------------------ */
/* the render loop                                                     */
/* ------------------------------------------------------------------ */
void Pipeline::run()
{
    auto logi = [&](const char *fmt, ...) {
        if (!m_log) return;
        char buf[512];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof buf, fmt, ap);
        va_end(ap);
        m_log("INFO", buf);
    };
    auto loge = [&](const char *fmt, ...) {
        if (!m_log) return;
        char buf[512];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof buf, fmt, ap);
        va_end(ap);
        m_log("ERROR", buf);
    };

    /* keep the big cores at full speed (RT kernel + interactive governor
     * otherwise park at 408MHz under this bursty load - osd_demo gotcha) */
    if (m_cfg.pin_big_cores) {
        for (const char *gov : {"/sys/devices/system/cpu/cpufreq/policy0/scaling_governor",
                                "/sys/devices/system/cpu/cpufreq/policy4/scaling_governor"}) {
            int fd = open(gov, O_WRONLY);
            if (fd >= 0) {
                if (write(fd, "performance", 11) < 0) {}
                close(fd);
            }
        }
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        for (int i = 4; i < 8; i++) CPU_SET(i, &cpus);
        sched_setaffinity(0, sizeof(cpus), &cpus);
    }

    hw::MipiCamera cam;
    if (m_cfg.use_camera && !cam.open(m_cfg.cam)) {
        loge("camera open failed");
        m_run_flag.store(false);
        return;
    }
    hw::HdmiDisplay disp;
    if (!disp.open(m_cfg.disp)) {
        loge("display open failed (is lightdm stopped?)");
        m_run_flag.store(false);
        return;
    }

    osd::Layer hud;
    if (!hud.init(disp.width(), disp.height())) {
        loge("hud layer init failed (dma_heap)");
        m_run_flag.store(false);
        return;
    }

    hw::Compositor comp;
    hw::Compositor::Config ccfg;
    ccfg.width = disp.width();
    ccfg.height = disp.height();
    ccfg.video = {m_cfg.map.video.x, m_cfg.map.video.y,
                  m_cfg.map.video.w, m_cfg.map.video.h};
    if (!comp.open(ccfg) || !comp.set_overlay(layer_image(hud))) {
        loge("compositor open failed");
        m_run_flag.store(false);
        return;
    }

    /* bench mode: constant gray frame through the same RGA chain */
    hw::ImageDesc gray;
    if (!m_cfg.use_camera) {
        if (!hw::dmabuf_alloc(m_gray, (size_t)ccfg.video.w * ccfg.video.h)) {
            loge("gray-frame dmabuf alloc failed");
            m_run_flag.store(false);
            return;
        }
        hw::dmabuf_sync_begin(m_gray);
        memset(m_gray.va, 32, m_gray.size);
        hw::dmabuf_sync_end(m_gray);
        gray.dma_fd = m_gray.fd;
        gray.va = m_gray.va;
        gray.size = m_gray.size;
        gray.width = ccfg.video.w;
        gray.height = ccfg.video.h;
        gray.stride = ccfg.video.w;
        gray.format = hw::PixelFormat::Gray8;
    }

    osd::layout::Layout layout;
    if (!layout.init(disp.width(), disp.height(), m_cfg.layout)) {
        loge("layout init failed");
        m_run_flag.store(false);
        return;
    }
    layout.draw_static(hud);
    hud.set_base();

    /* detection layer lives on the overlay-plane dumb buffer; the VOP
     * blends it at scanout - CPU just draws opaque shapes */
    osd::Layer det;
    {
        const hw::ImageDesc ovl = disp.overlay_image();
        if (!det.attach(ovl.dma_fd, ovl.va, ovl.width, ovl.height, ovl.size)) {
            loge("detection layer attach failed");
            m_run_flag.store(false);
            return;
        }
    }

    logi("pipeline up: %dx%d, camera=%s, sim_boxes=%s",
         disp.width(), disp.height(),
         m_cfg.use_camera ? "on" : "bench(gray)",
         m_sim_on.load() ? "on" : "off");

    const double t0 = now_ms();
    double t_stat = t0, acc_det = 0, acc_rga = 0, acc_flip = 0;
    uint32_t frames = 0;
    std::vector<osd::BBoxRect> boxes_draw;   /* canvas space */

    while (m_run_flag.load()) {
        if (disp.flip_pending() && !disp.wait_flip(100))
            break;

        double tphase = now_ms();
        hw::ImageDesc video = gray;
        uint64_t frame_epoch = 0;
        if (m_cfg.use_camera) {
            hw::CameraBase::Frame frame = cam.capture();
            if (!frame.valid())
                break;
            video = frame.image();
            frame_epoch = frame.epoch_ns;
            frame.release();
        }

        /* ---- pull the injected state ---- */
        osd::layout::Telemetry tel;
        {
            std::lock_guard<std::mutex> lk(m_tel_mtx);
            tel = m_tel;
        }
        if (!tel.epoch_ns && frame_epoch)
            tel.epoch_ns = frame_epoch;      /* per-frame SOF time */

        boxes_draw.clear();
        {
            std::lock_guard<std::mutex> lk(m_box_mtx);
            for (const osd::BBoxRect &b : m_boxes) {
                osd::BBoxRect m = b;
                if (!m_cfg.boxes_canvas_space)
                    m.rect = m_cfg.map(b.rect);
                boxes_draw.push_back(m);
            }
        }

        /* two-target sim in canvas space (board verification without a
         * detector) - styled by the live tracker state */
        if (m_sim_on.load()) {
            const double t = frames / 60.0;
            const bool lock = tel.tracker_state == 2;
            osd::BBoxRect a;
            const int bw = 116 + (int)(18.0 * sin(0.31 * t));
            const int bh = 84 + (int)(14.0 * sin(0.24 * t + 2.0));
            const int bx = 360 + (int)(250.0 * sin(0.42 * t));
            const int by = 250 + (int)(110.0 * sin(0.9 * t + 1.0));
            a.rect = {bx - bw / 2, by - bh / 2, bw, bh};
            a.label = "UAV";
            a.conf = 0.9f;
            a.tag = lock ? "T01 LCK" : "T01";
            a.color = osd::kRed;
            boxes_draw.push_back(style_box(tel.tracker_state, a));

            osd::BBoxRect s;
            s.rect = {404 - 35, 154 - 26, 70, 52};
            s.label = "BIRD";
            s.conf = 0.6f;
            s.tag = "T02";
            s.color = osd::kGreen;
            boxes_draw.push_back(style_box(-1, s));
        }

        /* ---- draw both layers ---- */
        hud.begin_frame();
        det.begin_frame();
        layout.update(tel, boxes_draw, hud, det,
                      {hud.frame_count(), (now_ms() - t0) / 1000.0});
        det.end_frame();
        hud.end_frame();
        acc_det += now_ms() - tphase;

        /* ---- hardware composition ---- */
        tphase = now_ms();
        if (!comp.render(video, disp.back_image()))
            break;
        acc_rga += now_ms() - tphase;

        const bool snap = (m_cfg.snap_frame >= 0 &&
                           (int)frames == m_cfg.snap_frame) ||
                          m_snap_req.exchange(false);
        if (snap) {
            char path[256];
            snprintf(path, sizeof path, "%s/bridge_back_%06u.xrgb",
                     m_cfg.snap_dir.c_str(), frames);
            FILE *f = fopen(path, "wb");
            if (f) {
                fwrite(disp.back_pixels(), 1,
                       (size_t)disp.pitch() * disp.height(), f);
                fclose(f);
                logi("snapshot: %s", path);
            }
            snprintf(path, sizeof path, "%s/bridge_hud_%06u.rgba",
                     m_cfg.snap_dir.c_str(), frames);
            f = fopen(path, "wb");
            if (f) {
                fwrite(hud.pixels(), 1, hud.size(), f);
                fclose(f);
                logi("snapshot: %s", path);
            }
            snprintf(path, sizeof path, "%s/bridge_ovl_%06u.rgba",
                     m_cfg.snap_dir.c_str(), frames);
            f = fopen(path, "wb");
            if (f) {
                const hw::ImageDesc ovl = disp.overlay_image();
                fwrite(ovl.va, 1, ovl.size, f);
                fclose(f);
                logi("snapshot: %s", path);
            }
        }

        tphase = now_ms();
        if (!disp.present())
            break;
        acc_flip += now_ms() - tphase;

        frames++;
        const double now = now_ms();
        if (now - t_stat >= 5000) {
            logi("frames=%u fps=%.2f glyphs=%zu det=%.2fms rga=%.2fms flip=%.2fms",
                 frames, frames * 1000.0 / (now - t0),
                 hud.glyph_cache_count(),
                 acc_det / frames, acc_rga / frames, acc_flip / frames);
            t_stat = now;
        }
    }

    disp.wait_flip(200);
    logi("pipeline stopped: %u frames in %.2fs (%.2f fps)",
         frames, (now_ms() - t0) / 1000.0,
         frames / ((now_ms() - t0) / 1000.0));
    if (m_gray.valid())
        hw::dmabuf_free(m_gray);
    m_run_flag.store(false);
}

} // namespace osd_bridge
