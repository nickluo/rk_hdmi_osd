/*
 * osd_bridge::Pipeline - the render engine behind the OSD HDMI output.
 *
 * ROS-free C++ (links only osd::core / osd::layout / osd::hw): the render
 * thread runs the proven osd_demo loop (camera-paced 60 fps, RGA 3-pass
 * composition, double-buffered page flips, detection layer on the VOP
 * overlay plane) but draws from injected data instead of simulations:
 *
 *   set_telemetry()  - POD snapshot, copied under a mutex (ROS executor
 *                      threads call this; 10 Hz is plenty)
 *   set_boxes()      - BBoxRect vector (camera space unless configured
 *                      otherwise), the seam a future detector node plugs
 *                      into; mapped to canvas pixels via CanvasMap
 *
 * Without a camera (bench mode) the loop feeds a constant gray frame
 * through the same RGA chain and paces on vblank.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hw/camera.h"
#include "hw/display.h"
#include "hw/compositor.h"
#include "hw/dmabuf.h"
#include "osd/osd.h"
#include "layout/layout.h"

namespace osd_bridge {

class Pipeline {
public:
    struct Config {
        hw::CameraBase::Config cam{};
        hw::HdmiDisplay::Config disp{};
        osd::layout::Config layout{};
        osd::layout::CanvasMap map{};    /* camera -> canvas mapping */
        bool use_camera = true;          /* false: bench mode, gray frame */
        bool boxes_canvas_space = false; /* set_boxes rects already canvas px */
        bool sim_boxes = false;          /* two-target sim (verification) */
        bool pin_big_cores = true;       /* governor + cpu4-7 affinity */
        int  snap_frame = -1;            /* dump at frame N, -1 = off */
        std::string snap_dir = "/tmp";
    };

    /* level is "INFO" or "ERROR"; msg is one line, no trailing newline */
    using LogFn = std::function<void(const char *level, const std::string &msg)>;

    Pipeline() = default;
    ~Pipeline();
    Pipeline(const Pipeline &) = delete;
    Pipeline &operator=(const Pipeline &) = delete;

    bool start(const Config &cfg, LogFn log);
    void stop();
    bool running() const { return m_run_flag.load(); }

    /* ---- data seams (call from any thread) ---- */
    void set_telemetry(const osd::layout::Telemetry &t);
    void set_boxes(const std::vector<osd::BBoxRect> &boxes);
    void set_sim_boxes(bool on) { m_sim_on.store(on); }
    void request_snapshot() { m_snap_req.store(true); }

    /* ---- pure helpers (unit-testable) ---- */
    /* apply the tracker-state color policy to a box; per-box color wins
     * when the injector already chose a non-default one */
    static osd::BBoxRect style_box(int8_t tracker_state, osd::BBoxRect b);
    /* FLU quaternion (ENU yaw, as /mavros/imu/data) -> compass degrees */
    static float HeadingFromQuaternionDeg(double x, double y, double z, double w);
    /* standard geo helpers */
    static float BearingDeg(double lat1, double lon1, double lat2, double lon2);
    static double HaversineM(double lat1, double lon1, double lat2, double lon2);
    /* barometric altitude above a ground pressure reference */
    static float BaroAltM(float pressure_pa, double ground_pa);

private:
    void run();

    Config m_cfg;
    LogFn m_log;

    std::thread m_thread;
    std::atomic_bool m_run_flag{false};
    std::atomic_bool m_snap_req{false};
    std::atomic_bool m_sim_on{false};

    std::mutex m_tel_mtx;
    osd::layout::Telemetry m_tel;
    std::mutex m_box_mtx;
    std::vector<osd::BBoxRect> m_boxes;     /* as injected (camera space) */

    /* bench-mode gray video frame */
    hw::DmaBuf m_gray;
};

} // namespace osd_bridge
