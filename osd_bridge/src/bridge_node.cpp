/*
 * OsdBridgeNode implementation - see bridge_node.h.
 */
#include "osd_bridge/bridge_node.h"

#include <algorithm>
#include <cmath>
#include <cstring>

using namespace std::chrono_literals;

namespace osd_bridge {

namespace
{
rclcpp::QoS best_effort(size_t depth = 10)
{
    return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort();
}
} // namespace

OsdBridgeNode::OsdBridgeNode(const rclcpp::NodeOptions &options)
    : Node("osd_bridge", options)
{
    // ---------------- parameters ----------------
    const bool cam_enabled = declare_parameter<bool>("camera.enabled", true);
    const std::string sensor = declare_parameter<std::string>("camera.sensor", "sc233hgs");
    const int cam_index = declare_parameter<int>("camera.index", 0);
    const std::string dri_dev = declare_parameter<std::string>("display.device", "/dev/dri/card0");
    const int disp_w = declare_parameter<int>("display.width", 720);
    const int disp_h = declare_parameter<int>("display.height", 480);
    const int ovl_alpha = declare_parameter<int>("display.overlay_alpha", 235);
    const std::string craft = declare_parameter<std::string>("craft_name", "ROCKET");
    lipo_cells_ = declare_parameter<int>("lipo_cells", 0);       /* 0 = auto from FC */
    const double capacity_mah = declare_parameter<double>("battery.capacity_mah", 0.0);
    baro_ground_ref_ = declare_parameter<bool>("baro.ground_ref", true);
    capture_home_on_arm_ = declare_parameter<bool>("home.capture_on_arm", true);
    const bool sim_boxes = declare_parameter<bool>("boxes.sim", false);
    const bool boxes_canvas = declare_parameter<bool>("boxes.canvas_space", false);
    const bool pin_cores = declare_parameter<bool>("render.pin_big_cores", true);
    const int snap_frame = declare_parameter<int>("snapshot.frame", -1);
    const std::string snap_dir = declare_parameter<std::string>("snapshot.dir", "/tmp");

    /* Config holds const char*: back them with node-lifetime strings */
    dri_dev_str_ = dri_dev;
    craft_str_ = craft;
    pipe_cfg_.disp.dev = dri_dev_str_.c_str();
    pipe_cfg_.disp.width = disp_w;
    pipe_cfg_.disp.height = disp_h;
    pipe_cfg_.disp.overlay_alpha = (uint8_t)ovl_alpha;
    pipe_cfg_.use_camera = cam_enabled;
    pipe_cfg_.boxes_canvas_space = boxes_canvas;
    pipe_cfg_.sim_boxes = sim_boxes;
    pipe_cfg_.pin_big_cores = pin_cores;
    pipe_cfg_.snap_frame = snap_frame;
    pipe_cfg_.snap_dir = snap_dir;
    pipe_cfg_.layout.craft_name = craft_str_.c_str();
    snprintf(craft_, sizeof craft_, "%.15s", craft.c_str());
    capacity_mah_ = (float)capacity_mah;
    if (sensor == "ar0234")
        pipe_cfg_.cam.sensor = hw::CameraBase::Sensor::Ar0234M;
    else if (sensor == "sc233hgs-isp")
        pipe_cfg_.cam.sensor = hw::CameraBase::Sensor::Sc233hgsIsp;
    else
        pipe_cfg_.cam.sensor = hw::CameraBase::Sensor::Sc233hgs;
    pipe_cfg_.cam.camera_index = cam_index;

    // ---------------- subscriptions (QoS mirrors the publishers) --------
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", best_effort(),
        [this](const sensor_msgs::msg::Imu::SharedPtr m) { onImu(m); });
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10,
        [this](const mavros_msgs::msg::State::SharedPtr m) { onState(m); });
    ext_state_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
        "/mavros/extended_state", 10,
        [this](const mavros_msgs::msg::ExtendedState::SharedPtr m) {
            std::lock_guard<std::mutex> lk(mtx_);
            on_ground_ = m->landed_state ==
                         mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
        });
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
        "/mavros/battery", 10,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr m) { onBattery(m); });
    gpsraw_sub_ = create_subscription<mavros_msgs::msg::GPSRAW>(
        "/mavros/gpsstatus/gpsraw", best_effort(),
        [this](const mavros_msgs::msg::GPSRAW::SharedPtr m) { onGpsRaw(m); });
    navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", best_effort(),
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr m) { onNavSat(m); });
    pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
        "/fpv/static_pressure", best_effort(),
        [this](const sensor_msgs::msg::FluidPressure::SharedPtr m) { onPressure(m); });
    temp_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
        "/fpv/temperature_baro", best_effort(),
        [this](const sensor_msgs::msg::Temperature::SharedPtr m) {
            std::lock_guard<std::mutex> lk(mtx_);
            temperature_c_ = (float)m->temperature;
        });
    llfb_sub_ = create_subscription<quadrotor_msgs::msg::LowLevelFeedback>(
        "/fpv/low_level_feedback", 10,
        [this](const quadrotor_msgs::msg::LowLevelFeedback::SharedPtr m) {
            std::lock_guard<std::mutex> lk(mtx_);
            bat_state_ = m->battery_state;
            if (lipo_cells_ <= 0 && m->battery_voltage > 1.0)
                lipo_cells_ = (int)std::lround(m->battery_voltage / 3.7); /* heuristic */
        });
    tracker_state_sub_ = create_subscription<std_msgs::msg::Int8>(
        "/fpv/tracker_state", 1,
        [this](const std_msgs::msg::Int8::SharedPtr m) { onTrackerState(m); });
    radar_state_sub_ = create_subscription<std_msgs::msg::Int8>(
        "/fpv/radar_state", 1,
        [this](const std_msgs::msg::Int8::SharedPtr m) {
            /* radar tracking promotes the effective tracker state (box colors) */
            std::lock_guard<std::mutex> lk(mtx_);
            if (m->data == 2 && tracker_state_ < 2)
                tracker_state_ = 2;
        });
    trigger_sub_ = create_subscription<std_msgs::msg::Header>(
        "/fpv/tracker_trigger", 10,
        [this](const std_msgs::msg::Header::SharedPtr m) { onTrigger(m); });
    failsafe_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/fpv/failsafe", 10,
        [this](const std_msgs::msg::Bool::SharedPtr m) { onFailsafe(m); });

    snapshot_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/snapshot",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            pipeline_.request_snapshot();
            res->success = true;
            res->message = "snapshot queued";
        });

    // ---------------- 10 Hz telemetry assembly ----------------
    assemble_timer_ = create_wall_timer(100ms, [this] { assembleTelemetry(); });

    // ---------------- render pipeline ----------------
    auto log = [this](const char *level, const std::string &msg) {
        if (strcmp(level, "ERROR") == 0)
            RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
        else
            RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    };
    if (!pipeline_.start(pipe_cfg_, log))
        RCLCPP_ERROR(get_logger(), "pipeline failed to start");
}

/* ------------------------------------------------------------------ */
/* callbacks                                                           */
/* ------------------------------------------------------------------ */
void OsdBridgeNode::onImu(const sensor_msgs::msg::Imu::SharedPtr m)
{
    const auto &q = m->orientation;
    std::lock_guard<std::mutex> lk(mtx_);
    heading_deg_ = Pipeline::HeadingFromQuaternionDeg(q.x, q.y, q.z, q.w);
    /* standard FLU RPY. Sign conventions inherited from the bridge's
     * euler->quaternion; verify against the horizon on the bench. */
    pitch_deg_ = (float)(std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x),
                                              -1.0, 1.0)) * 180.0 / M_PI);
    roll_deg_ = (float)(std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                                   1.0 - 2.0 * (q.x * q.x + q.y * q.y)) *
                        180.0 / M_PI);
}

void OsdBridgeNode::onState(const mavros_msgs::msg::State::SharedPtr m)
{
    rclcpp::Time now = this->now();
    std::lock_guard<std::mutex> lk(mtx_);
    connected_ = m->connected;
    last_state_stamp_ = now;
    snprintf(flight_mode_, sizeof flight_mode_, "%.11s", m->mode.c_str());
    if (m->armed && !armed_) {
        arm_stamp_ = now;
        if (capture_home_on_arm_ && gps_fix_) {
            home_lat_ = lat_;
            home_lon_ = lon_;
            home_valid_ = true;
        }
    }
    armed_ = m->armed;
    if (!armed_)
        home_valid_ = false;
}

void OsdBridgeNode::onBattery(const sensor_msgs::msg::BatteryState::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mtx_);
    batt_v_ = m->voltage;
    current_a_ = std::fabs(m->current);      /* discharge is negative */
    if (std::isfinite(m->charge))
        mah_used_ = (float)(m->charge * 1000.0);
    if (std::isfinite(m->capacity) && m->capacity > 0.f)
        capacity_mah_ = (float)(m->capacity * 1000.0);
}

void OsdBridgeNode::onGpsRaw(const mavros_msgs::msg::GPSRAW::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mtx_);
    gps_fix_ = m->fix_type >= mavros_msgs::msg::GPSRAW::GPS_FIX_TYPE_2D_FIX;
    sats_ = m->satellites_visible;
    gspeed_mps_ = m->vel * 0.01f;            /* cm/s -> m/s */
    course_deg_ = m->cog * 0.01f;
}

void OsdBridgeNode::onNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (m->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        return;
    lat_ = m->latitude;
    lon_ = m->longitude;
}

void OsdBridgeNode::onPressure(const sensor_msgs::msg::FluidPressure::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (baro_ground_ref_) {
        if (!armed_ && on_ground_) {
            /* EMA the ground reference at 100 Hz */
            ground_pa_ = ground_pa_valid_ ? ground_pa_ * 0.95 + m->fluid_pressure * 0.05
                                          : m->fluid_pressure;
            ground_pa_valid_ = true;
        }
        baro_alt_m_ = ground_pa_valid_
                          ? Pipeline::BaroAltM((float)m->fluid_pressure, ground_pa_)
                          : 0.f;
    } else {
        baro_alt_m_ = Pipeline::BaroAltM((float)m->fluid_pressure, 101325.0);
    }
}

void OsdBridgeNode::onTrackerState(const std_msgs::msg::Int8::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mtx_);
    tracker_state_ = m->data;
}

void OsdBridgeNode::onTrigger(const std_msgs::msg::Header::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mtx_);
    trigger_seq_++;
    snprintf(trigger_kind_, sizeof trigger_kind_, "%.7s", m->frame_id.c_str());
}

void OsdBridgeNode::onFailsafe(const std_msgs::msg::Bool::SharedPtr m)
{
    std::lock_guard<std::mutex> lk(mtx_);
    failsafe_ = m->data;
}

/* ------------------------------------------------------------------ */
/* 10 Hz snapshot assembly                                             */
/* ------------------------------------------------------------------ */
void OsdBridgeNode::assembleTelemetry()
{
    osd::layout::Telemetry t;
    rclcpp::Time now = this->now();

    {
        std::lock_guard<std::mutex> lk(mtx_);
        const bool link_lost = last_state_stamp_.nanoseconds() != 0 &&
                               (now - last_state_stamp_).seconds() > 2.0;
        snprintf(t.craft, sizeof t.craft, "%s", craft_);
        t.connected = connected_ && !link_lost;
        t.armed = armed_;
        t.failsafe = failsafe_;
        snprintf(t.flight_mode, sizeof t.flight_mode, "%s",
                 link_lost ? "" : flight_mode_);
        t.batt_v = batt_v_;
        t.cells = lipo_cells_ > 0 ? lipo_cells_ : 0;
        t.current_a = current_a_;
        t.mah_used = mah_used_ < 0 ? 0.f : mah_used_;
        t.capacity_mah = capacity_mah_;
        t.bat_state = bat_state_;
        t.heading_deg = heading_deg_;
        t.pitch_deg = pitch_deg_;
        t.roll_deg = roll_deg_;
        t.baro_alt_m = baro_alt_m_;
        t.temperature_c = temperature_c_;
        t.gps_fix = gps_fix_;
        t.sats = sats_;
        t.gspeed_mps = gspeed_mps_;
        t.course_deg = course_deg_;
        t.lat_deg = lat_;
        t.lon_deg = lon_;
        if (home_valid_ && gps_fix_) {
            t.home_valid = true;
            t.home_bearing_deg = Pipeline::BearingDeg(lat_, lon_,
                                                      home_lat_, home_lon_);
            t.home_dist_m = (float)Pipeline::HaversineM(lat_, lon_,
                                                        home_lat_, home_lon_);
        }
        t.tracker_state = tracker_state_;
        t.trigger_seq = trigger_seq_;
        snprintf(t.trigger_kind, sizeof t.trigger_kind, "%s", trigger_kind_);
        t.arm_s = armed_ && arm_stamp_.nanoseconds() != 0
                      ? (float)(now - arm_stamp_).seconds()
                      : -1.f;

        /* system message line: highest severity wins */
        if (link_lost || !connected_) {
            snprintf(t.message, sizeof t.message, "LINK LOST");
            t.message_level = 3;
        } else if (failsafe_) {
            snprintf(t.message, sizeof t.message, "FAILSAFE");
            t.message_level = 3;
        } else if (bat_state_ == 3) {
            snprintf(t.message, sizeof t.message, "BATT CRITICAL");
            t.message_level = 3;
        } else if (bat_state_ == 2) {
            snprintf(t.message, sizeof t.message, "BATT LOW");
            t.message_level = 2;
        } else if (armed_ && t.arm_s >= 0.f && t.arm_s < 3.f) {
            snprintf(t.message, sizeof t.message, "ARMED");
            t.message_level = 1;
        }
    }

    t.epoch_ns = (uint64_t)now.nanoseconds();
    pipeline_.set_telemetry(t);
}

} // namespace osd_bridge
