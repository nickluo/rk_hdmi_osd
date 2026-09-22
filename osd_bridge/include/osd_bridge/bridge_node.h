/*
 * OsdBridgeNode - subscribes apm_bridge topics, converts them into
 * osd::layout::Telemetry and drives the Pipeline render thread.
 *
 * QoS mirrors the publishers exactly (best_effort for the 100/200 Hz
 * sensor streams - a reliable subscriber would receive NOTHING from a
 * best_effort publisher). Callbacks only lock-and-copy; the 10 Hz
 * assembly timer builds the snapshot handed to the pipeline.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <quadrotor_msgs/msg/low_level_feedback.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <mutex>
#include <string>

#include "osd_bridge/pipeline.h"

namespace osd_bridge {

class OsdBridgeNode : public rclcpp::Node
{
public:
    explicit OsdBridgeNode(const rclcpp::NodeOptions &options =
                               rclcpp::NodeOptions());

private:
    /* ---- ROS I/O ---- */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gpsraw_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::LowLevelFeedback>::SharedPtr llfb_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr tracker_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr radar_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr trigger_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr failsafe_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr snapshot_srv_;
    rclcpp::TimerBase::SharedPtr assemble_timer_;

    /* ---- callbacks ---- */
    void onImu(const sensor_msgs::msg::Imu::SharedPtr m);
    void onState(const mavros_msgs::msg::State::SharedPtr m);
    void onBattery(const sensor_msgs::msg::BatteryState::SharedPtr m);
    void onGpsRaw(const mavros_msgs::msg::GPSRAW::SharedPtr m);
    void onNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr m);
    void onPressure(const sensor_msgs::msg::FluidPressure::SharedPtr m);
    void onTrackerState(const std_msgs::msg::Int8::SharedPtr m);
    void onTrigger(const std_msgs::msg::Header::SharedPtr m);
    void onFailsafe(const std_msgs::msg::Bool::SharedPtr m);
    void assembleTelemetry();

    /* ---- shared state (callback threads write, timer reads) ---- */
    std::mutex mtx_;
    // link / mode
    bool connected_ = false;
    bool armed_ = false;
    bool failsafe_ = false;
    bool on_ground_ = true;
    char flight_mode_[12] = {};
    rclcpp::Time last_state_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time arm_stamp_{0, 0, RCL_ROS_TIME};
    // battery
    float batt_v_ = 0.f;
    float current_a_ = 0.f;
    float mah_used_ = -1.f;
    float capacity_mah_ = 0.f;
    uint8_t bat_state_ = 0;
    // attitude / env
    float heading_deg_ = 0.f, pitch_deg_ = 0.f, roll_deg_ = 0.f;
    float temperature_c_ = 0.f;
    double ground_pa_ = 0.0;          /* EMA while disarmed on ground */
    bool ground_pa_valid_ = false;
    float baro_alt_m_ = 0.f;
    // gps / home
    bool gps_fix_ = false;
    uint8_t sats_ = 0;
    float gspeed_mps_ = 0.f, course_deg_ = 0.f;
    double lat_ = 0.0, lon_ = 0.0;
    bool home_valid_ = false;
    double home_lat_ = 0.0, home_lon_ = 0.0;
    // tracker
    int8_t tracker_state_ = -1;
    uint32_t trigger_seq_ = 0;
    char trigger_kind_[8] = {};

    /* ---- params cached for the timer ---- */
    bool baro_ground_ref_ = true;
    bool capture_home_on_arm_ = true;
    char craft_[16] = {};
    int lipo_cells_ = 0;
    /* Config stores const char* - the backing strings must outlive it */
    std::string dri_dev_str_;
    std::string craft_str_;

    Pipeline pipeline_;
    Pipeline::Config pipe_cfg_;

    /* test seam: publish canned boxes is deliberately NOT a topic yet -
     * inject via boxes.sim (pipeline sim) or a future detector node */
};

} // namespace osd_bridge
