/*
 * osd::layout::Telemetry - one snapshot of craft state for the OSD.
 *
 * The bridge node converts ROS messages into this POD (unit-converted,
 * ROS-free) and hands it to the render pipeline, which copies it under a
 * mutex at cadence. Deliberately fixed-size: no allocation, no map lookups
 * in the render path. Detection boxes do NOT live here - they change at
 * image rate and travel as a separate vector (see Layout::update).
 */
#pragma once

#include <cstdint>

namespace osd {
namespace layout {

/* battery_state values mirror quadrotor_msgs/LowLevelFeedback:
 * 0 invalid, 1 good, 2 low, 3 critical */
struct Telemetry {
    /* ---- identity / link ---- */
    char    craft[16] = {};        /* ASCII only (font covers 32..126) */
    bool    connected = false;     /* FC link up */
    bool    armed = false;
    bool    failsafe = false;
    char    flight_mode[12] = {};  /* "ACRO" "ANGLE" "HORIZON" "OFFBOARD" */

    /* ---- battery ---- */
    float   batt_v = 0.f;          /* pack voltage, V */
    int     cells = 0;             /* 0 = unknown -> hide per-cell element */
    float   current_a = 0.f;       /* A, >= 0 discharge (bridge de-negates) */
    float   mah_used = 0.f;        /* mAh drawn */
    float   capacity_mah = 0.f;    /* 0 = unknown -> hide usage bar */
    uint8_t bat_state = 0;         /* LowLevelFeedback enum value */

    /* ---- attitude / env ---- */
    float   heading_deg = 0.f;     /* 0..359 compass, 0 = north */
    float   pitch_deg = 0.f;
    float   roll_deg = 0.f;
    float   baro_alt_m = 0.f;      /* m above ground reference */
    float   temperature_c = 0.f;

    /* ---- gps ---- */
    bool    gps_fix = false;
    uint8_t sats = 0;
    float   gspeed_mps = 0.f;      /* ground speed m/s */
    float   course_deg = 0.f;      /* course over ground 0..359 */
    double  lat_deg = 0.0, lon_deg = 0.0;
    float   alt_msl_m = 0.f;

    /* ---- home (bridge captures position at arm, computes bearing/dist) ---- */
    bool    home_valid = false;
    float   home_bearing_deg = 0.f;   /* 0..359 compass, direction TO home */
    float   home_dist_m = 0.f;

    /* ---- tracker / mission ---- */
    int8_t  tracker_state = -1;    /* -1 unknown 0 Detecting 1 Found 2 Tracking 3 Lost */
    uint32_t trigger_seq = 0;      /* +1 per /fpv/tracker_trigger event */
    char    trigger_kind[8] = {};  /* "ON" / "OFF" / "MISSION" */

    /* ---- system / custom message line ---- */
    char    message[48] = {};      /* one-line message, "" = none */
    uint8_t message_level = 0;     /* 1 info (blue) 2 warn (amber) 3 error (red, blinking) */

    /* ---- time ---- */
    uint64_t epoch_ns = 0;         /* frame wall time (0 = caller fallback) */
    float   arm_s = -1.f;          /* s since arm, < 0 = disarmed */
};

} // namespace osd::layout
} // namespace osd
