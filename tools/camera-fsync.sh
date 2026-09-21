#!/bin/bash
# camera-fsync.sh - manual control of the dual-camera PWM FSYNC trigger
#
# The rk-camera-sync-pwm driver probes with the 60 Hz trigger output
# deliberately DISABLED; sensors sit in trigger-slave mode waiting for
# pulses. Start the pulses when the capture pipeline is ready to run.
#
# usage: camera-fsync.sh [status|on|off|toggle]
#        (no argument prints status)

SYNC_DIR="/sys/devices/platform/camera-fsin-sync"

cmd="${1:-status}"

if [ ! -d "$SYNC_DIR" ]; then
    echo "[E] $SYNC_DIR not found: driver not probed (CONFIG_PWM_ROCKCHIP_CAMERA_SYNC / DTS node camera-fsin-sync)" >&2
    exit 1
fi

enabled() { cat "$SYNC_DIR/enabled"; }
freq_hz() { awk -v p="$(cat "$SYNC_DIR/period_ns")" 'BEGIN { printf "%.4f", 1000000000 / p }'; }

case "$cmd" in
status)
    printf "camera-fsin-sync: enabled=%s period=%s ns (%s Hz) pulse=%s ns\n" \
        "$(enabled)" "$(cat "$SYNC_DIR/period_ns")" "$(freq_hz)" \
        "$(cat "$SYNC_DIR/pulse_width_ns")"
    ;;
on)
    echo 1 > "$SYNC_DIR/enabled" 2>/dev/null || { echo "[E] enable failed" >&2; exit 1; }
    echo "FSYNC trigger ON ($(freq_hz) Hz, $(cat "$SYNC_DIR/pulse_width_ns") ns pulse)"
    ;;
off)
    echo 0 > "$SYNC_DIR/enabled" 2>/dev/null || { echo "[E] disable failed" >&2; exit 1; }
    echo "FSYNC trigger OFF (sensors free-run)"
    ;;
toggle)
    new=$(( 1 - $(enabled) ))
    echo $new > "$SYNC_DIR/enabled" 2>/dev/null || { echo "[E] toggle failed" >&2; exit 1; }
    echo "FSYNC trigger -> $new"
    ;;
*)
    echo "usage: $0 [status|on|off|toggle]" >&2
    exit 1
    ;;
esac
