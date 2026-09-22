#!/usr/bin/env bash
# Cycle /fpv/tracker_state through Detecting->Found->Tracking->Lost and
# watch the OSD boxes + banner change color (needs the bridge running):
#   0 white (scanning) / 1 cyan (found) / 2 green+lock (tracking) / 3 red (lost)
set -u
if ! ros2 topic list 2>/dev/null | grep -q /fpv/tracker_state; then
    echo "no ROS graph or topic missing - source your workspace overlay" >&2
    exit 1
fi
while true; do
    for s in 0 1 2 3; do
        echo "tracker_state <- $s"
        ros2 topic pub -r 2 /fpv/tracker_state std_msgs/msg/Int8 "{data: $s}" &
        pid=$!
        sleep 4
        kill $pid 2>/dev/null
        wait $pid 2>/dev/null
    done
done
