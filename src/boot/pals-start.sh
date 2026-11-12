#!/usr/bin/env bash
# Runs on the Pi host for the dashboard's Connect and Restart
# buttons. Starts the pidrone_pkg container and the `pidrone` screen session
# (pi.screenrc) that owns every onboard node. The EKF starts facing map +Y at the
# UWB position. Restart also reboots the warm FC and checks gyro readings.
# Usage: pals-start.sh start|restart|align|warm-reboot|status
set -u
container=pidrone_pkg
session=pidrone
ports=$HOME/catkin_ws/pals.ports # by-id names from tools/pals.py provision
env_file=$HOME/catkin_ws/pals.env # resolved ports, read by screen_node.sh
worker=$(dirname "$(readlink -f "$0")")/../../tools/warm_reboot_fc.py

# A second Connect during a start must wait, then find the session, not start another.
if [[ "${1:-}" != status ]]; then
    exec 9>/tmp/pals-start.lock || exit 1
    if [[ "${1:-}" == start ]]; then
        flock -w 240 9 || { echo "Another startup or maintenance action is still running." >&2; exit 1; }
    else
        flock -n 9 || { echo "Another startup or maintenance action is running; restart was not queued." >&2; exit 1; }
    fi
fi

session_running() {
    # A large descriptor limit makes these screen queries expensive.
    docker exec "$container" bash -c '
        ulimit -Sn 1024 || exit 1
        screen -wipe >/dev/null 2>&1
        exec screen -ls' 2>/dev/null |
        grep -Eq "[0-9]+\.${session}[[:space:]].*\((Attached|Detached|Multi)"
}

# rostopic echo quotes strings (mode: "DISARMED"), so strip the quotes before comparing.
drone_mode() {
    docker exec "$container" bash -ic \
        'timeout 3 rostopic echo -n1 /pidrone/mode 2>/dev/null' 2>/dev/null |
        sed -n 's/^mode: *//p' | tr -d '"' | head -n1
}

# by-id links exist only on the host, so resolve them to /dev/ttyACM* each start.
resolve_ports() {
    [[ -r "$ports" ]] || return 0
    local fc uwb
    # shellcheck disable=SC1090
    source "$ports"
    fc=$(readlink -e "${PALS_FC_ID:-}") || { echo "Flight controller not found: ${PALS_FC_ID:-unset}" >&2; return 1; }
    uwb=$(readlink -e "${PALS_UWB_ID:-}") || { echo "UWB tag not found: ${PALS_UWB_ID:-unset}" >&2; return 1; }
    [[ "$fc" != "$uwb" ]] || { echo "FC and UWB resolve to the same serial device; fix pals.ports." >&2; return 1; }
    printf 'PALS_FC_PORT=%s\nPALS_UWB_PORT=%s\n' "$fc" "$uwb" > "$env_file"
}

wait_for_localization() {
    docker exec -i "$container" bash -ic 'timeout 85 python -' <<'PY'
from __future__ import print_function
import sys
import threading
import math
import time
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
rospy.init_node('restart_readiness', anonymous=True)
ready = threading.Event()
reason = ['No localization diagnostics received; check the SE window']
range_stamp = [None]
def range_received(message):
    value = message.range
    if not math.isnan(value) and not math.isinf(value) and message.min_range <= value <= message.max_range:
        range_stamp[0] = message.header.stamp.to_sec()
def fresh_range():
    return range_stamp[0] is not None and 0 <= rospy.get_time() - range_stamp[0] <= .5
def diagnostic(message):
    for status in message.status:
        if status.name == 'fusion_localization':
            reason[0] = status.message
rospy.Subscriber('/pidrone/localization/fusion_status', DiagnosticArray, diagnostic)
rospy.Subscriber('/pidrone/localization/local_ready', Bool,
                 lambda message: ready.set() if message.data else ready.clear())
rospy.Subscriber('/pidrone/range', Range, range_received)
for attempt in range(750):
    if (ready.is_set() and fresh_range()) or rospy.is_shutdown():
        break
    if attempt >= 150 and not fresh_range():
        break
    time.sleep(.1)
if rospy.is_shutdown() or not ready.is_set() or not fresh_range():
    print(('Localization is not ready: ' + reason[0]) if fresh_range() else
          'ToF has no fresh valid range; check the TOF terminal. Restart did not complete.')
    sys.exit(1)
print('EKF initialized facing map +Y at the UWB position.')
PY
    local status=$?
    [[ $status != 124 ]] || echo "Timed out connecting to localization; check ROS and the SE window." >&2
    return "$status"
}

align_heading() {
    docker exec "$container" screen -S "$session" -p SE -X stuff '^C' || return 1
    # [e] keeps pgrep -f from matching this script's own bash command line.
    docker exec "$container" bash -ic '
        for _ in $(seq 40); do pgrep -f "(fusion|ekf)_localization_nod[e]|roslaunch .*fusion_localization.launc[h]" >/dev/null || break; sleep 0.5; done
        pgrep -f "(fusion|ekf)_localization_nod[e]|roslaunch .*fusion_localization.launc[h]" >/dev/null && { echo "Localization did not stop; check the SE window."; exit 1; }
        screen -S pidrone -p SE -X stuff "^Ubash $HOME/catkin_ws/src/pidrone_pkg/scripts/screen_node.sh se^M"' 2>/dev/null || return 1
    wait_for_localization
}

warm_reboot() {
    result=$(sudo -n python3 "$worker" reboot 2>&1) || { echo "$result"; return 1; }
}

check_gyro() {
    result=$(python3 "$worker" check 2>&1) || { echo "$result"; return 1; }
}

case "${1:-}" in
    status) session_running; exit ;;
    start) ;;
    align)
        session_running || { echo "Nodes are not started."; exit 1; }
        mode=$(drone_mode)
        if [[ "$mode" != DISARMED ]]; then
            echo "Refusing to align: drone mode is ${mode:-unknown}; it must be DISARMED."
            exit 1
        fi
        align_heading
        exit
        ;;
    warm-reboot)
        session_running || { echo "Nodes are not started."; exit 1; }
        warm_reboot || exit 1
        docker exec "$container" screen -S "$session" -p FC -X stuff \
            '^Ubash /home/duckie/catkin_ws/src/pidrone_pkg/scripts/screen_node.sh fc^M' || exit 1
        check_gyro || exit 1
        align_heading || exit 1
        echo "Warm reboot complete. $result"
        exit
        ;;
    restart)
        docker start "$container" >/dev/null || exit 1
        echo "Stopping all nodes, rebooting the FC, and restarting."
        result=$(sudo -n python3 "$worker" restart 2>&1) || { echo "$result"; exit 1; }
        ;;
    *) echo "Usage: $0 start|restart|align|warm-reboot|status" >&2; exit 2 ;;
esac

docker start "$container" >/dev/null || exit 1

# Connect needs rosbridge, so a start succeeds only once it accepts connections.
wait_for_bridge() {
    for _ in $(seq 120); do
        if (exec 3<>/dev/tcp/127.0.0.1/9090) 2>/dev/null; then
            echo "Nodes started; rosbridge is up."
            return 0
        fi
        sleep 0.5
    done
    echo "rosbridge did not start; check screen window 6 (rosbridge)." >&2
    return 1
}

if session_running; then
    echo "Screen session '$session' is already running."
else
    if docker exec "$container" pgrep -x rosmaster >/dev/null 2>&1; then
        echo "ROS is already running outside the '$session' session; stop it or reboot." >&2
        exit 1
    fi
    resolve_ports || exit 1
    # Interactive bash loads the container's ROS environment and enters pidrone_pkg.
    # screen scans the descriptor range when creating windows. Docker's million-
    # descriptor default delays every window; children inherit this soft limit.
    docker exec -d "$container" bash -ic "ulimit -Sn 1024 && exec screen -dmS $session -c pi.screenrc" ||
        { echo "Could not launch the screen session." >&2; exit 1; }
    for _ in $(seq 20); do
        session_running && break
        sleep 0.5
    done
    session_running || { echo "Screen session '$session' did not start." >&2; exit 1; }
    echo "Started screen session '$session'; waiting for rosbridge..."
fi
wait_for_bridge || exit 1
if [[ "$1" == restart ]]; then
    # Observe the final FC instance while the EKF initializes in parallel.
    check_gyro || exit 1
    wait_for_localization || exit 1
    echo "Nodes restarted and localized. $result"
fi
