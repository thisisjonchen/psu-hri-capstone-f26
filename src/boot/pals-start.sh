#!/usr/bin/env bash
# Pi host entry point for Connect, Restart, and manual maintenance.
set -u
container=pidrone_pkg
session=pidrone
workspace=$HOME/catkin_ws
worker=$(dirname "$(readlink -f "$0")")/../../tools/warm_reboot_fc.py
action=${1:-}

# A second Connect during a start must wait, then find the session, not start another.
if [[ "$action" != status ]]; then
    exec 9>/tmp/pals-start.lock || exit 1
    if [[ "$action" == start ]]; then
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
    local ports=$workspace/pals.ports
    [[ -r "$ports" ]] || return 0
    local fc uwb
    # shellcheck disable=SC1090
    source "$ports" || return 1
    fc=$(readlink -e "${PALS_FC_ID:-}") || { echo "Flight controller not found: ${PALS_FC_ID:-unset}" >&2; return 1; }
    uwb=$(readlink -e "${PALS_UWB_ID:-}") || { echo "UWB tag not found: ${PALS_UWB_ID:-unset}" >&2; return 1; }
    [[ "$fc" != "$uwb" ]] || { echo "FC and UWB resolve to the same serial device; fix pals.ports." >&2; return 1; }
    printf 'PALS_FC_PORT=%s\nPALS_UWB_PORT=%s\n' "$fc" "$uwb" > "$workspace/pals.env"
}

wait_for_localization() {
    docker exec "$container" bash -ic 'timeout 85 python boot/wait_for_localization.py'
    local status=$?
    [[ $status != 124 ]] || echo "Timed out connecting to localization; check ROS and the SE window." >&2
    return "$status"
}

start_screen_node() {
    docker exec "$container" screen -S "$session" -p "$1" -X stuff \
        "^Ubash \"\$HOME/catkin_ws/src/pidrone_pkg/boot/screen_node.sh\" $2^M"
}

align_heading() {
    docker exec "$container" screen -S "$session" -p SE -X stuff '^C' || return 1
    # [e] keeps pgrep -f from matching this script's own bash command line.
    docker exec "$container" bash -ic '
        for _ in $(seq 40); do pgrep -f "(fusion|ekf)_localization_nod[e]|roslaunch .*fusion_localization.launc[h]" >/dev/null || break; sleep 0.5; done
        if pgrep -f "(fusion|ekf)_localization_nod[e]|roslaunch .*fusion_localization.launc[h]" >/dev/null; then
            echo "Localization did not stop; check the SE window."
            exit 1
        fi' 2>/dev/null || return 1
    start_screen_node SE se || return 1
    wait_for_localization
}

run_worker() {
    result=$("$@" 2>&1) || { echo "$result"; return 1; }
}

case "$action" in
    status) session_running; exit ;;
    start|restart) ;;
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
        run_worker sudo -n python3 "$worker" reboot || exit 1
        start_screen_node FC fc || exit 1
        run_worker python3 "$worker" check || exit 1
        align_heading || exit 1
        echo "Warm reboot complete. $result"
        exit
        ;;
    *) echo "Usage: $0 start|restart|align|warm-reboot|status" >&2; exit 2 ;;
esac

docker start "$container" >/dev/null || exit 1
if [[ "$action" == restart ]]; then
    echo "Stopping all nodes, rebooting the FC, and restarting."
    run_worker sudo -n python3 "$worker" restart || exit 1
    docker start "$container" >/dev/null || exit 1
fi

# Connect needs rosbridge, so a start succeeds only once it accepts connections.
wait_for_bridge() {
    local deadline=$((SECONDS + 60))
    until ((SECONDS >= deadline)); do
        if (exec 3<>/dev/tcp/127.0.0.1/9090) 2>/dev/null; then
            echo "Nodes started; rosbridge is up."
            return 0
        fi
        sleep 0.25
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
    # Load ROS once. Pass the library path separately for setgid Screen builds.
    docker exec -d "$container" bash -ic \
        'export PIDRONE_LIBRARY_PATH="${LD_LIBRARY_PATH-}"
         ulimit -Sn 1024 && exec screen -dmS "$1" -c boot/pi.screenrc' bash "$session" ||
        { echo "Could not launch the screen session." >&2; exit 1; }
    deadline=$((SECONDS + 10))
    until session_running; do
        ((SECONDS < deadline)) || { echo "Screen session '$session' did not start." >&2; exit 1; }
        sleep 0.25
    done
    echo "Started screen session '$session'; waiting for rosbridge..."
fi
wait_for_bridge || exit 1
if [[ "$action" == restart ]]; then
    # Observe the final FC instance while the EKF initializes in parallel.
    run_worker python3 "$worker" check || exit 1
    wait_for_localization || exit 1
    echo "Nodes restarted and localized. $result"
fi
