#!/usr/bin/env bash
# Runs on the Pi host for the dashboard's Connect, Restart nodes and Align +Y
# buttons. Starts the pidrone_pkg container and the `pidrone` screen session
# (pi.screenrc) that owns every onboard node. Fusion aligns map +Y when it starts,
# so `align` restarts only fusion (its SE window).
# Usage: pals-start.sh start|restart|align|status
set -u
container=pidrone_pkg
session=pidrone
ports=$HOME/catkin_ws/pals.ports # by-id names from tools/pals.py provision
env_file=$HOME/catkin_ws/pals.env # resolved ports, read by screen_node.sh

# A second Connect during a start must wait, then find the session, not start another.
if [[ "${1:-}" != status ]]; then
    exec 9>/tmp/pals-start.lock && flock 9 || exit 1
fi

session_running() {
    docker exec "$container" screen -wipe >/dev/null 2>&1
    docker exec "$container" screen -ls 2>/dev/null |
        grep -Eq "[0-9]+\.$session[[:space:]].*\((Attached|Detached|Multi)"
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
    printf 'PALS_FC_PORT=%s\nPALS_UWB_PORT=%s\n' "$fc" "$uwb" > "$env_file"
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
        docker exec "$container" screen -S "$session" -p SE -X stuff '^C'
        # The old offset stays on the parameter server, so clear it to see the new capture.
        # [e] keeps pgrep -f from matching this script's own bash command line.
        docker exec "$container" bash -ic '
            for _ in $(seq 40); do pgrep -f "(fusion|ekf)_localization_nod[e]|roslaunch .*fusion_localization.launc[h]" >/dev/null || break; sleep 0.5; done
            pgrep -f "(fusion|ekf)_localization_nod[e]|roslaunch .*fusion_localization.launc[h]" >/dev/null && { echo "Localization did not stop; check the SE window."; exit 1; }
            rosparam delete /fusion_localization/room_yaw_offset >/dev/null 2>&1
            screen -S pidrone -p SE -X stuff "^Ubash $HOME/catkin_ws/src/pidrone_pkg/scripts/screen_node.sh se^M"
            for _ in $(seq 60); do
                rosparam get /fusion_localization/room_yaw_offset >/dev/null 2>&1 && { echo "Heading aligned with map +Y."; exit 0; }
                sleep 0.5
            done
            echo "Fusion restarted but has not aligned yet; check the SE window."; exit 1' 2>/dev/null
        exit
        ;;
    restart)
        mode=$(drone_mode)
        if [[ "$mode" == ARMED || "$mode" == FLYING ]]; then
            echo "Refusing to restart nodes: drone mode is $mode."
            exit 1
        fi
        echo "Drone mode: ${mode:-unknown (no flight controller)}; restarting nodes."
        docker exec "$container" screen -S "$session" -X quit >/dev/null 2>&1
        for _ in $(seq 30); do
            docker exec "$container" pgrep -x rosmaster >/dev/null 2>&1 || break
            sleep 0.5
        done
        ;;
    *) echo "Usage: $0 start|restart|align|status" >&2; exit 2 ;;
esac

docker start "$container" >/dev/null || exit 1

# Connect needs rosbridge, so a start succeeds only once it accepts connections.
wait_for_bridge() {
    for _ in $(seq 120); do
        if (exec 3<>/dev/tcp/127.0.0.1/9090) 2>/dev/null; then
            echo "Nodes started; rosbridge is up."
            exit 0
        fi
        sleep 0.5
    done
    echo "rosbridge did not start; check screen window 6 (rosbridge)." >&2
    exit 1
}

if session_running; then
    echo "Screen session '$session' is already running."
    wait_for_bridge
fi
if docker exec "$container" pgrep -x rosmaster >/dev/null 2>&1; then
    echo "ROS is already running outside the '$session' session; stop it or reboot." >&2
    exit 1
fi
resolve_ports || exit 1
# Interactive bash loads the container's ROS environment and enters pidrone_pkg.
docker exec -d "$container" bash -ic "screen -dmS $session -c pi.screenrc"
for _ in $(seq 20); do
    if session_running; then
        echo "Started screen session '$session'; waiting for rosbridge..."
        wait_for_bridge
    fi
    sleep 0.5
done
echo "Screen session '$session' did not start." >&2
exit 1
