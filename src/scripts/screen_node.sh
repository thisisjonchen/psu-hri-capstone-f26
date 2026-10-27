#!/usr/bin/env bash
# Sourced by pi.screenrc after interactive bash has loaded the ROS environment.
# Also usable from a window's shell: bash screen_node.sh <node>.

# Startup report for the routine window, which the dashboard terminal shows first.
pidrone_node_check() {
    local name nodes pending=() missing=()
    local -A windows=([/flight_controller_node]='1 FC' [/pid_controller]='2 PID'
        [/fusion_localization]='3 SE' [/raspicam_node]='4 Vision' [/vl53l1x]='5 TOF'
        [/rosbridge_websocket]='6 rosbridge' [/web_video_server]='7 web_vid_serv'
        [/apriltag_detection]='8 AprilTags' [/uwb_localization]='9 UWB'
        [/apriltag_mapping]='12 TagMap' [/payload_detection]='14 Payload')
    python "$package_dir/scripts/wait_for_ros_master.py" || return
    echo 'Checking onboard nodes (up to 60 s)...'
    pending=("${!windows[@]}")
    for _ in $(seq 20); do
        nodes=$(rosnode list 2>/dev/null)
        missing=()
        for name in "${pending[@]}"; do
            if grep -qx -- "$name" <<<"$nodes"; then
                printf '  ok       %s\n' "$name"
            else
                missing+=("$name")
            fi
        done
        pending=("${missing[@]}")
        ((${#pending[@]})) || { echo "All ${#windows[@]} nodes are running."; return 0; }
        sleep 3
    done
    for name in "${pending[@]}"; do
        printf '  MISSING  %s (window %s; switch with ` then the number)\n' "$name" "${windows[$name]}"
    done
    return 1
}

pidrone_screen_node() {
    local node=${1:-} package_dir status
    local -a command
    package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd) || return
    cd -- "$package_dir/scripts" || return
    # Serial ports resolved by boot/pals-start.sh when nodes start.
    if [[ -r "$HOME/catkin_ws/pals.env" ]]; then
        set -a
        # shellcheck disable=SC1091
        source "$HOME/catkin_ws/pals.env" || return
        set +a
    fi

    case "$node" in
        shell) ;;
        check) pidrone_node_check; return ;;
        *) echo "Retry after stopping this command: bash screen_node.sh $node" ;;
    esac

    case "$node" in
        core) command=(roscore) ;;
        fc) command=(python -u flight_controller_node.py) ;;
        rigid) command=(python -u rigid_transform_node.py) ;;
        shell) return 0 ;;
        pid) command=(python -u pid_controller.py) ;;
        se) command=(roslaunch --wait "$package_dir/launch/fusion_localization.launch") ;;
        vision) command=(roslaunch --wait "$package_dir/launch/raspicam_node.launch") ;;
        flow) command=(python -u optical_flow_node.py) ;;
        tof)
            if [[ ! -r "$HOME/catkin_ws/install/setup.bash" ]]; then
                echo 'TOF startup failed: missing ~/catkin_ws/install/setup.bash' >&2
                return 1
            fi
            # shellcheck disable=SC1091
            source "$HOME/catkin_ws/install/setup.bash" || return
            command=(roslaunch --wait "$package_dir/launch/tof.launch")
            ;;
        bridge) command=(roslaunch --wait rosbridge_server rosbridge_websocket.launch) ;;
        video) command=(rosrun web_video_server web_video_server) ;;
        tags) command=(roslaunch --wait "$package_dir/launch/apriltag_detection.launch") ;;
        payload) command=(roslaunch --wait "$package_dir/launch/payload_detection.launch") ;;
        tagmap) command=(roslaunch --wait "$package_dir/launch/apriltag_mapping.launch") ;;
        uwb)
            command=(roslaunch --wait "$package_dir/launch/uwb_localization.launch")
            if [[ -n "${PALS_UWB_PORT:-}" ]]; then
                command+=("port:=$PALS_UWB_PORT")
            fi
            ;;
        *) echo 'Usage: bash screen_node.sh {core|fc|pid|se|vision|flow|rigid|tof|bridge|video|tags|payload|tagmap|uwb|shell}' >&2
           return 2 ;;
    esac

    if ! command -v "${command[0]}" >/dev/null; then
        echo "Startup failed: ${command[0]} is not available; check the ROS environment." >&2
        return 127
    fi
    if [[ "$node" != core ]]; then
        python "$package_dir/scripts/wait_for_ros_master.py" || return
    fi

    printf 'Starting:'
    printf ' %q' "${command[@]}"
    printf '\n'
    "${command[@]}"
    status=$?
    echo "$node exited (status $status)."
    return "$status"
}

pidrone_screen_node "$@"
