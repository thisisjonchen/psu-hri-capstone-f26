#!/usr/bin/env bash
# Launch or retry a node from a Screen window with the ROS environment loaded.

# Startup report for the routine window, which the dashboard terminal shows first.
pidrone_node_check() {
    local name nodes attempt pending=() missing=()
    local -A windows=([/flight_controller_node]='1 FC' [/pid_controller]='2 PID'
        [/fusion_localization]='3 SE' [/ekf_localization]='3 SE' [/raspicam_node]='4 Vision' [/vl53l1x]='5 TOF'
        [/rosbridge_websocket]='6 rosbridge' [/web_video_server]='7 web_vid_serv'
        [/apriltag_detection]='8 AprilTags' [/uwb_localization]='9 UWB'
        [/apriltag_mapping]='12 TagMap')
    python "$package_dir/boot/wait_for_ros_master.py" || return
    echo 'Checking onboard nodes (up to 60 s)...'
    pending=("${!windows[@]}")
    for ((attempt = 0; attempt < 20; attempt++)); do
        nodes=$(rosnode list 2>/dev/null) || nodes=
        nodes=$'\n'"$nodes"$'\n'
        missing=()
        for name in "${pending[@]}"; do
            if [[ $nodes == *$'\n'"$name"$'\n'* ]]; then
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
    local node=${1:-} package_dir status launch
    local -a command launch_args=()
    package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd) || return
    cd -- "$package_dir" || return
    if [[ ${PIDRONE_LIBRARY_PATH+x} ]]; then
        export LD_LIBRARY_PATH="$PIDRONE_LIBRARY_PATH"
        unset PIDRONE_LIBRARY_PATH
    fi
    case ":${PYTHONPATH:-}:" in
        *":$package_dir:"*) ;;
        *) export PYTHONPATH="$package_dir${PYTHONPATH:+:$PYTHONPATH}" ;;
    esac
    # Serial ports resolved by boot/pals-start.sh when nodes start.
    if [[ -r "$HOME/catkin_ws/pals.env" ]]; then
        # shellcheck disable=SC1091
        source "$HOME/catkin_ws/pals.env" || return
        export PALS_FC_PORT PALS_UWB_PORT
    fi

    case "$node" in
        check) pidrone_node_check; return ;;
        core) command=(roscore) ;;
        fc) command=(python -u flight/flight_controller_node.py) ;;
        shell) return 0 ;;
        pid) command=(python -u flight/pid_controller.py) ;;
        se) launch=fusion_localization ;;
        vision) launch=raspicam_node ;;
        tof)
            if [[ ! -r "$HOME/catkin_ws/install/setup.bash" ]]; then
                echo 'TOF startup failed: missing ~/catkin_ws/install/setup.bash' >&2
                return 1
            fi
            # shellcheck disable=SC1091
            source "$HOME/catkin_ws/install/setup.bash" || return
            launch=tof
            ;;
        bridge) launch=rosbridge ;;
        video) command=(rosrun web_video_server web_video_server) ;;
        tags) launch=apriltag_detection ;;
        tagmap) launch=apriltag_mapping ;;
        uwb)
            launch=uwb_localization
            if [[ -n "${PALS_UWB_PORT:-}" ]]; then
                launch_args+=("port:=$PALS_UWB_PORT")
            fi
            ;;
        *) echo 'Usage: bash boot/screen_node.sh {core|fc|pid|se|vision|tof|bridge|video|tags|tagmap|uwb|check|shell}' >&2
           return 2 ;;
    esac
    if [[ -n ${launch:-} ]]; then
        command=(roslaunch --wait "$package_dir/launch/$launch.launch" "${launch_args[@]}")
    fi

    echo "Retry from this window with: bash boot/screen_node.sh $node"
    if ! command -v "${command[0]}" >/dev/null; then
        echo "Startup failed: ${command[0]} is not available; check the ROS environment." >&2
        return 127
    fi
    if [[ "$node" != core ]]; then
        python "$package_dir/boot/wait_for_ros_master.py" || return
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
