#!/usr/bin/env bash
# Sourced by pi.screenrc after interactive bash has loaded the ROS environment.
# Also usable from a window's shell: bash screen_node.sh <node>.

pidrone_screen_node() {
    local node=${1:-} package_dir status
    local -a command
    package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd) || return
    cd -- "$package_dir/scripts" || return

    case "$node" in
        fc|shell) ;;
        *) echo "Retry after stopping this command: bash screen_node.sh $node" ;;
    esac

    case "$node" in
        core) command=(roscore) ;;
        fc)
            echo 'Manual start: python flight_controller_node.py'
            return 0
            ;;
        rigid) command=(python -u rigid_transform_node.py) ;;
        shell) return 0 ;;
        pid) command=(python -u pid_controller.py) ;;
        se) command=(python -u state_estimator.py -p ema) ;;
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
        *) echo 'Usage: bash screen_node.sh {core|fc|pid|se|vision|flow|rigid|tof|bridge|video|tags|shell}' >&2
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
