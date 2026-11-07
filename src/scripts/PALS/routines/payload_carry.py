#!/usr/bin/env python
"""Paired payload carry: climb, hold while the payload is attached, move forward, land.

Run on both drones (paired) from the dashboard. With --attach-seconds, the
payload hold ends at a fixed time after the synchronized start, so the drones
leave together without exchanging routine state. Without it, each drone moves
as soon as it reaches carry height. A drone that misses a phase deadline lands;
its peer continues its own schedule, so be ready to disarm both.
Test stages: no payload; --distance 0 with a long hold and a static weight;
full carry with --attach-seconds.
"""

import argparse
import math
import os
import sys
import time

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone


# Phase windows (seconds) after the synchronized start.
CLIMB_WINDOW_S = 15.0  # takeoff and climb to 0.914 m usually take about 10 s
# Forward move allowance beyond distance / speed. The move takes about 1 s
# longer than that, leaving a short hold so both drones land together.
LAND_MARGIN_S = 6.0


def check_on_time(deadline, phase):
    late = time.time() - deadline
    if late > 0:
        raise RuntimeError('{} finished {:.1f} s late; drones are out of step'.format(phase, late))


def hold_until(drone, deadline, phase):
    check_on_time(deadline, phase)
    drone.hover(deadline - time.time())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    parser.add_argument('--height', type=float, default=0.914,
                        help='carry height in meters, 0.25-1.0 (default: 0.914, 3 ft)')
    parser.add_argument('--attach-seconds', type=float, default=0.0,
                        help='minimum hold at carry height while the payload is attached; '
                             'time left from a quick climb is added. 0 skips the hold and '
                             'the drones move without re-aligning (default: 0)')
    parser.add_argument('--distance', type=float, default=1.0,
                        help='forward distance in meters; 0 skips the move (default: 1.0)')
    parser.add_argument('--speed', type=float, default=0.05,
                        help='forward speed in m/s; keep below the leader limit of about 0.067 '
                             'so both drones move alike (default: 0.05)')
    args = parser.parse_args()
    values = (args.height, args.attach_seconds, args.distance, args.speed)
    if any(math.isnan(v) or math.isinf(v) for v in values):
        parser.error('arguments must be finite')
    if not 0.25 <= args.height <= 1.0:
        parser.error('--height must be between 0.25 and 1.0 m')
    if args.attach_seconds < 0:
        parser.error('--attach-seconds must not be negative')
    if args.distance != 0 and args.distance <= 0.06:
        parser.error('--distance must be 0 or greater than the 0.06 m arrival tolerance')
    if not 0 < args.speed <= 0.066:
        parser.error('--speed must be positive and at most 0.066 m/s')
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('payload_carry', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        # Climb straight on after takeoff settles. Slack before the climb
        # deadline becomes extra payload hold, so both drones leave together.
        drone.takeoff(hold_xy=True)
        rospy.loginfo('Climbing to %.3f m', args.height)
        drone.move_to_altitude(args.height)
        if args.attach_seconds:
            deadline = drone.start_at + CLIMB_WINDOW_S
            check_on_time(deadline, 'Climb')
            deadline += args.attach_seconds
            rospy.loginfo('At carry height: attach the payload now (%.0f s hold)',
                          deadline - time.time())
            hold_until(drone, deadline, 'Payload hold')
        else:
            deadline = time.time()

        if args.distance:
            move_timeout = args.distance / args.speed + 8.0
            rospy.loginfo('Moving forward %.2f m at %.3f m/s', args.distance, args.speed)
            drone.unlock_xy()
            drone.move_forward(args.distance, speed_limit=args.speed, timeout=move_timeout)
            drone.lock_xy()
            deadline += args.distance / args.speed + LAND_MARGIN_S
            hold_until(drone, deadline, 'Forward move')
        rospy.loginfo('Landing')
    finally:
        drone.finish(hold_xy=True)


if __name__ == '__main__':
    main()
