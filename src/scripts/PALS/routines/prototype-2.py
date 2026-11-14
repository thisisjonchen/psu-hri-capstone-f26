#!/usr/bin/env python
"""Climb toward --height and, once within 0.5 m of it, move --forward meters (+Y) while the
climb finishes; hover until --seconds after the start, then descend to 0.2032 m, move
0.1 m forward and land, disarming at 0.15 m."""

import argparse
import math
import os
import sys
import time

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone


LANDING_STEP_HEIGHT_M = 0.2032 # 8 inches
CONTINUE_BELOW_TARGET_M = 0.5 # Start moving forward this far below --height.
LANDING_STEP_FORWARD_M = 0.1
DISARM_HEIGHT_M = 0.15 # Disarm here while landing, instead of the default ~0.10 m.
MAX_HEIGHT_M = 2.0 # Guards against a mistyped --height.
FORWARD_SPEED_M_S = 0.15 # Drone.move_forward default


def forward(drone, meters):
    drone.unlock_xy()
    rospy.loginfo('Moving forward %.4f m', meters)
    drone.move_forward(meters, timeout=max(15.0, meters / FORWARD_SPEED_M_S + 10.0))
    drone.lock_xy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    parser.add_argument('--height', type=float, required=True,
                        help='hover height in meters, above 0.7032 and at most 2.0; the forward '
                             'move starts once the drone is within 0.5 m of it')
    parser.add_argument('--forward', type=float, required=True,
                        help='forward distance at that height in meters (more than 0.06)')
    parser.add_argument('--seconds', type=float, required=True,
                        help='time from the synchronized start until landing begins, in seconds')
    args = parser.parse_args()
    if any(math.isnan(v) or math.isinf(v) for v in (args.height, args.forward, args.seconds)):
        parser.error('arguments must be finite')
    continue_above = args.height - CONTINUE_BELOW_TARGET_M
    if not LANDING_STEP_HEIGHT_M < continue_above or args.height > MAX_HEIGHT_M:
        parser.error('--height must be above 0.7032 m (so 0.5 m below it clears the 0.2032 m '
                     'landing step) and at most 2.0 m')
    if args.forward <= 0.06:
        parser.error('--forward must be greater than the 0.06 m arrival tolerance')
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('prototype_2', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        # One takeoff ramp to --height, without stopping at the default 0.25 m. Continue
        # once within 0.5 m of it; the climb to --height carries on during the move.
        drone.takeoff(hold_xy=True, height=args.height, continue_above=continue_above)
        forward(drone, args.forward)
        remaining = drone.start_at + args.seconds - time.time()
        if remaining > 0:
            rospy.loginfo('Hovering %.1f s until %.1f s after the start', remaining, args.seconds)
            drone.hover(remaining)
        else:
            rospy.logwarn('Climb and move took %.1f s past --seconds; landing now', -remaining)
        rospy.loginfo('Descending to %.4f m', LANDING_STEP_HEIGHT_M)
        drone.move_to_altitude(LANDING_STEP_HEIGHT_M)
        forward(drone, LANDING_STEP_FORWARD_M)
    finally:
        drone.finish(hold_xy=True, disarm_height=DISARM_HEIGHT_M)


if __name__ == '__main__':
    main()
