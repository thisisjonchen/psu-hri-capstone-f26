#!/usr/bin/env python
"""Hold 0.25, 0.50, and 0.25 m for 3 seconds each with relative X/Y hold, then land."""

import argparse
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone


ALTITUDES = (0.25, 0.50, 0.25)
HOLD_SECONDS = 2.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    args = parser.parse_args()
    if not args.fly:
        parser.print_help()
        return

    rospy.init_node('altitude_routine', disable_signals=True)
    drone = Drone()
    drone.ready()
    try:
        drone.takeoff(hold_xy=True)
        for stage, altitude in enumerate(ALTITUDES, 1):
            rospy.loginfo('Altitude stage %d/%d: moving to %.2f m',
                          stage, len(ALTITUDES), altitude)
            drone.move_to_altitude(altitude)
            rospy.loginfo('Altitude stage %d/%d: holding %.2f m for %.0f seconds',
                          stage, len(ALTITUDES), altitude, HOLD_SECONDS)
            drone.hover(HOLD_SECONDS)
    finally:
        if drone.mode == 'FLYING':
            try:
                drone.land(hold_xy=True)
            except Exception as error:
                rospy.logerr('Landing needs manual recovery: %s', error)
                # Keep the heartbeat active for manual landing.
                while drone.mode != 'DISARMED':
                    rospy.sleep(0.5)
        else:
            drone.disarm()


if __name__ == '__main__':
    main()
