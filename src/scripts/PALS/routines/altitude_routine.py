#!/usr/bin/env python
"""Hold 0.15, 0.40, and 0.15 m for 5 seconds each with relative X/Y hold, then land."""

import argparse
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone


ALTITUDES = (0.15, 0.40, 0.15)  # Downward range readings in meters.
HOLD_SECONDS = 5.0


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
            # move() takes a displacement from the current measured height.
            drone.move(z=altitude - drone.height)
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
