#!/usr/bin/env python
"""Follow fixed, mapped floor tags with one drone or a synchronized pair."""
import argparse
import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drone_controller import Drone
from apriltag_path import AprilTagPath, tag_ids


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fly', action='store_true', help='run the flight')
    parser.add_argument('--drones', choices=('single', 'both'), default='single')
    parser.add_argument('--tags', type=tag_ids, required=True, help='ordered tag IDs, e.g. 1,2,3')
    args = parser.parse_args()
    if not args.fly:
        parser.print_help()
        return
    rospy.init_node('prototype_1_apriltag', disable_signals=True)
    drone = Drone()
    path = AprilTagPath(drone, args.tags, args.drones)
    drone.ready()
    try:
        drone.takeoff(hold_xy=True, synchronized_stages=True, preflight=path.prepare, check=path.check)
        drone.move_to_altitude(0.6, check=path.check)
        for index in range(len(args.tags)):
            drone.wait_for_stage('tag-%d' % index, check=path.check)
            path.visit(index)
        drone.wait_for_stage('land', check=path.check)
    finally:
        if drone.mode == 'FLYING':
            try:
                drone.land(hold_xy=True)
            except Exception as error:
                rospy.logerr('Landing needs manual recovery: %s', error)
                while drone.mode != 'DISARMED':
                    rospy.sleep(.5)
        else:
            drone.disarm()


if __name__ == '__main__':
    main()
