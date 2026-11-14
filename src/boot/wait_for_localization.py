#!/usr/bin/env python
"""Wait for localization readiness and a fresh ToF sample after a restart."""
from __future__ import print_function

import sys
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from localization import monotonic
from localization.position import valid_range


def main():
    rospy.init_node('restart_readiness', anonymous=True)
    range_timeout = float(rospy.get_param('/fusion_localization/sensor_timeout', 0.5))
    ready = [False]
    reason = ['No localization diagnostics received; check the SE window']
    range_stamp = [None]

    def local_ready(message):
        ready[0] = message.data

    def range_received(message):
        range_stamp[0] = (message.header.stamp.to_sec()
                          if valid_range(message.range, message.min_range, message.max_range) else None)

    def fresh_range():
        stamp = range_stamp[0]
        return stamp is not None and stamp > 0 and 0 <= rospy.get_time() - stamp <= range_timeout

    def diagnostic(message):
        for status in message.status:
            if status.name == 'fusion_localization':
                reason[0] = status.message

    rospy.Subscriber('/pidrone/localization/fusion_status', DiagnosticArray, diagnostic)
    rospy.Subscriber('/pidrone/localization/local_ready', Bool, local_ready)
    rospy.Subscriber('/pidrone/range', Range, range_received)
    started = monotonic()
    while not rospy.is_shutdown():
        range_ready = fresh_range()
        if ready[0] and range_ready:
            print('EKF initialized facing map +Y at the UWB position.')
            return 0
        elapsed = monotonic() - started
        if elapsed >= 75 or (elapsed >= 15 and not range_ready):
            break
        time.sleep(0.1)

    if fresh_range():
        print('Localization is not ready: ' + reason[0])
    else:
        print('ToF has no fresh valid range; check the TOF terminal. Restart did not complete.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
