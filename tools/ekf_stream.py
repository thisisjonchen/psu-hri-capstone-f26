#!/usr/bin/env python
"""Record EKF inputs and baseline estimates over SSH for flow_capture.py.

Runs on ROS Kinetic with Python 2.7. Messages are saved in callback order.
"""
from __future__ import print_function

import base64
import json
import os
import sys
import threading

TOPICS = (
    '/pidrone/imu_msp', '/pidrone/range',
    '/raspicam_node/motion_vectors', '/raspicam_node/camera_info',
    '/pidrone/state', '/pidrone/localization/odometry',
)


class CaptureWriter(object):
    """Write message schemas and payloads under the caller's lock."""
    def __init__(self, stream, metadata):
        self.stream = stream
        self.sequence = 0
        self.counts = dict((topic, 0) for topic in TOPICS)
        self.schemas = set()
        self.write(dict(kind='session', version=1, topics=TOPICS, metadata=metadata))

    def write(self, record):
        self.stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + '\n')
        self.stream.flush()

    def message(self, topic, type_name, md5sum, definition, payload,
                received_ns, elapsed_ns, header_stamp_ns):
        schema = (type_name, md5sum)
        if schema not in self.schemas:
            self.write(dict(kind='schema', type=type_name, md5sum=md5sum, definition=definition))
            self.schemas.add(schema)
        self.write(dict(kind='message', sequence=self.sequence, topic=topic,
                        type=type_name, md5sum=md5sum, received_ns=received_ns,
                        elapsed_ns=elapsed_ns, header_stamp_ns=header_stamp_ns,
                        payload=base64.b64encode(payload).decode('ascii')))
        self.sequence += 1
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def finish(self, reason):
        self.write(dict(kind='end', reason=reason, messages=self.sequence, counts=self.counts))


def stream_ros(seconds):
    import rospy
    import rospkg
    from roslib.message import get_message_class
    try:
        from time import monotonic
    except ImportError:
        from monotonic import monotonic

    # Reserve stdout for the recording.
    output, sys.stdout = sys.stdout, sys.stderr
    rospy.init_node('ekf_capture', anonymous=True, log_level=rospy.WARN)
    root = rospkg.RosPack().get_path('pidrone_pkg')
    with open(os.path.join(root, 'params', 'multiwii.yaml')) as calibration:
        calibration_yaml = calibration.read()
    metadata = dict(started_ns=rospy.Time.now().to_nsec(),
                    multiwii_yaml=calibration_yaml,
                    parameters=dict((name, rospy.get_param(name, {})) for name in
                                    ('/optical_flow_node', '/fusion_localization', '/raspicam_node')),
                    imu_stamp='MSP packet host receipt; acquisition time unavailable',
                    order='serialized recorder callbacks; not hardware acquisition order')
    writer = CaptureWriter(output, metadata)
    lock = threading.Lock()
    state = dict(accepting=True, failed=False)
    classes = {}
    start = monotonic()

    def receive(raw, topic):
        with lock:
            if not state['accepting']:
                return
            received_ns = rospy.Time.now().to_nsec()
            elapsed_ns = int((monotonic() - start) * 1e9)
            try:
                type_name = raw._connection_header['type']
                md5sum = raw._connection_header['md5sum']
                if type_name not in classes:
                    classes[type_name] = get_message_class(type_name)
                cls = classes[type_name]
                if cls is None or cls._md5sum != md5sum:
                    raise ValueError('message definition unavailable or mismatched: ' + type_name)
                message = cls().deserialize(raw._buff)
                header = getattr(message, 'header', None)
                stamp = header.stamp.to_nsec() if header is not None else None
                writer.message(topic, type_name, md5sum, cls._full_text, raw._buff,
                               received_ns, elapsed_ns, stamp)
            except Exception as error:
                state['failed'] = True
                state['accepting'] = False
                print('Capture failed on %s: %s' % (topic, error), file=sys.stderr)

    subscribers = [rospy.Subscriber(topic, rospy.AnyMsg, receive, callback_args=topic,
                                   queue_size=100, buff_size=2**20) for topic in TOPICS]
    print('Recording sensor callbacks for %g seconds' % seconds, file=sys.stderr)
    waiter = threading.Event()
    try:
        while not rospy.is_shutdown() and not state['failed'] and monotonic() - start < seconds:
            waiter.wait(min(.1, max(0, seconds - (monotonic() - start))))
    finally:
        with lock:
            state['accepting'] = False
            reason = 'error' if state['failed'] else ('shutdown' if rospy.is_shutdown() else 'duration')
            writer.finish(reason)
        for subscriber in subscribers:
            subscriber.unregister()
    return 1 if state['failed'] else 0


if __name__ == '__main__':
    duration = float(sys.argv[1])
    if not 0 < duration <= 3600:
        sys.exit('seconds must be in (0, 3600]')
    sys.exit(stream_ros(duration))
