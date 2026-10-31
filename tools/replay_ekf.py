#!/usr/bin/env python3
"""Replay recorded IMU, camera, and range measurements through the EKF.

Usage: python3 tools/replay_ekf.py CAPTURE --calibration PROFILE.json --out NEW_FOLDER
The calibration profile supplies sensor scales, mounting, and the initial state.
Camera input defaults to JPEG images; --camera encoder compares raw encoder vectors.
Image-pair flow rates are fused at their midpoint using the matching IMU interval.
Optional stationary_intervals_s lists verified periods at rest as [start, end)
seconds since the first IMU stamp, for zero velocity and gyro-bias updates.
Disarmed hand-carried motion is not stationary.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import sys

import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'scripts'))
from localization.ekf import ErrorStateEKF, _array
from localization.ekf_replay import DelayedEKF
from localization.ekf_sensors import FloorCamera, FloorRange, mounting_rotation
from flow_capture import read_ekf_events


class CaptureDecoder:
    def __init__(self):
        self.store = get_typestore(Stores.EMPTY)
        self.schemas = {}

    def decode(self, record):
        name = record['type'].replace('/', '/msg/', 1)
        key = (name, record['md5sum'])
        if key not in self.schemas:
            self.store.register(get_types_from_msg(record['definition'], name))
            if self.store.generate_msgdef(name)[1] != record['md5sum']:
                raise ValueError('Recorded schema MD5 mismatch: ' + name)
            self.schemas[key] = record['definition']
        elif self.schemas[key] != record['definition']:
            raise ValueError('Recorded schema changed without an MD5 change')
        message = self.store.deserialize_ros1(record['payload'], name)
        header = getattr(message, 'header', None)
        if header is not None:
            stamp = header.stamp.sec*10**9 + header.stamp.nanosec
            if stamp != record['header_stamp_ns']:
                raise ValueError('Recorded header disagrees with payload timestamp')
        return message


class Replay:
    def __init__(self, profile, camera_source='images'):
        self.profile = profile
        self.camera_topic = {'images': '/raspicam_node/image/compressed',
                             'encoder': '/raspicam_node/motion_vectors_raw'}[camera_source]
        self.rotation = mounting_rotation(profile['rotation_body_msp'])
        self.scale = _array([profile['acceleration_m_s2_per_count'], profile['gyro_rad_s_per_count']], (2,))
        if np.any(self.scale <= 0):
            raise ValueError('Explicit positive sensor scales are required')
        self.acceleration_offset = _array(profile['acceleration_offset_msp_counts'], (3,))
        self.range = FloorRange(profile['range_offset_body'], profile['range_beam_body'])
        self.camera_rotation = mounting_rotation(profile['rotation_body_camera'])
        self.camera_offset = _array(profile['camera_offset_body'], (3,))
        self.camera_delay = float(profile.get('camera_delay_s', 0.))
        if not 0 <= self.camera_delay <= float(profile.get('fusion_delay_s', .25)):
            raise ValueError('Camera delay must fit within the fusion delay')
        self.stationary_intervals = []
        for interval in profile.get('stationary_intervals_s', []):
            start, end = _array(interval, (2,))
            if (start < 0 or end <= start
                    or (self.stationary_intervals and start < self.stationary_intervals[-1][1])):
                raise ValueError('Stationary intervals must be ordered, nonoverlapping [start, end) seconds')
            self.stationary_intervals.append((start, end))
        self.ekf = ErrorStateEKF()
        initial = profile['initial']
        self.ekf.initialize(0., initial['quaternion'], position=initial['position'],
            velocity=initial['velocity'],
            acceleration_bias=initial.get('acceleration_bias', (0., 0., 0.)),
            gyro_bias=initial.get('gyro_bias', (0., 0., 0.)))
        self.timeline = None
        self.origin_ns = None
        self.camera = self.camera_signature = None
        self.counts = Counter()

    def feed(self, record, message, metadata):
        topic, stamp_ns = record['topic'], record['header_stamp_ns']
        events = []
        if topic == '/raspicam_node/motion_vectors':
            if self.camera_topic.endswith('motion_vectors_raw'):
                raise ValueError('Processed motion_vectors cannot drive the EKF camera model. '
                                 'Encoder replay requires an older capture containing motion_vectors_raw.')
            return events
        if topic == self.camera_topic:
            self.counts['camera_received'] += 1
        if topic == '/raspicam_node/camera_info':
            signature = (message.width, message.height, tuple(message.K), tuple(message.D),
                         message.distortion_model)
            if signature != self.camera_signature:
                parameters = metadata.get('parameters', {})
                settings = parameters.get('/optical_flow_node', {})
                frame_rate = parameters.get('/raspicam_node', {}).get('framerate', settings.get('framerate'))
                # Camera offsets here are measured from the IMU.
                self.camera = FloorCamera(message.K, message.D, message.distortion_model,
                    (message.width, message.height), self.camera_rotation, self.camera_offset,
                    frame_interval=1/float(frame_rate),
                    max_floor_distance=float(settings.get('max_floor_distance', 1.5)))
                self.camera_signature = signature
            return events
        if stamp_ns is None or stamp_ns <= 0:
            self.counts['missing_source_stamp'] += 1
            return events
        if topic == '/pidrone/imu_msp':
            force = self.rotation.dot((np.asarray(message.accelerometer)-self.acceleration_offset)*self.scale[0])
            rate = self.rotation.dot(np.asarray(message.gyroscope)*self.scale[1])
            if self.timeline is None:
                self.origin_ns = stamp_ns
                self.timeline = DelayedEKF(self.ekf, delay=self.profile.get('fusion_delay_s', .25))
                self.counts['initialized'] += 1
            else:
                stamp = (stamp_ns-self.origin_ns)/1e9
                stationary = any(start <= stamp < end for start, end in self.stationary_intervals)
                status, events = self.timeline.add_imu(force, rate, stamp, stationary=stationary)
                self.counts[status] += 1
        elif self.timeline is None:
            self.counts['before_initial_imu'] += 1
        elif topic == '/pidrone/range':
            status = self.timeline.add_measurement((stamp_ns-self.origin_ns)/1e9, 'range',
                lambda f, t, m=message: self.range.correct(f, m.range, .02**2, t, m.min_range, m.max_range))
            self.counts[status] += 1
        elif topic == self.camera_topic:
            if self.camera is None:
                self.counts['missing_camera_calibration'] += 1
            else:
                if topic.endswith('image/compressed'):
                    image_stamp = stamp_ns-int(round(self.camera_delay*1e9))
                    sample, reason = self.camera.image_pair(message.data, (image_stamp-self.origin_ns)/1e9)
                    correct = self.camera.correct_image
                else:
                    sample, reason = self.camera.vector_pair(message.x, message.y, message.mbx,
                                                             (stamp_ns-self.origin_ns)/1e9)
                    correct = self.camera.correct_vectors
                if sample is None:
                    self.counts[reason] += 1
                    return events
                update = lambda f, t, s=sample, apply=correct: apply(
                    f, s, t, self.timeline.mean_angular_rate(s[0], s[1]))
                status = self.timeline.add_measurement((sample[0]+sample[1])/2, 'camera', update)
                self.counts[status] += 1
        return events


def replay(folder, profile, out, camera_source='images'):
    """Save EKF estimates, baseline estimates, and update counts."""
    runner, decoder = Replay(profile, camera_source), CaptureDecoder()
    out.mkdir(parents=True)
    metadata, outcomes, baselines = {}, Counter(), 0
    used = {'/pidrone/imu_msp', '/pidrone/range', '/raspicam_node/camera_info',
            runner.camera_topic, '/raspicam_node/motion_vectors',
            '/pidrone/state', '/pidrone/localization/odometry'}
    with (out/'estimates.csv').open('w', newline='') as stream, (out/'baseline.csv').open('w', newline='') as reference:
        writer, baseline = csv.writer(stream), csv.writer(reference)
        writer.writerow(['time_s', 'sensor', 'accepted', 'reason', 'nis'] +
                        ['%s_%s' % (name, axis) for name, axes in
                         [('p', 'xyz'), ('v', 'xyz'), ('q', 'xyzw'), ('ba', 'xyz'), ('bg', 'xyz')]
                         for axis in axes] + ['variance_%d' % i for i in range(15)])
        baseline.writerow(['source_stamp_ns', 'topic', 'pose_frame', 'x', 'y', 'z',
                           'v_right', 'v_forward', 'v_up'])

        def write_events(events):
            for event in events:
                outcomes[event['sensor']+':'+event['reason']] += 1
                writer.writerow([event['stamp'], event['sensor'], event['accepted'], event['reason'], event['nis']] +
                    np.concatenate([event[k] for k in ('position', 'velocity', 'quaternion',
                                                     'acceleration_bias', 'gyro_bias', 'variances')]).tolist())

        for record in read_ekf_events(folder, metadata):
            if record['topic'] not in used:
                continue
            message = decoder.decode(record)
            if record['topic'] in ('/pidrone/state', '/pidrone/localization/odometry'):
                if record['topic'] == '/pidrone/state':
                    p, v = message.pose_with_covariance.pose.position, message.twist_with_covariance.twist.linear
                else:
                    p, v = message.pose.pose.position, message.twist.twist.linear
                baseline.writerow([record['header_stamp_ns'], record['topic'], message.header.frame_id,
                                   p.x, p.y, p.z, v.x, v.y, v.z])
                baselines += 1
            else:
                write_events(runner.feed(record, message, metadata))
        if runner.timeline is None:
            raise ValueError('No raw MSP IMU samples: this capture cannot drive the EKF')
        if not runner.counts['camera_received'] or runner.camera is None:
            raise ValueError('Replay requires camera calibration and '+runner.camera_topic+
                             '; use --camera encoder for recordings with raw vectors only.')
        events, beyond_imu = runner.timeline.finish()
        write_events(events)
    summary = dict(profile=profile, camera_source=camera_source, camera_model='flow_rates',
                   capture_metadata=metadata, origin_ns=runner.origin_ns,
                   arrivals=dict(runner.counts), updates=dict(outcomes),
                   measurements_beyond_final_imu=beyond_imu, baseline_samples=baselines,
                   fault=runner.timeline.fault)
    (out/'summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True)+'\n')
    if runner.timeline.fault:
        raise ValueError('Replay stopped: '+runner.timeline.fault+'; see '+str(out/'summary.json'))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--camera', choices=('images', 'encoder'), default='images',
                        help='camera measurement source (default: images)')
    args = parser.parse_args()
    summary = replay(args.capture, json.loads(args.calibration.read_text()), args.out, args.camera)
    print(json.dumps({'updates': summary['updates'], 'arrivals': summary['arrivals'],
                      'output': str(args.out)}, indent=2))


if __name__ == '__main__':
    main()
