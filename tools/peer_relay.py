#!/usr/bin/env python3
"""Relay map-frame peer telemetry between drone1 and drone2; never send route commands."""
import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'scripts'))
from localization.distance_guard import fresh, finite, uncertainty, STATE_TIMEOUT


TOPICS = {
    'pose': ('/pidrone/localization/fused', 'geometry_msgs/PoseWithCovarianceStamped'),
    'odom': ('/pidrone/localization/odometry', 'nav_msgs/Odometry'),
    'ready': ('/pidrone/localization/ready', 'std_msgs/Bool'),
    'anchors': ('/pidrone/localization/anchors', 'pidrone_pkg/AnchorLayout'),
    'generation': ('/pidrone/localization/generation', 'std_msgs/UInt32'),
    'mode': ('/pidrone/mode', 'pidrone_pkg/Mode'),
}
PEER_TOPIC = '/pidrone/coordination/peer'


def stamp(header):
    return float(header['stamp']['secs']) + float(header['stamp']['nsecs']) * 1e-9


class Client:
    def __init__(self, socket):
        self.socket, self.latest, self.error = socket, {}, None
        self.lock = threading.Lock()
        for key, (topic, kind) in TOPICS.items():
            self.send({'op': 'subscribe', 'id': key, 'topic': topic, 'type': kind,
                       'queue_length': 1, 'throttle_rate': 50})
        self.send({'op': 'advertise', 'topic': PEER_TOPIC, 'type': 'pidrone_pkg/PeerState'})
        threading.Thread(target=self.receive, daemon=True).start()

    def send(self, message):
        self.socket.send(json.dumps(message, allow_nan=False))

    def receive(self):
        keys = {value[0]: key for key, value in TOPICS.items()}
        try:
            for packet in self.socket:
                message = json.loads(packet)
                if message.get('op') == 'publish' and message.get('topic') in keys:
                    with self.lock:
                        self.latest[keys[message['topic']]] = (message['msg'], time.monotonic())
        except Exception as error:
            self.error = str(error)
        finally:
            self.error = self.error or 'rosbridge disconnected'

    def snapshot(self):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            return dict(self.latest)


def read_state(snapshot, now, received_now):
    for key in TOPICS:
        if key not in snapshot or not 0 <= received_now - snapshot[key][1] <= STATE_TIMEOUT:
            raise ValueError('missing/stale ' + key)
    data = {key: snapshot[key][0] for key in TOPICS}
    pose, odom, anchors = (data[key] for key in ('pose', 'odom', 'anchors'))
    if not data['ready']['data'] or data['mode']['mode'] != 'FLYING':
        raise ValueError('both drones need fresh localization and FLYING mode')
    for message in (pose, odom, anchors):
        if not fresh(stamp(message['header']), now):
            raise ValueError('old/future sensor timestamp; check clock synchronization')
    frame = pose['header']['frame_id']
    if not frame or anchors['header']['frame_id'] != frame or odom['child_frame_id'] != 'pidrone_body':
        raise ValueError('inconsistent map/body frames')
    if abs(stamp(pose['header']) - stamp(odom['header'])) > .15:
        raise ValueError('pose/velocity timestamp skew')
    uncertainty(pose['pose']['covariance'])
    q = pose['pose']['pose']['orientation']
    qx, qy, qz, qw = (float(q[k]) for k in ('x', 'y', 'z', 'w'))
    norm = math.hypot(math.hypot(qx, qy), math.hypot(qz, qw))
    if not finite((norm,)) or norm < 1e-6:
        raise ValueError('invalid attitude')
    qx, qy, qz, qw = (v / norm for v in (qx, qy, qz, qw))
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    velocity = odom['twist']['twist']['linear']
    vx, vy = float(velocity['x']), float(velocity['y'])
    world = {'x': math.cos(yaw)*vx-math.sin(yaw)*vy,
             'y': math.sin(yaw)*vx+math.cos(yaw)*vy, 'z': 0.0}
    p = pose['pose']['pose']['position']
    if not finite(tuple(float(p[k]) for k in ('x', 'y', 'z')) + (world['x'], world['y'])):
        raise ValueError('invalid position/velocity')
    if len(set(anchors['anchor_ids'])) != 4 or len(anchors['anchor_ids']) != 4 or len(anchors['positions']) != 4:
        raise ValueError('four distinct surveyed anchors required')
    layout = sorted((key, tuple(float(p[k]) for k in ('x', 'y', 'z')))
                    for key, p in zip(anchors['anchor_ids'], anchors['positions']))
    if not finite(tuple(v for _, p in layout for v in p)):
        raise ValueError('invalid anchor geometry')
    oldest = min((pose['header'], odom['header']), key=stamp)
    return {'header': dict(oldest, frame_id=frame), 'pose': pose['pose'], 'velocity': world,
            'generation': data['generation']['data'], 'layout': layout}


def peer_messages(snapshots, now, received_now):
    states = [read_state(snapshot, now, received_now) for snapshot in snapshots]
    if (states[0]['layout'] != states[1]['layout'] or
            states[0]['header']['frame_id'] != states[1]['header']['frame_id']):
        raise ValueError('drones must share the same surveyed anchor map')
    messages = []
    for recipient in (0, 1):
        peer = states[1-recipient]
        messages.append({'header': peer['header'], 'pose': peer['pose'], 'velocity': peer['velocity'],
                         'drone_id': 'drone%d' % (2-recipient), 'recipient_id': 'drone%d' % (recipient+1),
                         'recipient_generation': states[recipient]['generation']})
    return messages


def run(stop, endpoints=('ws://drone1.local:9090', 'ws://drone2.local:9090'), enabled=None):
    """Wait for both ROS connections and reconnect until the laptop server stops."""
    from websockets.sync.client import connect
    from websockets.exceptions import WebSocketException
    previous = None

    def report(status):
        nonlocal previous
        if status != previous:
            print(status, flush=True)
            previous = status

    while not stop.is_set():
        try:
            with ExitStack() as stack:
                clients = []
                for url in endpoints:
                    if stop.is_set():
                        return
                    # Drone endpoints are on the local network, outside laptop proxies.
                    # Allow delayed pongs; STATE_TIMEOUT still rejects stale telemetry after 0.5 s.
                    clients.append(Client(stack.enter_context(connect(
                        url, proxy=None, open_timeout=3, close_timeout=.3, ping_interval=1, ping_timeout=5))))
                while not stop.is_set():
                    if enabled is not None and not enabled.is_set():
                        report('Peer relay disabled.')
                        stop.wait(.1)
                        continue
                    try:
                        messages = peer_messages([c.snapshot() for c in clients], time.time(), time.monotonic())
                        for client, message in zip(clients, messages):
                            client.send({'op': 'publish', 'topic': PEER_TOPIC, 'msg': message})
                        status = 'Relaying peer telemetry; routes remain under their existing controllers.'
                    except (ValueError, KeyError, TypeError, IndexError, OverflowError) as error:
                        status = 'Peer relay paused: ' + str(error)
                    report(status)
                    stop.wait(.1)
        except (OSError, RuntimeError, WebSocketException) as error:
            report('Peer relay waiting for both drones: ' + str(error))
            stop.wait(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drone1', default='ws://drone1.local:9090')
    parser.add_argument('--drone2', default='ws://drone2.local:9090')
    args = parser.parse_args()
    if args.drone1 == args.drone2:
        parser.error('Use distinct rosbridge endpoints')
    stop = threading.Event()
    try:
        run(stop, (args.drone1, args.drone2))
    except KeyboardInterrupt:
        print('\nRelay stopped; enabled onboard distance controllers will request XY braking on stale peer data.')
    finally:
        stop.set()
    return 0


if __name__ == '__main__':
    sys.exit(main())
