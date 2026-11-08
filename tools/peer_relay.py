#!/usr/bin/env python3
"""Relay map-frame peer telemetry between drone1 and drone2."""
import argparse
import asyncio
from contextlib import AsyncExitStack
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
PREVIEW_TOPIC = '/pidrone/coordination/peer_preview'


def stamp(header):
    return float(header['stamp']['secs']) + float(header['stamp']['nsecs']) * 1e-9


class Client:
    def __init__(self, socket):
        self.socket, self.latest, self.error = socket, {}, None

    async def start(self):
        for key, (topic, kind) in TOPICS.items():
            await self.send({'op': 'subscribe', 'id': key, 'topic': topic, 'type': kind,
                             'queue_length': 1, 'throttle_rate': 100})
        for topic in (PEER_TOPIC, PREVIEW_TOPIC):
            await self.send({'op': 'advertise', 'topic': topic, 'type': 'pidrone_pkg/PeerState'})
        self.receiver = asyncio.create_task(self.receive())

    async def send(self, message):
        await self.socket.send(json.dumps(message, allow_nan=False))

    async def receive(self):
        keys = {value[0]: key for key, value in TOPICS.items()}
        try:
            async for packet in self.socket:
                message = json.loads(packet)
                if message.get('op') == 'publish' and message.get('topic') in keys:
                    self.update(keys[message['topic']], message['msg'])
        except Exception as error:
            self.error = str(error)
        finally:
            self.error = self.error or 'rosbridge disconnected'

    def update(self, key, message):
        if key == 'anchors' and key in self.latest:
            previous = self.latest[key][0]
            try:
                ids, positions = message['anchor_ids'], message['positions']
                known = dict(zip(previous['anchor_ids'], previous['positions']))
                if (len(previous['anchor_ids']) == len(previous['positions']) == len(known) == 4
                        and len(ids) == len(positions) == len(set(ids)) < 4
                        and message['header']['frame_id'] == previous['header']['frame_id']
                        and fresh(stamp(message['header']), time.time())
                        and all(name in known and all(float(p[axis]) == float(known[name][axis])
                                                     for axis in ('x', 'y', 'z'))
                                for name, p in zip(ids, positions))):
                    # Keep the complete observation's sensor AND receipt timestamps.
                    # Partial reports cannot extend its existing 0.5 s lifetime.
                    return
            except (KeyError, TypeError, ValueError, IndexError, OverflowError):
                pass
        # Changed/invalid reports replace the old layout, requiring a new complete one.
        self.latest[key] = (message, time.monotonic())

    def snapshot(self):
        if self.error:
            raise RuntimeError(self.error)
        return dict(self.latest)


def read_state(snapshot, now, received_now):
    # Generation is latched metadata; live measurements must still arrive promptly.
    for key in TOPICS:
        if key not in snapshot or (key != 'generation' and
                                   not 0 <= received_now - snapshot[key][1] <= STATE_TIMEOUT):
            raise ValueError('missing/stale ' + key)
    data = {key: snapshot[key][0] for key in TOPICS}
    pose, odom, anchors = (data[key] for key in ('pose', 'odom', 'anchors'))
    if not data['ready']['data']:
        raise ValueError('both drones need fresh localization')
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
            'generation': data['generation']['data'], 'layout': layout, 'mode': data['mode']['mode']}


def peer_messages(snapshots, now, received_now):
    states = [read_state(snapshot, now, received_now) for snapshot in snapshots]
    modes = [state['mode'] for state in states]
    if any(mode not in ('DISARMED', 'ARMED', 'FLYING') for mode in modes):
        raise ValueError('both drones need a known flight mode')
    topics = [PEER_TOPIC if mode == 'FLYING' and 'DISARMED' not in modes else PREVIEW_TOPIC
              for mode in modes]
    if (states[0]['layout'] != states[1]['layout'] or
            states[0]['header']['frame_id'] != states[1]['header']['frame_id']):
        raise ValueError('drones must share the same surveyed anchor map')
    messages = []
    for recipient in (0, 1):
        peer = states[1-recipient]
        messages.append({'header': peer['header'], 'pose': peer['pose'], 'velocity': peer['velocity'],
                         'drone_id': 'drone%d' % (2-recipient), 'recipient_id': 'drone%d' % (recipient+1),
                         'recipient_generation': states[recipient]['generation']})
    return topics, messages


async def relay(stop, endpoints=('ws://drone1.local:9090', 'ws://drone2.local:9090')):
    """Wait for both ROS connections and reconnect until the laptop server stops."""
    from websockets.asyncio.client import connect
    from websockets.exceptions import WebSocketException
    previous = None

    def report(status):
        nonlocal previous
        if status != previous:
            print(status, flush=True)
            previous = status

    while not stop.is_set():
        try:
            async with AsyncExitStack() as stack:
                clients = []
                for url in endpoints:
                    if stop.is_set():
                        return
                    # Connect directly to the drones; telemetry still expires after 0.5 s.
                    clients.append(Client(await stack.enter_async_context(connect(
                        url, proxy=None, open_timeout=3, close_timeout=.3, ping_interval=1, ping_timeout=5))))
                    await asyncio.wait_for(clients[-1].start(), timeout=1)
                    stack.callback(clients[-1].receiver.cancel)
                deadline = time.monotonic()
                while not stop.is_set():
                    try:
                        topics, messages = peer_messages([c.snapshot() for c in clients], time.time(), time.monotonic())
                        # Both sends start together; a stalled connection ends this pair.
                        await asyncio.wait_for(asyncio.gather(*(
                            client.send({'op': 'publish', 'topic': topic, 'msg': message})
                            for client, topic, message in zip(clients, topics, messages))), timeout=.1)
                        status = ('Relaying preflight peer telemetry.' if all(t == PREVIEW_TOPIC for t in topics)
                                  else 'Relaying flight peer telemetry.' if all(t == PEER_TOPIC for t in topics)
                                  else 'Relaying takeoff-transition peer telemetry.')
                    except (ValueError, KeyError, TypeError, IndexError, OverflowError) as error:
                        status = 'Peer relay paused: ' + str(error)
                    report(status)
                    deadline = max(deadline + .1, time.monotonic())
                    await asyncio.sleep(max(0., deadline - time.monotonic()))
        except (OSError, RuntimeError, asyncio.TimeoutError, WebSocketException) as error:
            report('Peer relay waiting for both drones: ' + (str(error) or 'send timed out'))
            await asyncio.sleep(2)


def run(stop, endpoints=('ws://drone1.local:9090', 'ws://drone2.local:9090')):
    asyncio.run(relay(stop, endpoints))


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
        print('\nRelay stopped; paired flights will request XY braking on stale peer data.')
    finally:
        stop.set()
    return 0


if __name__ == '__main__':
    sys.exit(main())
