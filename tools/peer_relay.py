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
from localization.state_checks import fresh, finite, uncertainty, STATE_TIMEOUT


TOPICS = {
    'pose': ('/pidrone/localization/fused', 'geometry_msgs/PoseWithCovarianceStamped'),
    'odom': ('/pidrone/localization/odometry', 'nav_msgs/Odometry'),
    'ready': ('/pidrone/localization/ready', 'std_msgs/Bool'),
    'anchors': ('/pidrone/localization/anchors', 'pidrone_pkg/AnchorLayout'),
    'generation': ('/pidrone/localization/generation', 'std_msgs/UInt32'),
}
PEER_TOPIC = '/pidrone/coordination/peer'
# Handshake messages pass to the other drone unchanged; the drones validate them.
FORWARD = {
    '/pidrone/coordination/plan': ('/pidrone/coordination/peer_plan', 'pidrone_pkg/Plan'),
    '/pidrone/coordination/plan_status': ('/pidrone/coordination/peer_plan_status', 'pidrone_pkg/PlanStatus'),
}


def stamp(header):
    return float(header['stamp']['secs']) + float(header['stamp']['nsecs']) * 1e-9


class Client:
    def __init__(self, socket):
        self.socket, self.latest, self.error = socket, {}, None
        self.peer = None

    async def start(self):
        for key, (topic, kind) in TOPICS.items():
            await self.send({'op': 'subscribe', 'id': key, 'topic': topic, 'type': kind,
                             'queue_length': 1, 'throttle_rate': 100})
        await self.send({'op': 'advertise', 'topic': PEER_TOPIC, 'type': 'pidrone_pkg/PeerState'})
        for topic, (peer_topic, kind) in FORWARD.items():
            await self.send({'op': 'subscribe', 'topic': topic, 'type': kind, 'queue_length': 1})
            await self.send({'op': 'advertise', 'topic': peer_topic, 'type': kind})
        self.receiver = asyncio.create_task(self.receive())

    async def send(self, message):
        await self.socket.send(json.dumps(message, allow_nan=False))

    async def receive(self):
        keys = {value[0]: key for key, value in TOPICS.items()}
        try:
            async for packet in self.socket:
                message = json.loads(packet)
                if message.get('op') != 'publish':
                    continue
                if message.get('topic') in keys:
                    self.latest[keys[message['topic']]] = (message['msg'], time.monotonic())
                elif message.get('topic') in FORWARD and self.peer is not None:
                    # Drop a stalled delivery; the sender re-publishes its state at 10 Hz.
                    try:
                        await asyncio.wait_for(self.peer.send({
                            'op': 'publish', 'topic': FORWARD[message['topic']][0],
                            'msg': message['msg']}), timeout=.1)
                    except asyncio.TimeoutError:
                        pass
        except Exception as error:
            self.error = str(error)
        finally:
            self.error = self.error or 'rosbridge disconnected'

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
                clients[0].peer, clients[1].peer = clients[1], clients[0]
                deadline = time.monotonic()
                while not stop.is_set():
                    try:
                        messages = peer_messages([c.snapshot() for c in clients], time.time(), time.monotonic())
                        # Both sends start together; a stalled connection ends this pair.
                        await asyncio.wait_for(asyncio.gather(*(
                            client.send({'op': 'publish', 'topic': PEER_TOPIC, 'msg': message})
                            for client, message in zip(clients, messages))), timeout=.1)
                        status = 'Relaying peer telemetry.'
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
