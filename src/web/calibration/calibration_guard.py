"""Observe dashboard-selected drones and retain participants until release."""
import json
import logging
import socket
import ssl
import threading
import uuid

from time import monotonic
from urllib.parse import urlsplit

REQUEST = '/pidrone/localization/calibration_owner/request'
OWNER = '/pidrone/localization/calibration_owner/status'
# Calibration is performed disarmed; tolerate brief telemetry gaps without
# changing the stricter freshness limits used by flight/navigation.
CONTROL_STATE_TIMEOUT = 2.0


def connect_bridge(websocket, url):
    """Try all resolved addresses, preserving the bridge hostname for WebSocket/TLS."""
    endpoint = urlsplit(url)
    if endpoint.scheme not in ('ws', 'wss') or not endpoint.hostname:
        raise ValueError('Use a ws:// or wss:// drone rosbridge URL')
    port = endpoint.port or (443 if endpoint.scheme == 'wss' else 80)
    # socket.create_connection retries other addresses even after a timeout.
    stream = socket.create_connection((endpoint.hostname, port), timeout=5)
    try:
        if endpoint.scheme == 'wss':
            stream = ssl.create_default_context().wrap_socket(stream, server_hostname=endpoint.hostname)
        bridge = websocket.create_connection(url, socket=stream, timeout=5)
        bridge.settimeout(1)
        return bridge
    except Exception:
        stream.close()
        raise


class BridgeGuard(object):
    def __init__(self, urls, owner, claimed=False, participants=None):
        import websocket
        self.websocket = websocket
        if not 1 <= len(urls) <= 2 or len(set(urls)) != len(urls):
            raise ValueError('Configure one or two distinct drone rosbridge URLs')
        if participants and any(url not in urls for url in participants):
            raise ValueError('Restore the saved participant bridge addresses before calibration recovery')
        self.stop_event = threading.Event()
        self.lock = threading.RLock()
        self.owner = owner
        self.claimed = claimed
        self.participants = list(participants) if participants else None
        self.selection = []
        self.peers = [{'url': url, 'socket': None, 'mode': None, 'mode_at': 0,
                       'error': 'Connecting...',
                       'owner': None, 'owner_at': 0, 'request': None} for url in urls]
        for peer in self.peers:
            worker = threading.Thread(target=self.watch, args=(peer,))
            worker.daemon = True
            worker.start()

    def send(self, peer, message):
        with self.lock:
            if peer['socket'] is None:
                raise ValueError('Reconnect calibration participant %s' % peer['url'])
            peer['socket'].send(json.dumps(message))

    def watch(self, peer):
        websocket = self.websocket
        while not self.stop_event.is_set():
            socket = None
            try:
                socket = connect_bridge(websocket, peer['url'])
                with self.lock:
                    peer['socket'] = socket
                    peer['error'] = None
                self.send(peer, {'op': 'advertise', 'topic': REQUEST, 'type': 'std_msgs/String'})
                for topic, kind in (('/pidrone/mode', 'pidrone_pkg/Mode'),
                                    (OWNER, 'std_msgs/String')):
                    self.send(peer, {'op': 'subscribe', 'topic': topic, 'type': kind, 'queue_length': 1})
                while not self.stop_event.is_set():
                    try:
                        data = json.loads(socket.recv())
                    except websocket.WebSocketTimeoutException:
                        continue
                    if data.get('op') != 'publish':
                        continue
                    self.receive(peer, data.get('topic'), data.get('msg', {}))
            except Exception as error:
                reason = '%s: %s' % (type(error).__name__, error)
                with self.lock:
                    changed = peer['error'] != reason
                    peer['error'] = reason
                if changed and not self.stop_event.is_set():
                    logging.warning('Calibration rosbridge %s: %s', peer['url'], reason)
            finally:
                with self.lock:
                    peer['socket'] = None
                    peer['mode_at'] = peer['owner_at'] = 0
                if socket is not None:
                    socket.close()
            self.stop_event.wait(1)

    def receive(self, peer, topic, message):
        now = monotonic()
        with self.lock:
            if topic == '/pidrone/mode':
                peer['mode'], peer['mode_at'] = message.get('mode'), now
            elif topic == OWNER and isinstance(message.get('data'), str):
                try:
                    status = json.loads(message['data'])
                    if isinstance(status['owner'], str) and isinstance(status['request'], str):
                        peer['owner'], peer['request'], peer['owner_at'] = status['owner'], status['request'], now
                        if status['owner'] == self.owner:
                            self.claimed = True
                except (ValueError, KeyError, TypeError):
                    pass

    def select(self, urls):
        with self.lock:
            known = {p['url'] for p in self.peers}
            if (not isinstance(urls, list) or len(urls) > len(known) or
                    any(not isinstance(url, str) or url not in known for url in urls) or
                    len(set(urls)) != len(urls)):
                raise ValueError('Select connected dashboard drones for calibration')
            self.selection = list(urls)

    def selected(self, include_new=True):
        with self.lock:
            peers = [p for p in self.peers if
                     (self.participants is not None and p['url'] in self.participants) or
                     ((include_new or self.participants is None) and p['url'] in self.selection)]
            return peers

    def connections(self):
        with self.lock:
            return [{'url': peer['url'], 'connected': peer['socket'] is not None,
                     'error': peer['error']} for peer in self.peers]

    def prepare(self):
        with self.lock:
            # A retry can add a newly connected drone, but cannot drop an existing participant.
            self.participants = [p['url'] for p in self.selected()]
            return list(self.participants)

    def check(self, owned=False, allow_other=False):
        now = monotonic()
        with self.lock:
            for peer in self.selected(include_new=not allow_other):
                if peer['socket'] is None:
                    raise ValueError('Laptop calibration cannot connect to drone rosbridge: %s (%s)' %
                                     (peer['url'], peer.get('error') or 'Disconnected'))
                # Releasing a coordinator reservation does not depend on flight state.
                if allow_other:
                    continue
                if not 0 <= now - peer['mode_at'] <= CONTROL_STATE_TIMEOUT:
                    if not peer['mode_at']:
                        raise ValueError('No flight state received from %s; start its flight controller' % peer['url'])
                    raise ValueError('Flight state delayed from %s (last update %.1f s ago); retry calibration' %
                                     (peer['url'], now - peer['mode_at']))
                if peer['mode'] != 'DISARMED':
                    raise ValueError('Disarm %s before calibration' % peer['url'])
                if not 0 <= now - peer['owner_at'] <= CONTROL_STATE_TIMEOUT or peer['owner'] is None:
                    raise ValueError('Fresh calibration ownership required from %s; update its flight controller' % peer['url'])
                if peer['owner'] not in ('', self.owner):
                    raise ValueError('Anchor calibration is owned by another coordinator')
                if owned and peer['owner'] != self.owner:
                    raise ValueError('Every participating drone must acknowledge calibration ownership; measure again if another drone joined')

    def reserve(self, value):
        with self.lock:
            had_claim = self.claimed or any(p['owner'] == self.owner for p in self.peers)
        try:
            self.change_reservation(value)
        except Exception:
            if value and self.claimed and not had_claim:
                # A competing coordinator may have won one master. Release only our claims.
                try:
                    self.change_reservation(False)
                except Exception:
                    pass  # Keep the reservation if either release is unconfirmed.
            raise

    def change_reservation(self, value):
        if value and self.participants is None:
            self.prepare()
        self.check(allow_other=not value)
        nonce = str(uuid.uuid4())
        with self.lock:
            if value:
                self.claimed = True
            peers = self.selected(include_new=False)
            if not peers:
                # Room setup can run before drones are connected. A drone joining
                # later still has to acknowledge ownership before ranging/writes.
                self.claimed = False
                self.participants = [] if value else None
                return
            for peer in peers:
                peer['owner_at'] = 0
                request = json.dumps({'owner': self.owner, 'action': 'acquire' if value else 'release', 'request': nonce})
                self.send(peer, {'op': 'publish', 'topic': REQUEST, 'msg': {'data': request}})
        deadline = monotonic() + 3
        while monotonic() < deadline:
            # Wait for new acknowledgments after publishing; old ownership cannot confirm a change.
            now = monotonic()
            with self.lock:
                fresh = all(p['request'] == nonce and p['owner_at'] > 0 and
                            0 <= now - p['owner_at'] <= CONTROL_STATE_TIMEOUT
                            for p in peers)
                if fresh:
                    self.check(allow_other=not value)
                    if all(p['owner'] == self.owner if value else
                           p['owner'] != self.owner for p in peers):
                        self.claimed = value
                        if not value:
                            self.participants = None
                        return
                for peer in peers:
                    if (peer['socket'] is None or (value and
                            (not 0 <= now - peer['mode_at'] <= CONTROL_STATE_TIMEOUT or peer['mode'] != 'DISARMED'))):
                        raise ValueError('Fresh disarmed flight state required from %s' % peer['url'])
            if self.stop_event.wait(0.05):
                break
        raise ValueError('Calibration ownership was not acknowledged by every participant')

    def close(self):
        self.stop_event.set()
        with self.lock:
            for peer in self.peers:
                if peer['socket'] is not None:
                    peer['socket'].close()
