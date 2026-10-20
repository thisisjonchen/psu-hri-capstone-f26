"""Hardware-free checks for the laptop dashboard server."""
import mimetypes
import io
import json
from email.message import Message
import subprocess
import sys
import threading
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import DashboardHandler
import peer_relay


class StaticFilesTest(unittest.TestCase):
    def test_server_import_does_not_require_drone_serial_modules(self):
        result = subprocess.run([sys.executable, '-c', '''
import importlib.abc
import sys
class NoDroneSerial(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('serial', 'fcntl', 'termios'):
            raise ModuleNotFoundError('Unavailable on laptop: ' + fullname)
sys.meta_path.insert(0, NoDroneSerial())
import server
'''], cwd=str(Path(__file__).resolve().parents[1]),
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_browser_assets_override_incorrect_system_mime_types(self):
        handler = DashboardHandler.__new__(DashboardHandler)
        with mock.patch.dict(mimetypes.types_map, {'.js': 'text/plain', '.css': 'text/plain',
                                                   '.svg': 'application/octet-stream'}):
            for name, expected in [('js/main.js', 'text/javascript'),
                                   ('css/main.css', 'text/css'), ('nocamera.svg', 'image/svg+xml')]:
                self.assertEqual(handler.guess_type(name), expected)


class ResponseTest(unittest.TestCase):
    def test_closed_browser_connection_ends_response_without_retry(self):
        for stage in ('headers', 'body'):
            for error in (BrokenPipeError, ConnectionResetError, OSError):
                with self.subTest(stage=stage, error=error):
                    handler = DashboardHandler.__new__(DashboardHandler)
                    handler.send_response = mock.Mock()
                    handler.send_header = mock.Mock()
                    handler.end_headers = mock.Mock()
                    handler.wfile = mock.Mock()
                    handler.close_connection = False
                    write = handler.end_headers if stage == 'headers' else handler.wfile.write
                    write.side_effect = error('write failed')
                    if error is OSError:
                        with self.assertRaises(OSError):
                            handler.respond(200, {'running': False})
                    else:
                        handler.respond(200, {'running': False})
                        self.assertTrue(handler.close_connection)
                    write.assert_called_once()
                    if stage == 'headers':
                        handler.wfile.write.assert_not_called()


class PeerRelayLifecycleTest(unittest.TestCase):
    def test_toggle_api_validates_boolean_and_returns_shared_state(self):
        handler = DashboardHandler.__new__(DashboardHandler)
        handler.allowed = lambda: True
        handler.respond = mock.Mock()
        handler.server = mock.Mock(relay_enabled=threading.Event())
        handler.path = '/api/peer-relay'
        for value in (False, True, 'false'):
            body = json.dumps({'enabled': value}).encode()
            handler.headers = Message()
            handler.headers['Content-Type'] = 'application/json'
            handler.headers['X-Routine-Request'] = '1'
            handler.headers['Content-Length'] = str(len(body))
            handler.rfile = io.BytesIO(body)
            handler.do_POST()
            if isinstance(value, bool):
                handler.respond.assert_called_with(200, {'enabled': value})
                handler.do_GET()
                handler.respond.assert_called_with(200, {'enabled': value})
            else:
                self.assertEqual(handler.respond.call_args.args[0], 400)
                self.assertTrue(handler.server.relay_enabled.is_set())

    def test_disabled_relay_stops_forwarding_and_can_resume(self):
        stop, enabled = threading.Event(), threading.Event()
        clients = [mock.Mock(), mock.Mock()]
        messages = [{'drone_id': 'drone2'}, {'drone_id': 'drone1'}]
        waits = 0
        def wait(_):
            nonlocal waits
            waits += 1
            if waits == 1:
                enabled.set()
            elif waits == 2:
                enabled.clear()
            else:
                stop.set()
        with mock.patch('websockets.sync.client.connect', return_value=mock.MagicMock()), \
                mock.patch.object(peer_relay, 'Client', side_effect=clients), \
                mock.patch.object(peer_relay, 'peer_messages', return_value=messages) as states, \
                mock.patch.object(stop, 'wait', side_effect=wait):
            peer_relay.run(stop, enabled=enabled)
        states.assert_called_once()
        for client, message in zip(clients, messages):
            client.send.assert_called_once_with({
                'op': 'publish', 'topic': peer_relay.PEER_TOPIC, 'msg': message})

    def test_waits_for_pair_skips_invalid_data_and_reconnects_after_loss(self):
        stop = threading.Event()
        sockets = [mock.MagicMock() for _ in range(5)]
        clients = [mock.Mock() for _ in sockets]
        for client in clients:
            client.snapshot.return_value = {}
        # First attempt: drone2 is offline. Second: valid pair, then a lost
        # connection. Third: both reconnect with new telemetry subscriptions.
        clients[1].snapshot.side_effect = [{}, {}, RuntimeError('connection lost')]
        messages = [{'drone_id': 'drone2'}, {'drone_id': 'drone1'}]
        attempts = [sockets[0], OSError('drone2 offline')] + sockets[1:]
        with mock.patch('websockets.sync.client.connect', side_effect=attempts) as connect, \
                mock.patch.object(peer_relay, 'Client', side_effect=clients), \
                mock.patch.object(peer_relay, 'peer_messages',
                                  side_effect=[ValueError('stale pose'), messages, messages]) as states:
            def wait(_):
                if states.call_count == 3:
                    stop.set()
            with mock.patch.object(stop, 'wait', side_effect=wait):
                peer_relay.run(stop)
        self.assertEqual(connect.call_count, 6)
        for call in connect.call_args_list:
            self.assertIn('proxy', call.kwargs)
            self.assertIsNone(call.kwargs['proxy'])
        clients[0].send.assert_not_called()
        for index, client in enumerate(clients[1:]):
            client.send.assert_called_once_with({
                'op': 'publish', 'topic': peer_relay.PEER_TOPIC, 'msg': messages[index % 2]})
        for socket in sockets:
            socket.__exit__.assert_called_once()
