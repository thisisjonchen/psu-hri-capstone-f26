"""Hardware-free checks for the laptop dashboard server."""
import mimetypes
import sys
import threading
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import DashboardHandler
import peer_relay


class StaticFilesTest(unittest.TestCase):
    def test_browser_assets_override_incorrect_system_mime_types(self):
        handler = DashboardHandler.__new__(DashboardHandler)
        with mock.patch.dict(mimetypes.types_map, {'.js': 'text/plain', '.css': 'text/plain',
                                                   '.svg': 'application/octet-stream'}):
            for name, expected in [('js/main.js', 'text/javascript'),
                                   ('css/main.css', 'text/css'), ('nocamera.svg', 'image/svg+xml')]:
                self.assertEqual(handler.guess_type(name), expected)


class PeerRelayLifecycleTest(unittest.TestCase):
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
        clients[0].send.assert_not_called()
        for index, client in enumerate(clients[1:]):
            client.send.assert_called_once_with({
                'op': 'publish', 'topic': peer_relay.PEER_TOPIC, 'msg': messages[index % 2]})
        for socket in sockets:
            socket.__exit__.assert_called_once()
