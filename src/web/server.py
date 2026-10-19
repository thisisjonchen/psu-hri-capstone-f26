#!/usr/bin/env python3
"""Serve the dashboard and laptop Bluetooth calibration on localhost."""
import argparse
import json
import logging
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import threading
from urllib.parse import parse_qs, urlsplit

from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from calibration.calibration_service import AnchorCalibration
import drones


class DashboardHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def allowed(self):
        host = self.headers.get('Host', '')
        allowed = {'localhost:%d' % self.server.server_port, '127.0.0.1:%d' % self.server.server_port}
        origin = self.headers.get('Origin')
        if host not in allowed or (origin is not None and origin != 'http://' + host):
            self.send_error(403)
            return False
        return True

    def respond(self, code, value):
        data = json.dumps(value, allow_nan=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if not self.allowed():
            return
        url = urlsplit(self.path)
        if url.path == '/api/calibration':
            self.respond(200, self.server.calibration.status())
        elif url.path == '/api/routine':
            drone = parse_qs(url.query).get('drone', [''])[0]
            try:
                self.respond(200, drones.status(drone))
            except (ValueError, RuntimeError) as error:
                self.respond(503, {'error': str(error)})
        else:
            super().do_GET()

    def do_POST(self):
        if not self.allowed():
            return
        headers = {'/api/calibration': 'X-Calibration-Request', '/api/execute': 'X-Routine-Request',
                   '/api/nodes': 'X-Routine-Request'}
        if self.path not in headers:
            self.send_error(404)
            return
        if (self.headers.get_content_type() != 'application/json' or
                self.headers.get(headers[self.path]) != '1'):
            self.send_error(403)
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('Request must be between 1 and 4096 bytes')
            request = json.loads(self.rfile.read(length))
            if self.path == '/api/execute':
                drones.execute(request['drone'], request['script'], request['args'])
                self.respond(200, {'started': True})
                return
            if self.path == '/api/nodes':
                self.respond(200, {'output': drones.node_action(request['drone'], request['action'])})
                return
            if isinstance(request, dict) and request.get('action') == 'status':
                self.respond(200, self.server.calibration.status())
                return
            self.server.calibration.request(request)
        except (ValueError, TypeError, KeyError) as error:
            self.respond(400, {'error': str(error)})
            return
        except RuntimeError as error:
            self.respond(409, {'error': str(error)})
            return
        self.respond(202, {'accepted': True})

    def log_message(self, format, *args):
        if urlsplit(self.path).path not in ('/api/calibration', '/api/routine'):
            super().log_message(format, *args)


def terminal(websocket):
    """Bridge one browser terminal to a drone's screen session.

    Text frames are JSON {"input": str} or {"cols": int, "rows": int}; output is binary.
    """
    url = urlsplit(websocket.request.path)
    drone = url.path.rsplit('/', 1)[-1]
    start = parse_qs(url.query).get('start') == ['1']
    size = json.loads(websocket.recv())
    try:
        client, channel = drones.open_terminal(drone, int(size['cols']), int(size['rows']), start)
    except (ValueError, RuntimeError) as error:
        websocket.send('\r\n[%s]\r\n' % error)
        return

    def output():
        try:
            while True:
                data = channel.recv(4096)
                if not data:
                    break
                websocket.send(data)
        except (OSError, ConnectionClosed):
            pass
        websocket.close()

    threading.Thread(target=output, daemon=True).start()
    try:
        for message in websocket:
            message = json.loads(message)
            if 'input' in message:
                channel.send(message['input'].encode('utf-8'))
            else:
                channel.resize_pty(width=int(message['cols']), height=int(message['rows']))
    finally:
        # Detaches screen; anything running on the drone keeps running.
        client.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--state', type=Path, default=ROOT.parents[1] / '.calibration-state.json')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    handler = partial(DashboardHandler, directory=str(ROOT))
    with ThreadingHTTPServer(('127.0.0.1', args.port), handler) as server:
        server.calibration = AnchorCalibration(args.state)
        origins = ['http://localhost:%d' % args.port, 'http://127.0.0.1:%d' % args.port]
        terminals = serve(terminal, '127.0.0.1', args.port + 1, origins=origins)
        threading.Thread(target=terminals.serve_forever, daemon=True).start()
        print('Dashboard: http://localhost:%d' % server.server_port, flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            terminals.shutdown()
            server.calibration.close()


if __name__ == '__main__':
    main()
