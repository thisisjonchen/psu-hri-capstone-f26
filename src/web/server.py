#!/usr/bin/env python3
"""Serve the dashboard and laptop Bluetooth calibration on localhost."""
import argparse
import json
import logging
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from calibration.calibration_service import AnchorCalibration


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
        if urlsplit(self.path).path == '/api/calibration':
            self.respond(200, self.server.calibration.status())
        else:
            super().do_GET()

    def do_POST(self):
        if not self.allowed():
            return
        if self.path != '/api/calibration':
            self.send_error(404)
            return
        if (self.headers.get_content_type() != 'application/json' or
                self.headers.get('X-Calibration-Request') != '1'):
            self.send_error(403)
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('Calibration request must be between 1 and 4096 bytes')
            request = json.loads(self.rfile.read(length))
            if isinstance(request, dict) and request.get('action') == 'status':
                self.respond(200, self.server.calibration.status())
                return
            self.server.calibration.request(request)
        except (ValueError, TypeError) as error:
            self.respond(400, {'error': str(error)})
            return
        self.respond(202, {'accepted': True})

    def log_message(self, format, *args):
        if self.path != '/api/calibration':
            super().log_message(format, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--state', type=Path, default=ROOT.parents[1] / '.calibration-state.json')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    handler = partial(DashboardHandler, directory=str(ROOT))
    with ThreadingHTTPServer(('127.0.0.1', args.port), handler) as server:
        server.calibration = AnchorCalibration(args.state)
        print('Dashboard: http://localhost:%d' % server.server_port, flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.calibration.close()


if __name__ == '__main__':
    main()
