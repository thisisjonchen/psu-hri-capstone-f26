#!/usr/bin/env python3
"""Serve the dashboard and start its local AprilTag detectors on demand."""

import argparse
import json
import signal
import subprocess
import sys
import tempfile
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError
from urllib.parse import urlsplit
from urllib.request import urlopen

WEB_ROOT = Path(__file__).resolve().parent
DETECTOR_SCRIPT = WEB_ROOT.parent / "scripts" / "PALS" / "apriltag_detection_drone"
DETECTOR_PORTS = {"drone1": 5001, "drone2": 5002}


class DetectorManager:
    def __init__(self):
        self.processes = {}
        self.lock = threading.Lock()
        self.log_dir = Path(tempfile.mkdtemp(prefix="pals-detectors-"))

    def start(self, drone_id):
        if drone_id not in DETECTOR_PORTS:
            raise ValueError("Unknown drone")
        with self.lock:
            process = self.processes.get(drone_id)
            if process is not None and process.poll() is None:
                return {"drone_id": drone_id, "status": "running"}
            # Leave an already-running standalone detector in place.
            try:
                with urlopen("http://127.0.0.1:{}/zone".format(DETECTOR_PORTS[drone_id]), timeout=0.3) as response:
                    detection = json.loads(response.read(4096))
                    if isinstance(detection, dict) and detection.get("drone_id") == drone_id:
                        return {"drone_id": drone_id, "status": "running"}
            except (URLError, OSError, ValueError):
                pass
            log_path = self.log_dir / (drone_id + ".log")
            with log_path.open("ab") as log:
                process = subprocess.Popen(
                    [sys.executable, "-u", str(DETECTOR_SCRIPT), "--drone", drone_id, "--headless"],
                    stdout=log, stderr=subprocess.STDOUT,
                )
            self.processes[drone_id] = process
            print("Started {} detector; log: {}".format(drone_id, log_path), flush=True)
            return {"drone_id": drone_id, "status": "starting"}

    def close(self):
        with self.lock:
            processes = list(self.processes.values())
            for process in processes:
                if process.poll() is None:
                    process.terminate()
            for process in processes:
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            self.processes.clear()


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB_ROOT), **kwargs)

    def send_json(self, code, data):
        body = json.dumps(data).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        # Only same-origin requests to this loopback server can launch processes.
        port = self.server.server_port
        allowed_hosts = {"127.0.0.1:{}".format(port), "localhost:{}".format(port)}
        host = self.headers.get("Host", "")
        origin = self.headers.get("Origin")
        if host not in allowed_hosts or (origin and origin != "http://" + host):
            self.send_json(403, {"error": "Use the local dashboard to start detectors"})
            return
        parts = urlsplit(self.path).path.strip("/").split("/")
        if len(parts) != 4 or parts[:2] != ["api", "detectors"] or parts[3] != "start":
            self.send_json(404, {"error": "Unknown endpoint"})
            return
        if parts[2] not in DETECTOR_PORTS:
            self.send_json(404, {"error": "Unknown drone"})
            return
        try:
            self.send_json(202, self.server.detectors.start(parts[2]))
        except OSError:
            self.send_json(503, {"error": "Could not start detector"})


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = True

    def __init__(self, port=8765):
        self.detectors = DetectorManager()
        super().__init__(("127.0.0.1", port), DashboardHandler)

    def server_close(self):
        super().server_close()
        self.detectors.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    with DashboardServer(args.port) as server:
        def stop_server(*_):
            raise KeyboardInterrupt

        signal.signal(signal.SIGTERM, stop_server)
        print("Dashboard: http://127.0.0.1:{}/".format(server.server_port), flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
