import importlib.util
import json
import shutil
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

spec = importlib.util.spec_from_file_location("dashboard_server", Path(__file__).parents[1] / "server.py")
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class DetectorManagerTests(unittest.TestCase):
    def setUp(self):
        self.manager = server.DetectorManager()
        self.addCleanup(self.manager.close)
        self.addCleanup(lambda: shutil.rmtree(self.manager.log_dir))
        self.probe = patch.object(server, "urlopen", side_effect=URLError("offline")).start()
        self.addCleanup(patch.stopall)
        self.spawn = patch.object(server.subprocess, "Popen").start()

    def process(self):
        process = Mock()
        process.poll.return_value = None
        return process

    def test_two_drone_commands_are_fixed_and_reconnections_do_not_duplicate(self):
        a, b = self.process(), self.process()
        self.spawn.side_effect = [a, b]
        self.assertEqual(self.manager.start("drone1")["status"], "starting")
        self.assertEqual(self.manager.start("drone1")["status"], "running")
        self.manager.start("drone2")
        self.assertEqual(self.spawn.call_count, 2)
        commands = [call.args[0] for call in self.spawn.call_args_list]
        self.assertEqual(commands[0][-3:], ["--drone", "drone1", "--headless"])
        self.assertEqual(commands[1][-3:], ["--drone", "drone2", "--headless"])
        self.assertEqual(commands[0][0], server.sys.executable)

    def test_exited_detector_restarts_and_shutdown_only_terminates_owned_children(self):
        a, replacement, b = self.process(), self.process(), self.process()
        self.spawn.side_effect = [a, replacement, b]
        self.manager.start("drone1")
        a.poll.return_value = 1
        self.manager.start("drone1")
        self.manager.start("drone2")
        self.manager.close()
        a.terminate.assert_not_called()
        replacement.terminate.assert_called_once()
        b.terminate.assert_called_once()
        self.assertEqual(self.manager.processes, {})

    def test_matching_standalone_detector_is_reused_but_wrong_drone_is_not(self):
        response = Mock()
        response.read.return_value = b'{"drone_id":"drone1"}'
        self.probe.side_effect = None
        self.probe.return_value.__enter__ = Mock(return_value=response)
        self.probe.return_value.__exit__ = Mock(return_value=False)
        self.assertEqual(self.manager.start("drone1")["status"], "running")
        self.spawn.assert_not_called()
        self.spawn.return_value = self.process()
        self.manager.start("drone2")
        self.spawn.assert_called_once()

    def test_unknown_id_cannot_launch_a_process(self):
        with self.assertRaises(ValueError):
            self.manager.start("arbitrary-script")
        self.spawn.assert_not_called()


class DashboardHTTPTests(unittest.TestCase):
    def setUp(self):
        self.http = server.DashboardServer(0)
        self.logs = self.http.detectors.log_dir
        self.http.detectors = Mock()
        self.http.detectors.start.side_effect = lambda drone: {"drone_id": drone, "status": "starting"}
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.thread.start()
        self.url = "http://127.0.0.1:{}".format(self.http.server_port)

    def tearDown(self):
        self.http.shutdown()
        self.http.server_close()
        self.thread.join()
        shutil.rmtree(self.logs)

    def post(self, path, origin=None, host=None):
        headers = {"Origin": origin or self.url}
        if host:
            headers["Host"] = host
        return urlopen(Request(self.url + path, data=b"", headers=headers, method="POST"), timeout=2)

    def test_dashboard_assets_and_start_both_drone_endpoints(self):
        with urlopen(self.url) as response:
            self.assertIn(b"PALS Flight Dashboard", response.read())
        with urlopen(self.url + "/js/main.js") as response:
            self.assertIn(b"requestDetectorStart", response.read())
        for drone in ["drone1", "drone2"]:
            with self.post("/api/detectors/" + drone + "/start") as response:
                self.assertEqual(response.status, 202)
                self.assertEqual(json.load(response)["drone_id"], drone)
        self.assertEqual(self.http.detectors.start.call_count, 2)

    def test_unknown_drone_and_foreign_origins_cannot_start_a_detector(self):
        for path, origin, host, code in [
            ("/api/detectors/drone3/start", None, None, 404),
            ("/api/detectors/drone1/start", "http://example.com", None, 403),
            ("/api/detectors/drone1/start", None, "example.com", 403),
        ]:
            with self.assertRaises(HTTPError) as error:
                self.post(path, origin, host)
            self.assertEqual(error.exception.code, code)
        self.http.detectors.start.assert_not_called()

    def test_spawn_failure_is_a_service_error(self):
        self.http.detectors.start.side_effect = OSError("Cannot spawn")
        with self.assertRaises(HTTPError) as error:
            self.post("/api/detectors/drone1/start")
        self.assertEqual(error.exception.code, 503)


if __name__ == "__main__":
    unittest.main()
