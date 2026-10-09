import contextlib
import io
import runpy
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np
from flask import Flask

SCRIPT = Path(__file__).parents[2] / "scripts" / "PALS" / "apriltag_detection_drone"


class HeadlessDetectorTests(unittest.TestCase):
    def test_each_drone_detects_a_real_tag_without_opening_a_window(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        for drone, tag, port, zone in [("drone1", 0, 5001, "PICKUP"), ("drone2", 1, 5002, "DROP_OFF")]:
            with self.subTest(drone=drone):
                marker = cv2.aruco.generateImageMarker(dictionary, tag, 160)
                frame = np.full((320, 240), 255, dtype=np.uint8)
                frame[80:240, 40:200] = marker
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                camera = Mock()
                camera.isOpened.return_value = True
                camera.read.side_effect = [(True, frame), (False, None)]
                app = Flask("test_" + drone)
                with patch.object(sys, "argv", [str(SCRIPT), "--drone", drone, "--headless"]), \
                        patch("flask.Flask", return_value=app), \
                        patch("werkzeug.serving.make_server") as api, \
                        patch("threading.Thread"), \
                        patch.object(cv2, "VideoCapture", return_value=camera) as capture, \
                        patch.object(cv2, "imshow") as show, \
                        patch.object(cv2, "waitKey") as key, \
                        contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaises(SystemExit) as stopped:
                        runpy.run_path(str(SCRIPT), run_name="__main__")
                    self.assertEqual(stopped.exception.code, 1)
                    api.assert_called_once_with("127.0.0.1", port, app)
                    self.assertIn(drone + ".local:8080", capture.call_args.args[0])
                    self.assertEqual(app.test_client().get("/zone").json,
                                     {"drone_id": drone, "tag_id": tag, "zone": zone})
                    show.assert_not_called()
                    key.assert_not_called()
                    camera.release.assert_called_once()


if __name__ == "__main__":
    unittest.main()
