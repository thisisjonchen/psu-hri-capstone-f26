"""Initial rosbridge replay must not overlap a normal publisher write."""
import runpy
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace


synchronize_replay = runpy.run_path(str(
    Path(__file__).resolve().parents[2] / 'scripts' / 'rosbridge_websocket.py'
))['synchronize_replay']


class RosbridgeTest(unittest.TestCase):
    def test_replay_waits_for_publish_and_keeps_buffered_messages(self):
        class Listener:
            def __init__(self):
                self.publisher = SimpleNamespace(impl=SimpleNamespace(publock=threading.RLock()))
                self.messages = [False, True]

            def peer_subscribe(self, topic_name, topic_publish, peer_publish):
                for message in self.messages:
                    peer_publish(message)

        synchronize_replay(Listener)
        listener = Listener()
        started, finished = threading.Event(), threading.Event()
        published = []

        def replay():
            started.set()
            try:
                listener.peer_subscribe('/setup', None, published.append)
            finally:
                finished.set()

        with listener.publisher.impl.publock:
            thread = threading.Thread(target=replay)
            thread.daemon = True
            thread.start()
            self.assertTrue(started.wait(1))
            overlapped = finished.wait(.05)
        thread.join(1)
        self.assertFalse(overlapped, 'Replay wrote during a normal publish')
        self.assertFalse(thread.is_alive())
        self.assertEqual(published, [False, True])

        # Preserve errors; the publisher lock must still be released afterward.
        def fail(message):
            raise ValueError('connection failed')
        with self.assertRaisesRegex(ValueError, 'connection failed'):
            listener.peer_subscribe('/setup', None, fail)
        finished.clear()
        thread = threading.Thread(target=replay)
        thread.daemon = True
        thread.start()
        thread.join(1)
        self.assertTrue(finished.is_set())
        self.assertEqual(published, [False, True, False, True])
