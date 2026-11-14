#!/usr/bin/env python
"""Run Kinetic rosbridge with serialized initial subscriber replay."""
import runpy


def synchronize_replay(listener_type):
    replay = listener_type.peer_subscribe

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        # Kinetic's peer_publish bypasses Publisher.publish's lock. Replay must
        # not serialize into the same buffer as a concurrent dashboard publish.
        with self.publisher.impl.publock:
            return replay(self, topic_name, topic_publish, peer_publish)

    listener_type.peer_subscribe = peer_subscribe


if __name__ == '__main__':
    from roslib.packages import find_node
    from rosbridge_library.internal.publishers import PublisherConsistencyListener

    synchronize_replay(PublisherConsistencyListener)
    runpy.run_path(find_node('rosbridge_server', 'rosbridge_websocket')[0],
                   run_name='__main__')
