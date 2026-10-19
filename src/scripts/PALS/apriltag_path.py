"""Validated, frozen floor-tag routes for the AprilTag prototype (Python 2.7)."""
import math
import os
import time
try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

import rospy
from pidrone_pkg.msg import AnchorLayout, TagDetection
from std_msgs.msg import UInt32


MAX_MAP_AGE = 120.0
SPEED = 0.06


def tag_ids(value):
    ids = [int(part) for part in value.split(',')]
    if not 1 <= len(ids) <= 12 or any(i < 0 for i in ids) or len(set(ids)) != len(ids):
        raise ValueError('Use 1 to 12 distinct, nonnegative tag IDs')
    return ids


def inside(point, anchors, margin=0.15):
    """Require clearance inside the convex, ordered room outline."""
    return all((b['x'] - a['x']) * (point[1] - a['y']) -
               (b['y'] - a['y']) * (point[0] - a['x']) >=
               margin * math.hypot(b['x'] - a['x'], b['y'] - a['y'])
               for a, b in zip(anchors, anchors[1:] + anchors[:1]))


class AprilTagPath(object):
    def __init__(self, drone, ids, mode):
        self.drone, self.ids, self.mode = drone, ids, mode
        host = urlparse(os.environ.get('ROS_MASTER_URI', '')).hostname
        self.own = host[:-6] if host and host.endswith('.local') else host
        self.generation = self.layout = self.detection = None
        self.generation_at = 0
        self.plan = None
        rospy.Subscriber('/pidrone/localization/generation', UInt32, self._generation)
        rospy.Subscriber('/pidrone/localization/anchors', AnchorLayout, self._layout)
        rospy.Subscriber('/pidrone/apriltag/detections', TagDetection, self._detection, queue_size=1)

    def _generation(self, message):
        self.generation, self.generation_at = message.data, time.time()

    def _layout(self, message):
        self.layout = message

    def _detection(self, message):
        self.detection = message

    def prepare(self):
        """Validate the dashboard's common route before the routine arms."""
        plan = self.drone.routine_plan
        if not isinstance(plan, dict) or plan.get('tag_ids') != self.ids:
            raise RuntimeError('Start this routine from the dashboard with a mapped tag route')
        participants = plan.get('ids')
        if (participants not in (['drone1'], ['drone2'], ['drone1', 'drone2']) or
                self.own not in participants or len(participants) != (2 if self.mode == 'both' else 1)):
            raise RuntimeError('Requested drone mode does not match the dashboard participants')
        frame = self.drone._hold_pose().header.frame_id
        if plan.get('frame') != frame or len(plan.get('points', [])) != len(self.ids):
            raise RuntimeError('Invalid tag route frame or points')
        anchors = plan.get('anchors', [])
        if (len(anchors) != 4 or len(set(a['name'] for a in anchors)) != 4 or
                not all(all(not math.isnan(a[k]) and not math.isinf(a[k]) for k in ('x', 'y', 'z'))
                        for a in anchors)):
            raise RuntimeError('Invalid route anchor layout')
        self.plan = plan
        self.check()
        if (self.detection is None or not self.detection.camera_online or
                not 0 <= rospy.get_time() - self.detection.header.stamp.to_sec() <= .5):
            raise RuntimeError('AprilTag camera is not delivering fresh frames')
        offset = (0., 0.)
        if self.mode == 'both':
            settings = rospy.get_param('/pidrone/coordination', {})
            nominal = float(settings.get('nominal_separation_m', 0))
            if (settings.get('enabled') is False or not settings.get('min_distance', 0) < nominal <
                    settings.get('max_distance', 0) or settings.get('drone_id') != self.own):
                raise RuntimeError('Paired tag flight requires valid onboard distance coordination')
            first, second = (plan['starts'][name] for name in participants)
            dx, dy = second[0] - first[0], second[1] - first[1]
            distance = math.hypot(dx, dy)
            if not 0 < distance or abs(distance - nominal) > .06:
                raise RuntimeError('Place drones at nominal separation before starting the tag path')
            side = -.5 if self.own == participants[0] else .5
            offset = (side * nominal * dx / distance, side * nominal * dy / distance)
        self.targets = []
        for expected, point in zip(self.ids, plan['points']):
            values = [point[k] for k in ('x', 'y', 'stamp')]
            if (point['id'] != expected or any(math.isnan(v) or math.isinf(v) for v in values) or
                    point['stamp'] <= 0 or not 0 <= rospy.get_time() - point['stamp'] <= MAX_MAP_AGE):
                raise RuntimeError('Tag coordinates are missing, stale, or invalid; remap the route')
            target = (point['x'] + offset[0], point['y'] + offset[1])
            if not inside(target, anchors):
                raise RuntimeError('Offset tag waypoint is too close to or outside the room boundary')
            self.targets.append(target)
        p = self.drone._hold_pose().pose.pose.position
        start = plan['starts'][self.own]
        if math.hypot(p.x - start[0], p.y - start[1]) > .06 or not inside((p.x, p.y), anchors):
            raise RuntimeError('Drone moved since route planning or is outside the clear room area')

    def check(self):
        """A new room reference invalidates the route; never reinterpret its coordinates."""
        if (self.plan is None or self.generation != self.plan['generations'][self.own] or
                not 0 <= time.time() - self.generation_at <= 1.0):
            raise RuntimeError('Room reference changed or stopped updating')
        layout = self.layout
        if (layout is None or layout.header.frame_id != self.plan['frame'] or
                not 0 <= rospy.get_time() - layout.header.stamp.to_sec() <= .5 or
                len(layout.anchor_ids) != 4 or len(layout.positions) != 4):
            raise RuntimeError('Missing or stale route anchor layout')
        actual = dict(zip([name.upper() for name in layout.anchor_ids], layout.positions))
        for anchor in self.plan['anchors']:
            point = actual.get(anchor['name'].upper())
            if point is None or any(not abs(getattr(point, k) - anchor[k]) <= .02 for k in ('x', 'y', 'z')):
                raise RuntimeError('Route anchor layout changed')

    def visit(self, index):
        self.check()
        target = self.targets[index]
        p = self.drone._hold_pose().pose.pose.position
        distance = math.hypot(target[0] - p.x, target[1] - p.y)
        self.drone.unlock_xy()
        self.drone.move_to(target[0], target[1], self.plan['frame'], speed_limit=SPEED,
                           timeout=max(15., 2 * distance / SPEED + 5.), check=self.check)
        self.drone.lock_xy()
        # Confirm at arrival, not from a tag seen earlier on the route. A tag
        # outside the camera view is a failed checkpoint, not permission to fly on.
        arrived = rospy.get_time()
        deadline = time.time() + 5.
        while time.time() < deadline:
            self.check()
            self.drone.hover(.1)
            msg = self.detection
            if (msg is not None and msg.camera_online and self.ids[index] in msg.tag_ids and
                    arrived < msg.header.stamp.to_sec() <= rospy.get_time() and
                    rospy.get_time() - msg.header.stamp.to_sec() <= .5):
                self.drone.hover(1.)
                return
        raise RuntimeError('Tag %d was not visible at its waypoint' % self.ids[index])
