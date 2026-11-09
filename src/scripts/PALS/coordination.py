"""Timed leader/follower plan handshake. Compatible with Python 2.7.

The leader proposes an action starting at a shared wall-clock time; the follower
accepts it; both run it from that time for the same duration, then hold for the
plan's hold_s and land unless the next plan starts first. Handshakes happen only
while held: DISARMED on the ground, or FLYING with the X/Y lock. Any failure
raises RuntimeError so the routine's `finally: drone.finish()` lands the drone.
Blocks live in actions.py; routines `import actions` to register them.
"""
import math
import os
import time

import rospy
from pidrone_pkg.msg import Plan, PlanStatus
try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

ACCEPT_MARGIN_S = 1.0   # follower accepts only with this much time before the start
COMMIT_MARGIN_S = 0.5   # leader needs the acceptance this long before the start
MIN_LEAD_S = 2.0        # shortest lead time a routine may request
MAX_LEAD_S = 60.0       # follower rejects starts further away (clock fault)
MAX_HOLD_S = 300.0      # longest hold a plan may request after its action
START_CONFIRM_S = 0.5   # peer must report executing this soon after the start
PEER_TIMEOUT_S = 0.5    # peer status older than this counts as lost
PEER_WAIT_S = 60.0      # wait this long for the peer's routine to appear
OVERRUN_S = 0.5         # an action may finish this late before it fails
STEP_S = 0.05
LEADER, FOLLOWER = 'drone1', 'drone2'
FAILED_STATES = ('rejected', 'cancelled', 'failed')


class Action(object):
    """Base for plan actions; register subclasses with @register.

    prepare(drone, params) runs on the leader only and returns the plan params.
    validate(drone, params) raises ValueError for unusable params or state.
    duration(params) must give the same seconds on both drones.
    run(drone, start, end, params, check) calls check() every cycle and must
    leave the drone held (X/Y locked if flying) by `end`.
    """
    name = None

    def prepare(self, drone, params):
        return params

    def validate(self, drone, params):
        pass

    def duration(self, params):
        raise NotImplementedError

    def run(self, drone, start, end, params, check):
        raise NotImplementedError


ACTIONS = {}


def register(cls):
    ACTIONS[cls.name] = cls()
    return cls


def valid_hold(hold_s):
    if math.isnan(hold_s) or not 0 <= hold_s <= MAX_HOLD_S:
        raise ValueError('Hold must be 0-{:.0f} s'.format(MAX_HOLD_S))


def identity(master_uri=None):
    host = urlparse(master_uri or os.environ.get('ROS_MASTER_URI', '')).hostname or ''
    own = host[:-6] if host.endswith('.local') else host
    if own not in (LEADER, FOLLOWER):
        raise ValueError('Cannot identify drone1/drone2 from ROS_MASTER_URI')
    return own


class Handshake(object):
    def __init__(self, drone, role):
        if not ACTIONS:
            raise RuntimeError('No actions registered; import actions before creating Leader/Follower')
        self.drone_id = identity()
        if self.drone_id != role:
            raise RuntimeError('{} cannot run the {} handshake'.format(
                self.drone_id, 'leader' if role == LEADER else 'follower'))
        self.peer_id = FOLLOWER if role == LEADER else LEADER
        self.drone = drone
        self.status = (0, 'idle', '')
        self.peer = None  # (PlanStatus, local receipt time)
        self.deadline = None  # the next plan must start by this time
        self.status_pub = rospy.Publisher('/pidrone/coordination/plan_status', PlanStatus, queue_size=1)
        rospy.Subscriber('/pidrone/coordination/peer_plan_status', PlanStatus, self._peer_received, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), lambda _: self._publish())

    def _peer_received(self, msg):
        if msg.drone_id == self.peer_id:
            self.peer = (msg, time.time())

    def _publish(self):
        msg = PlanStatus()
        msg.header.stamp = rospy.Time.from_sec(time.time())
        msg.drone_id = self.drone_id
        msg.plan_id, msg.state, msg.reason = self.status
        self.status_pub.publish(msg)

    def _set(self, plan_id, state, reason=''):
        self.status = (plan_id, state, reason)
        self._publish()
        if state in FAILED_STATES:
            rospy.logwarn('Plan %d %s: %s', plan_id, state, reason)

    def _fail(self, plan_id, state, reason):
        self._set(plan_id, state, reason)
        raise RuntimeError('Plan {} {}: {}'.format(plan_id, state, reason))

    def _report_failure(self, error):
        # Errors outside a plan still tell the peer to land.
        if self.status[1] not in FAILED_STATES:
            self._set(self.status[0], 'failed', str(error))

    def _peer_status(self):
        """Fresh peer status, or raise if it was seen and then lost."""
        if self.peer is None:
            return None
        msg, received = self.peer
        if not 0 <= time.time() - received <= PEER_TIMEOUT_S:
            raise RuntimeError('Peer plan status is stale')
        return msg

    def _wait_for_peer(self):
        deadline = time.time() + PEER_WAIT_S
        while self.peer is None:
            if time.time() > deadline:
                raise RuntimeError('Peer routine did not appear within {:.0f} s'.format(PEER_WAIT_S))
            self._hold(STEP_S)

    def _check_held(self):
        drone = self.drone
        if drone.mode == 'FLYING':
            if drone.xy_anchor is None:
                raise RuntimeError('Handshake requires the X/Y lock in flight')
            drone._check_flight('handshake')
        elif drone.mode == 'DISARMED':
            drone._check_sensors()
        else:
            raise RuntimeError('Handshake requires DISARMED on the ground or a held hover')

    def _hold(self, seconds):
        self._check_held()
        if self.drone.mode == 'FLYING':
            self.drone.hover(seconds)
        else:
            time.sleep(seconds)

    def _hold_until(self, deadline, check):
        while True:
            check()
            remaining = deadline - time.time()
            if remaining <= 0:
                return
            self._hold(min(STEP_S, remaining))

    def _check_hold(self, start=None):
        """Raise once the hold after the last plan has expired (or would before start)."""
        if self.deadline is None:
            return
        if time.time() >= self.deadline:
            raise RuntimeError('No new plan started within the hold')
        if start is not None and start > self.deadline:
            raise ValueError('start is {:.2f} s after the hold ends'.format(start - self.deadline))

    def _check_idle(self):
        peer = self._peer_status()
        if peer is not None and peer.state in FAILED_STATES:
            raise RuntimeError('Peer plan {} {}: {}'.format(peer.plan_id, peer.state, peer.reason))

    def _check_peer(self, plan_id, start):
        peer = self._peer_status()
        if peer is None:
            raise RuntimeError('Peer plan status is missing')
        if peer.plan_id == plan_id and peer.state in FAILED_STATES:
            raise RuntimeError('Peer {} plan {}: {}'.format(peer.state, plan_id, peer.reason))
        # A peer that finished first may already report the routine end or the next plan.
        if time.time() >= start + START_CONFIRM_S and not (peer.plan_id > plan_id or (
                peer.plan_id == plan_id and peer.state in ('executing', 'done', 'finished'))):
            raise RuntimeError('Peer is not executing plan {}'.format(plan_id))

    def _execute(self, plan_id, action, params, start, hold_s):
        end = start + action.duration(params)
        check = lambda: self._check_peer(plan_id, start)
        try:
            self._hold_until(start, check)
            self._set(plan_id, 'executing')
            rospy.loginfo('Plan %d %s started %.3f s after %.3f', plan_id, action.name,
                          time.time() - start, start)
            action.run(self.drone, start, end, params, check)
            if time.time() > end + OVERRUN_S:
                raise RuntimeError('{} overran by {:.2f} s'.format(action.name, time.time() - end))
            self._hold_until(end, check)
        except Exception as error:
            self._fail(plan_id, 'failed', str(error))
        self.deadline = end + hold_s
        self._set(plan_id, 'done')
        rospy.loginfo('Plan %d %s done; holding up to %.1f s', plan_id, action.name, hold_s)


class Leader(Handshake):
    """Each registered block is a method: pair.takeoff(0.5), pair.land().
    lead_s and hold_s default to the constructor's and may be passed per call."""
    def __init__(self, drone, lead_s=10.0, hold_s=30.0):
        Handshake.__init__(self, drone, LEADER)
        self.lead_s, self.hold_s = lead_s, hold_s
        self.plan_pub = rospy.Publisher('/pidrone/coordination/plan', Plan, queue_size=1)

    def __getattr__(self, name):
        if name not in ACTIONS:
            raise AttributeError(name)

        def block(*params, **timing):
            lead_s, hold_s = timing.pop('lead_s', self.lead_s), timing.pop('hold_s', self.hold_s)
            if timing:
                raise TypeError('{}() got unexpected arguments: {}'.format(name, ', '.join(timing)))
            self.run(name, params, lead_s, hold_s)
        return block

    def run(self, name, params, lead_s, hold_s):
        """Propose `name` to start lead_s from now, run it with the follower, then
        hold up to hold_s for the routine's next plan (or finish())."""
        try:
            self._run(name, params, lead_s, hold_s)
        except Exception as error:
            self._report_failure(error)
            raise

    def _run(self, name, params, lead_s, hold_s):
        if name not in ACTIONS:
            raise ValueError('Unknown action: {}'.format(name))
        if math.isnan(lead_s) or not MIN_LEAD_S <= lead_s <= MAX_LEAD_S:
            raise ValueError('Lead time must be {:.0f}-{:.0f} s'.format(MIN_LEAD_S, MAX_LEAD_S))
        valid_hold(hold_s)
        action = ACTIONS[name]
        params = [float(value) for value in action.prepare(self.drone, [float(value) for value in params])]
        action.validate(self.drone, params)
        self._wait_for_peer()
        self._check_idle()
        self._check_held()
        plan = Plan()
        plan.plan_id = self.status[0] + 1
        plan.action, plan.params, plan.hold_s = name, params, float(hold_s)
        start = time.time() + lead_s
        self._check_hold(start)
        plan.start_time = rospy.Time.from_sec(start)
        self._set(plan.plan_id, 'proposed')
        rospy.loginfo('Proposed plan %d: %s %s at %.3f', plan.plan_id, name, params, start)
        while True:
            if time.time() > start - COMMIT_MARGIN_S:
                self._fail(plan.plan_id, 'cancelled', 'no acceptance {:.1f} s before start'.format(COMMIT_MARGIN_S))
            peer = self._peer_status()
            if peer is not None and peer.plan_id == plan.plan_id:
                if peer.state == 'accepted':
                    break
                if peer.state in FAILED_STATES:
                    self._fail(plan.plan_id, 'cancelled', 'follower {}: {}'.format(peer.state, peer.reason))
            plan.header.stamp = rospy.Time.from_sec(time.time())
            self.plan_pub.publish(plan)
            self._hold(STEP_S)
        self._execute(plan.plan_id, action, params, start, plan.hold_s)

    def finish(self):
        """Tell the follower the routine has ended."""
        self._set(self.status[0], 'finished')


class Follower(Handshake):
    def __init__(self, drone):
        Handshake.__init__(self, drone, FOLLOWER)
        self.plan = None
        rospy.Subscriber('/pidrone/coordination/peer_plan', Plan, self._plan_received, queue_size=1)

    def _plan_received(self, msg):
        self.plan = msg

    def run(self):
        """Execute the leader's plans until it reports the routine finished."""
        try:
            self._run()
        except Exception as error:
            self._report_failure(error)
            raise

    def _run(self):
        self._wait_for_peer()
        while True:
            self._check_idle()
            self._check_hold()
            leader = self._peer_status()
            if leader is not None and leader.state == 'finished':
                rospy.loginfo('Leader finished the routine')
                return
            plan = self.plan
            if plan is not None and plan.plan_id > self.status[0]:
                self._accept(plan)
            else:
                self._hold(STEP_S)

    def _accept(self, plan):
        plan_id, start = plan.plan_id, plan.start_time.to_sec()
        try:
            action = ACTIONS.get(plan.action)
            if action is None:
                raise ValueError('unknown action {}'.format(plan.action))
            params = list(plan.params)
            lead = start - time.time()
            if not ACCEPT_MARGIN_S <= lead <= MAX_LEAD_S:
                raise ValueError('start is {:.2f} s away'.format(lead))
            valid_hold(plan.hold_s)
            self._check_hold(start)
            action.validate(self.drone, params)
            self._check_held()
        except (ValueError, RuntimeError) as error:
            self._fail(plan_id, 'rejected', str(error))
        self._set(plan_id, 'accepted')
        rospy.loginfo('Accepted plan %d: %s %s at %.3f', plan_id, plan.action, params, start)
        self._execute(plan_id, action, params, start, plan.hold_s)
