"""Hardware-free plan handshake with a fake clock and mocked ROS transport."""
import os
import runpy
import sys
import unittest
from types import SimpleNamespace as NS
from unittest import mock

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'coordination.py')


class Stamp(object):
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class HandshakeTest(unittest.TestCase):
    def setUp(self):
        self.now, self.ticks = 1000.0, []
        self.published, self.callbacks = {}, {}
        rospy = NS(
            Publisher=lambda topic, kind, queue_size: NS(
                publish=lambda msg: self.published.setdefault(topic, []).append(NS(**vars(msg)))),
            Subscriber=lambda topic, kind, callback, queue_size: self.callbacks.__setitem__(topic, callback),
            Timer=lambda period, callback: None, Duration=lambda seconds: seconds,
            Time=NS(from_sec=Stamp), loginfo=lambda *a: None, logwarn=lambda *a: None)
        msgs = NS(Plan=lambda: NS(header=NS(stamp=None), plan_id=0, action='', params=[], start_time=None),
                  PlanStatus=lambda: NS(header=NS(stamp=None), drone_id='', plan_id=0, state='', reason=''))
        with mock.patch.dict(sys.modules, {'rospy': rospy, 'pidrone_pkg': NS(), 'pidrone_pkg.msg': msgs}):
            self.module = runpy.run_path(SCRIPT)
        self.module['Handshake']._hold.__globals__['time'] = NS(time=lambda: self.now, sleep=self.sleep)
        self.drone = NS(mode='DISARMED', xy_anchor=None, hover=self.sleep,
                        _check_sensors=lambda: None, _check_flight=lambda operation: None)
        self.started = []
        test = self

        class Wait(self.module['Action']):
            name = 'wait'

            def duration(self, params):
                return params[0]

            def run(self, drone, start, end, params, check):
                test.started.append(test.now)
                while test.now < end:
                    check()
                    test.sleep(min(.05, end - test.now))
        self.module['register'](Wait)

    def sleep(self, seconds):
        self.now += seconds
        for tick in list(self.ticks):
            tick()

    def handshake(self, role):
        uri = 'http://%s.local:11311' % role
        with mock.patch.dict(os.environ, {'ROS_MASTER_URI': uri}):
            return self.module['Leader' if role == 'drone1' else 'Follower'](self.drone)

    def peer_says(self, handshake, script):
        """Each tick, publish the peer state that script(now) returns (None: silent)."""
        def tick():
            state = script(self.now)
            if state is not None:
                handshake._peer_received(NS(drone_id=handshake.peer_id, plan_id=state[0],
                                            state=state[1], reason=''))
        self.ticks.append(tick)
        tick()

    def proposed(self):
        return '/pidrone/coordination/plan' in self.published

    def states(self, handshake):
        return [msg.state for msg in self.published['/pidrone/coordination/plan_status']]

    def test_leader_runs_accepted_plan_at_the_proposed_time(self):
        leader = self.handshake('drone1')
        start = self.now + 3
        self.peer_says(leader, lambda now: (0, 'idle') if not self.proposed() else
                       (1, 'accepted' if now < start else 'executing'))
        leader.run('wait', [2], 3, 10)
        plan = self.published['/pidrone/coordination/plan'][0]
        self.assertEqual((plan.plan_id, plan.action, plan.params, plan.hold_s), (1, 'wait', [2.], 10.))
        self.assertAlmostEqual(plan.start_time.to_sec(), start)
        self.assertAlmostEqual(self.started[0], start)
        self.assertAlmostEqual(self.now, start + 2)
        self.assertEqual(self.states(leader)[-3:], ['proposed', 'executing', 'done'])

    def test_leader_cancels_on_late_acceptance_or_rejection(self):
        for reply in ('late', 'rejected'):
            with self.subTest(reply=reply):
                self.setUp()
                leader = self.handshake('drone1')
                start = self.now + 3
                self.peer_says(leader, lambda now: (0, 'idle') if not self.proposed() else
                               (1, 'rejected') if reply == 'rejected' else
                               (1, 'accepted') if now >= start - .4 else (0, 'idle'))
                with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                    leader.run('wait', [2], 3, 10)
                self.assertEqual(self.states(leader)[-1], 'cancelled')
                self.assertEqual(self.started, [])

    def test_follower_accepts_with_margin_runs_at_start_and_stops_when_leader_finishes(self):
        follower = self.handshake('drone2')
        start = self.now + 5
        self.peer_says(follower, lambda now: (1, 'proposed') if now < start else
                       (1, 'executing') if now < start + 2 else (1, 'finished'))
        follower._plan_received(NS(plan_id=1, action='wait', params=[2], hold_s=10, start_time=Stamp(start)))
        follower.run()
        self.assertAlmostEqual(self.started[0], start)
        self.assertEqual(self.states(follower)[:3], ['accepted', 'executing', 'done'])

    def test_follower_rejects_short_notice_unknown_action_or_unheld_drone(self):
        for case in ('short', 'unknown', 'unlocked'):
            with self.subTest(case=case):
                self.setUp()
                follower = self.handshake('drone2')
                self.peer_says(follower, lambda now: (1, 'proposed'))
                if case == 'unlocked':
                    self.drone.mode = 'FLYING'
                follower._plan_received(NS(plan_id=1, action='fly' if case == 'unknown' else 'wait',
                                           params=[2], hold_s=10, start_time=Stamp(self.now + (.9 if case == 'short' else 5))))
                with self.assertRaisesRegex(RuntimeError, 'rejected'):
                    follower.run()
                self.assertEqual(self.states(follower)[-1], 'rejected')
                self.assertEqual(self.started, [])

    def test_follower_fails_when_leader_never_starts_and_on_stale_leader_mid_action(self):
        for case in ('never_started', 'stale'):
            with self.subTest(case=case):
                self.setUp()
                follower = self.handshake('drone2')
                start = self.now + 5
                self.peer_says(follower, lambda now: (1, 'proposed') if now < start or case == 'never_started'
                               else (1, 'executing') if now < start + 1 else None)
                follower._plan_received(NS(plan_id=1, action='wait', params=[3], hold_s=10, start_time=Stamp(start)))
                with self.assertRaisesRegex(RuntimeError, 'failed'):
                    follower.run()
                self.assertEqual(self.states(follower)[-1], 'failed')
                limit = start + .5 if case == 'never_started' else start + 1.5
                self.assertLessEqual(self.now, limit + .06)

    def test_takeoff_plan_adds_a_settle_pad_and_rejects_unsafe_starts(self):
        takeoff = self.module['ACTIONS']['takeoff']
        stale = mock.Mock(side_effect=RuntimeError('Fused position is missing/stale'))
        drone = NS(mode='DISARMED', ground_height=.05, takeoff_rate=.1, voltage=16., ARM_SECONDS=1.,
                   ALTITUDE_SETTLE_SECONDS=1., MIN_START_V=15.2, _check_battery=lambda: None,
                   _hold_pose=lambda local=False: None)
        params = takeoff.prepare(drone, [.5])
        self.assertAlmostEqual(params[1], 1 + 4.5 + 3)
        takeoff.validate(drone, params)
        self.assertEqual(takeoff.duration(params), params[1])
        for change, bad, reason in [
                ({}, [1.2, 20], 'height'), ({}, [.5, 6], 'needs'), ({'mode': 'FLYING'}, params, 'disarmed'),
                ({'voltage': 15.}, params, 'battery'), ({'_hold_pose': stale}, params, 'stale')]:
            with self.subTest(reason=reason):
                with self.assertRaisesRegex((ValueError, RuntimeError), reason):
                    takeoff.validate(NS(**dict(vars(drone), **change)), bad)

    def test_follower_lands_when_no_plan_starts_within_the_hold(self):
        follower = self.handshake('drone2')
        start = self.now + 5
        self.peer_says(follower, lambda now: (1, 'proposed') if now < start else (1, 'executing')
                       if now < start + 2 else (2, 'proposed'))
        follower._plan_received(NS(plan_id=1, action='wait', params=[2], hold_s=3, start_time=Stamp(start)))
        with mock.patch.object(follower, '_accept', wraps=follower._accept) as accept:
            def late_plan():
                # A next plan that would start after the hold is rejected.
                if self.now >= start + 2.5 and accept.call_count == 1:
                    follower._plan_received(NS(plan_id=2, action='wait', params=[2], hold_s=3,
                                               start_time=Stamp(start + 5.5 + 1)))
            self.ticks.append(late_plan)
            with self.assertRaisesRegex(RuntimeError, 'after the hold ends'):
                follower.run()
        self.assertEqual(self.states(follower)[-1], 'rejected')
        self.ticks.pop()
        follower.status, follower.plan = (1, 'done', ''), None
        with self.assertRaisesRegex(RuntimeError, 'within the hold'):
            follower.run()
        self.assertAlmostEqual(self.now, start + 5, delta=.06)


if __name__ == '__main__':
    unittest.main()
