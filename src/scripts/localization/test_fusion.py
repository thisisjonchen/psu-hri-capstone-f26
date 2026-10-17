"""Behavioral checks for fusion, coordinate axes and controller integration."""
import math
import os
import runpy
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest import mock

import numpy as np

from localization.fusion import PlanarFusion, body_angle, rotate, target_velocity
from localization import test_ros_node as ros_doubles
from localization.test_ros_node import Pose, Stamp, Layout


def quaternion(roll=0, pitch=0, yaw=0):
    cr, sr, cp, sp, cy, sy = (math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2),
                             math.sin(pitch/2), math.cos(yaw/2), math.sin(yaw/2))
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def euler(q):
    x, y, z, w = q
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1, min(1, 2*(w*y-z*x)))), math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def vector():
    return NS(x=0., y=0., z=0.)


def twist():
    return NS(linear=vector(), angular=vector())


def state():
    return NS(header=NS(frame_id='', stamp=Stamp(0)), pose_with_covariance=Pose().pose,
              twist_with_covariance=NS(twist=twist(), covariance=[0.] * 36))


class FusionMathTest(unittest.TestCase):
    def test_controller_right_and_forward_axes_at_two_headings(self):
        np.testing.assert_allclose(rotate((0, 1), body_angle(0)), (1, 0), atol=1e-12)
        np.testing.assert_allclose(rotate((1, 0), body_angle(0)), (0, -1), atol=1e-12)
        np.testing.assert_allclose(rotate((0, 1), body_angle(math.pi/2)), (0, 1), atol=1e-12)
        v = target_velocity((4, 0), (0, 0), body_angle(0), 0.5, 0.2)
        np.testing.assert_allclose(v, (0, 0.2), atol=1e-12)
        for yaw, forward, right in ((0, (0, 1), (1, 0)), (math.pi/2, (-1, 0), (0, 1))):
            angle = body_angle(yaw) + math.pi/2
            np.testing.assert_allclose(rotate((0, 1), angle), forward, atol=1e-12)
            np.testing.assert_allclose(rotate((1, 0), angle), right, atol=1e-12)

    def test_delayed_uwb_corrects_room_without_jumping_local_odometry(self):
        f = PlanarFusion(yaw_offset=math.pi/2)
        f.predict(10)
        for time in (10.1, 10.2):
            f.predict(time)
        self.assertTrue(f.update_uwb((2, 3), np.eye(2)*0.0225, 10.1)[0])
        np.testing.assert_allclose(f.room, (2, 3), atol=1e-12)
        odom = f.odom.copy()
        before = f.room.copy()
        self.assertTrue(f.update_uwb((2.1, 3.2), np.eye(2)*0.0225, 10.2)[0])
        np.testing.assert_array_equal(f.odom, odom)
        self.assertTrue(before[0] < f.room[0] < 2.1)
        self.assertGreater(np.linalg.eigvalsh(f.covariance).min(), 0)

    def test_outliers_invalid_covariance_and_duplicate_fixes_cannot_renew_solution(self):
        f = PlanarFusion(yaw_offset=0)
        f.predict(10)
        f.update_uwb((1, 2), np.eye(2)*0.0225, 10)
        f.predict(10.1)
        before_velocity, before_covariance = f.velocity.copy(), f.covariance.copy()
        for position, noise, stamp in [((20, 20), np.eye(2)*.0225, 10.1),
                ((1, 2), np.zeros((2, 2)), 10.1), ((1, 2), np.eye(2), 10)]:
            self.assertFalse(f.update_uwb(position, noise, stamp)[0])
            np.testing.assert_array_equal(f.velocity, before_velocity)
            np.testing.assert_array_equal(f.covariance, before_covariance)
        self.assertEqual(f.uwb_stamp, 10)
        self.assertFalse(f.predict(10.05))

    def test_estimator_clock_gap_requires_new_room_fix_without_integrating_missing_motion(self):
        f = PlanarFusion(yaw_offset=0)
        f.predict(10)
        f.update_uwb((1, 2), np.eye(2), 10)
        f.predict(11)
        self.assertIsNone(f.room)
        np.testing.assert_array_equal(f.odom, (0, 0))
        self.assertFalse(f.update_uwb((1, 2), np.eye(2), 10.9)[0])

    def test_uwb_estimates_velocity_and_braking_with_delayed_fixes(self):
        room_speed = np.array((.2, -.1))
        for delay in (0, .1):
            with self.subTest(delay=delay):
                f = PlanarFusion(yaw_offset=.7)
                for i in range(301):
                    t = i / 60.0
                    f.predict(10+t)
                    if i % 6 == 0 and t >= delay:
                        acquired = t-delay
                        before = f.odom.copy()
                        accepted, reason = f.update_uwb(room_speed * min(acquired, 3),
                                                       np.eye(2)*.0225, 10+acquired)
                        self.assertTrue(accepted, reason)
                        np.testing.assert_array_equal(f.odom, before)
                    if i == 125:  # Velocity persists between UWB updates.
                        np.testing.assert_allclose(rotate(f.velocity, .7), room_speed, atol=.03)
                        learned = f.velocity.copy()
                        accepted, reason = f.update_uwb((20, 20), np.eye(2)*.0225, f.stamp)
                        self.assertFalse(accepted)
                        self.assertEqual(reason, 'UWB innovation rejected')
                        np.testing.assert_array_equal(f.velocity, learned)
                    if i == 300:
                        self.assertLess(np.linalg.norm(f.velocity), .025)
                        np.testing.assert_allclose(f.room, room_speed*3, atol=.025)
                self.assertGreater(np.linalg.eigvalsh(f.covariance).min(), 0)
                f.reset_room()
                np.testing.assert_array_equal(f.room_velocity, (0, 0))
                np.testing.assert_array_equal(f.velocity, (0, 0))

    def test_stationary_uwb_noise_is_filtered_instead_of_differentiated(self):
        f = PlanarFusion(yaw_offset=0)
        random = np.random.RandomState(7)
        fused, raw, previous = [], [], None
        for i in range(601):
            f.predict(10+i/60.0)
            if i % 6 == 0:
                position = random.normal(0, .05, 2)
                self.assertTrue(f.update_uwb(position, np.eye(2)*.0225, 10+i/60.0)[0])
                if i > 120:
                    fused.append(np.linalg.norm(f.velocity))
                    raw.append(np.linalg.norm((position-previous)/.1))
                previous = position
        self.assertLess(np.sqrt(np.mean(np.square(fused))), .08)
        self.assertLess(np.mean(fused), np.mean(raw)*.2)


class FusionNodeTest(unittest.TestCase):
    def test_legacy_calibration_lock_does_not_disable_fused_room_position(self):
        self.params['/pidrone/localization/calibration_lock'] = True
        self.node = type(self.node)()
        self.assertIsNone(self.node.filter.yaw_offset)
        self.node.mode_callback(NS(mode='DISARMED'))
        self.refresh()
        self.report_anchors()
        self.uwb()
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
        q = self.messages['pidrone/localization/fused'][-1].pose.pose.orientation
        np.testing.assert_allclose((q.x, q.y, q.z, q.w), (0, 0, 0, 1), atol=1e-12)

    def setUp(self):
        # Reuse the existing transport doubles; execute the real node/controller.
        ros_doubles.NodeTest.setUp(self)
        self.ros.get_time = lambda: self.ros_now
        self.ros.loginfo = self.ros.logwarn = mock.Mock()
        self.ros.loginfo_throttle = mock.Mock()
        self.ros.init_node = mock.Mock()
        self.ros.set_param = self.params.__setitem__
        tf = ModuleType('tf')
        tf.transformations = NS(euler_from_quaternion=euler, quaternion_from_euler=quaternion,
                                quaternion_multiply=multiply)
        tf.TransformBroadcaster = lambda: NS(sendTransform=mock.Mock())
        sys.modules['tf'] = tf
        self.addCleanup(lambda: sys.modules.pop('tf', None))
        for name in ('std_msgs', 'nav_msgs'):
            sys.modules[name] = ModuleType(name)
            sys.modules[name + '.msg'] = ModuleType(name + '.msg')
            self.addCleanup(lambda name=name: (sys.modules.pop(name, None), sys.modules.pop(name+'.msg', None)))
        for name in ('Bool', 'UInt32', 'Float32', 'Empty'):
            setattr(sys.modules['std_msgs.msg'], name, NS)
        sys.modules['nav_msgs.msg'].Odometry = lambda: NS(header=NS(), child_frame_id='')
        sys.modules['pidrone_pkg.msg'].State = state
        sys.modules['pidrone_pkg.msg'].Mode = sys.modules['pidrone_pkg.msg'].RC = NS
        sys.modules['geometry_msgs.msg'].TwistStamped = NS
        sys.modules['geometry_msgs.msg'].PoseStamped = NS
        sys.modules['geometry_msgs.msg'].Pose = sys.modules['geometry_msgs.msg'].Twist = NS
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'fusion_localization_node')
        self.node = runpy.run_path(script)['FusionLocalizationNode']()
        self.node.step.__globals__['monotonic'] = lambda: self.now
        self.node.mode_callback(NS(mode='DISARMED'))
        self.refresh()
        self.report_anchors()
        self.uwb()
        self.node.step()

    def refresh(self, yaw=0):
        self.node.range_callback(NS(range=.5, min_range=.05, max_range=4, header=NS(stamp=Stamp(self.ros_now))))
        q = quaternion(yaw=yaw)
        self.node.imu_callback(NS(orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]), angular_velocity=vector(),
            orientation_covariance=[0]*9, header=NS(stamp=Stamp(self.ros_now))))

    def test_startup_heading_waits_for_disarmed_mode_and_fresh_imu(self):
        self.node.filter.yaw_offset = None
        self.node.invalidate_room()
        for mode, mode_at, imu in [
                (None, None, self.node.imu),
                ('ARMED', self.ros_now, self.node.imu),
                ('FLYING', self.ros_now, self.node.imu),
                ('DISARMED', self.ros_now - 1, self.node.imu),
                ('DISARMED', self.ros_now, None),
                ('DISARMED', self.ros_now, (self.ros_now - 1,) + self.node.imu[1:])]:
            with self.subTest(mode=mode, mode_at=mode_at, imu=imu):
                self.node.mode, self.node.mode_at, self.node.imu = mode, mode_at, imu
                self.node.step()
                self.assertIsNone(self.node.filter.yaw_offset)
                self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)

    def test_startup_heading_aligns_once_and_preserves_position_height_and_turns(self):
        self.node = type(self.node)()
        self.assertIsNone(self.node.filter.yaw_offset)
        self.refresh(yaw=math.radians(64))
        self.node.mode_callback(NS(mode='DISARMED'))
        self.report_anchors()
        self.uwb()
        self.node.step()
        self.assertAlmostEqual(self.node.filter.yaw_offset, math.radians(26))
        self.assertEqual(self.params['~room_yaw_offset'], self.node.filter.yaw_offset)
        pose = self.messages['pidrone/localization/fused'][-1].pose.pose
        self.assertEqual((pose.position.x, pose.position.y, pose.position.z), (1, 2, .5))
        self.assertAlmostEqual(pose.orientation.z, 0)
        self.assertAlmostEqual(pose.orientation.w, 1)
        np.testing.assert_array_equal(self.node.filter.odom, (0, 0))
        # Turning, disarming and losing UWB must not recapture heading.
        for mode in ('FLYING', 'DISARMED'):
            self.ros_now += .05
            self.refresh(yaw=math.radians(74))
            self.node.mode_callback(NS(mode=mode))
            self.node.invalidate_room()
            self.uwb()
            self.node.step()
            self.assertAlmostEqual(self.node.filter.yaw_offset, math.radians(26))
            q = self.messages['pidrone/localization/fused'][-1].pose.pose.orientation
            self.assertAlmostEqual(q.z, math.sin(math.radians(5)))

    def uwb(self, x=1, y=2):
        msg = Pose()
        msg.header.frame_id, msg.header.stamp = 'map', Stamp(self.ros_now)
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, 999
        msg.pose.covariance[0] = msg.pose.covariance[7] = .0225
        self.node.uwb_callback(msg)

    def report_anchors(self, shift=0):
        msg = Layout()
        msg.header.frame_id, msg.header.stamp = 'map', Stamp(self.ros_now)
        msg.anchor_ids = ['0001', '0002', '0003', '0004']
        msg.positions = [NS(x=x+shift, y=y, z=2) for x, y in [(0, 0), (3, 0), (3, 6), (0, 6)]]
        self.node.anchors_callback(msg)
        return msg

    def test_tof_remains_z_and_room_corrections_leave_controller_pose_unchanged(self):
        state_msg = self.messages['pidrone/state'][-1]
        fused = self.messages['pidrone/localization/fused'][-1]
        self.assertEqual(state_msg.header.frame_id, 'odom')
        self.assertEqual(state_msg.pose_with_covariance.pose.position.z, .5)
        self.assertEqual(fused.pose.pose.position.z, .5)
        self.assertEqual(fused.pose.pose.position.x, 1)
        self.assertEqual(state_msg.pose_with_covariance.pose.position.x, 0)
        self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
        self.assertEqual(self.node.broadcaster.sendTransform.call_count, 2)
        self.node.height.floor_z = .7
        self.node.step()
        self.assertAlmostEqual(self.messages['pidrone/state'][-1].pose_with_covariance.pose.position.z, .5)
        self.assertAlmostEqual(self.messages['pidrone/localization/fused'][-1].pose.pose.position.z, 1.2)
        self.assertEqual(self.node.broadcaster.sendTransform.call_args[0][0][2], .7)

    def test_missing_heading_or_live_anchors_prevents_room_output(self):
        self.node.mode_callback(NS(mode='FLYING'))
        self.node.filter.yaw_offset = None
        self.node.invalidate_room()
        self.node.step()
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertIn('heading', self.messages['pidrone/localization/fusion_status'][-1].status[0].message)

    def test_uwb_velocity_reaches_controller_and_dropout_expires_once(self):
        generation = self.node.generation
        start = self.ros_now
        # About 7 Hz, with a 0.45 s gap: prediction must bridge missing fixes.
        for i in range(1, 181):
            self.now += 1/60.0
            self.ros_now = start + i/60.0
            self.refresh()
            if i % 9 == 0 and i not in (90, 99):
                self.uwb(x=1 + .1*i/60.0, y=2 + .2*i/60.0)
            self.node.step()
            self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
            self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
            self.assertEqual(self.messages['pidrone/localization/fused'][-1].header.stamp.to_sec(),
                             self.node.filter.uwb_stamp)
        controller = self.controller()
        controller.desired_twist_callback(twist())
        command = controller.step()
        self.assertAlmostEqual(controller.current_velocity.x, .1, delta=.03)
        self.assertAlmostEqual(controller.current_velocity.y, .2, delta=.03)
        self.assertLess(command[0], 1495)
        self.assertLess(command[1], 1490)
        covariance = self.messages['pidrone/state'][-1].twist_with_covariance.covariance
        self.assertGreater(covariance[0], self.node.filter.velocity_variance)
        self.assertEqual(self.node.generation, generation)
        for i in range(36):
            self.now += 1/60.0
            self.ros_now += 1/60.0
            self.refresh()
            self.node.step()
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertEqual(self.node.generation, generation)
        state_msg = self.messages['pidrone/state'][-1]
        self.assertEqual(state_msg.pose_with_covariance.pose.position.z, .5)
        self.assertEqual(state_msg.twist_with_covariance.twist.linear.y, 0)
        controller.current_state_callback(state_msg)
        controller.local_ready_callback(self.messages['pidrone/localization/local_ready'][-1])
        controller.desired_velocity.y = .2
        self.assertEqual(controller.step()[:2], [1500, 1500])
        # New UWB restores the same frame without a mode or generation change.
        self.uwb(x=1.3, y=2.6)
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
        self.assertEqual(self.node.generation, generation)

    def test_uwb_initializes_without_camera_and_height_loss_does_not_reset_xy(self):
        self.node = type(self.node)()
        self.node.mode_callback(NS(mode='DISARMED'))
        self.refresh()
        self.report_anchors()
        self.uwb()
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
        topics = [call.args[0] for call in self.ros.Subscriber.call_args_list]
        self.assertNotIn('pidrone/picamera/twist', topics)
        generation = self.node.generation
        count = len(self.messages['pidrone/state'])
        self.node.height.range = None
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertEqual(len(self.messages['pidrone/state']), count)
        self.assertIsNotNone(self.node.filter.room)
        self.assertEqual(self.node.generation, generation)
        self.refresh()
        self.node.step()
        self.assertEqual(len(self.messages['pidrone/state']), count + 1)

    def test_pending_uwb_at_timeout_preserves_learned_velocity(self):
        start = self.ros_now
        for i in range(1, 121):
            self.ros_now = start + i / 60.0
            self.now += 1 / 60.0
            self.refresh()
            if i % 6 == 0:
                self.uwb(x=1 + .1 * i / 60.0, y=2)
            self.node.step()
        previous = self.node.filter.velocity.copy()
        self.assertGreater(np.linalg.norm(previous), .08)
        for accepted in (True, False):
            last_fix = self.ros_now
            for i in range(1, 30):
                self.ros_now = last_fix + i / 60.0
                self.now += 1 / 60.0
                self.refresh()
                self.node.step()
            # A measurement is already queued on the first tick past expiry.
            self.now += .501 - (self.ros_now - last_fix)
            self.ros_now = last_fix + .501
            self.refresh()
            x, y = (self.node.filter.room + self.node.filter.room_velocity *
                    (self.ros_now - self.node.filter.stamp))
            self.uwb(x=x if accepted else x + 20, y=y)
            self.node.step()
            self.assertEqual(self.messages['pidrone/localization/ready'][-1].data, accepted)
            if accepted:
                # Exactly matches prediction: no velocity change needed.
                np.testing.assert_allclose(self.node.filter.velocity, previous, atol=1e-10)
            else:
                self.assertIsNone(self.node.filter.room)
                np.testing.assert_array_equal(self.node.filter.velocity, (0, 0))

    def test_layout_change_and_clock_reset_invalidate_xy(self):
        generation = self.node.generation
        self.ros_now += .1
        self.report_anchors(shift=.1)
        self.assertGreater(self.node.generation, generation)
        self.assertIsNone(self.node.filter.room)
        self.refresh()
        self.node.step()
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.ros_now = 50
        self.node.step()
        self.assertIsNone(self.node.filter.stamp)
        self.assertIsNone(self.node.height.range)

    def test_uwb_expiry_stops_room_output_but_partial_reports_keep_surveyed_geometry(self):
        for _ in range(6):
            self.now += .1
            self.ros_now += .1
            self.refresh()
            self.report_anchors()
            self.node.step()
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        before = len(self.messages['pidrone/localization/fused'])
        self.uwb()
        self.node.step()
        self.assertEqual(len(self.messages['pidrone/localization/fused']), before + 1)
        report = self.report_anchors()
        report.anchor_ids.pop()
        report.positions.pop()
        self.node.anchors_callback(report)
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
        self.assertIsNotNone(self.node.filter.room)

    def controller(self):
        script = os.path.join(os.path.dirname(__file__), '..', 'pid_controller.py')
        controller = runpy.run_path(script)['PIDController']()
        controller.position_control_pub = NS(publish=mock.Mock())
        controller.local_ready_callback(NS(data=True))
        controller.room_ready_callback(NS(data=True))
        controller.current_mode = 'FLYING'
        controller.current_state_callback(self.messages['pidrone/state'][-1])
        controller.room_pose_callback(self.messages['pidrone/localization/fused'][-1])
        controller.room_anchors_callback(self.report_anchors())
        return controller

    def test_room_navigation_pauses_on_sensor_loss_and_cancels_on_manual_input(self):
        controller = self.controller()
        target = NS(header=NS(frame_id='map', stamp=Stamp(self.ros_now)),
                    pose=NS(position=NS(x=2, y=2, z=999)))
        controller.room_target_callback(target)
        controller.calc_error()
        self.assertAlmostEqual(controller.desired_velocity.x, .1)
        self.assertAlmostEqual(controller.desired_velocity.y, 0, places=10)
        self.assertEqual(controller.desired_position.z, .25)
        controller.room_ready_callback(NS(data=False))
        controller.calc_error()
        np.testing.assert_array_equal(controller.room_target, (2, 2))
        self.assertEqual(controller.desired_velocity.planar_magnitude(), 0)
        self.assertTrue(controller.position_control)
        controller.room_ready_callback(NS(data=True))
        controller.calc_error()
        self.assertAlmostEqual(controller.desired_velocity.x, .1)
        # Missing readiness heartbeats do not defeat a still-current estimate.
        self.ros_now += .35
        controller.calc_error()
        self.assertAlmostEqual(controller.desired_velocity.x, .1)
        self.ros_now += .2
        controller.calc_error()
        self.assertEqual((controller.pid_error.x, controller.pid_error.y), (0, 0))
        np.testing.assert_array_equal(controller.room_target, (2, 2))
        controller.desired_twist_callback(twist())
        self.assertIsNone(controller.room_target)
        controller.local_ready_callback(NS(data=False))
        controller.calc_error()
        self.assertTrue(controller.position_control)
        self.assertEqual((controller.pid_error.x, controller.pid_error.y), (0, 0))

    def test_room_commands_reject_wrong_frame_outside_outline_and_disarmed_mode(self):
        controller = self.controller()
        target = NS(header=NS(frame_id='odom', stamp=Stamp(self.ros_now)), pose=NS(position=NS(x=2, y=2)))
        controller.room_target_callback(target)
        self.assertIsNone(controller.room_target)
        target.header.frame_id = 'map'
        target.pose.position.x = 20
        controller.room_target_callback(target)
        self.assertIsNone(controller.room_target)
        target.pose.position.x = 2
        controller.current_mode = 'DISARMED'
        controller.room_target_callback(target)
        self.assertIsNone(controller.room_target)

    def test_local_hold_keeps_velocity_damping_when_altitude_changes(self):
        controller = self.controller()
        current = state()
        current.header.stamp = Stamp(self.ros_now)
        current.pose_with_covariance.pose.orientation.w = 1
        current.pose_with_covariance.pose.position.z = .5
        controller.current_state_callback(current)
        controller.position_control_callback(NS(data=True))
        altitude = NS(position=NS(x=0, y=0, z=.1))
        controller.desired_pose_callback(altitude)
        current.pose_with_covariance.pose.position.y = .1
        current.twist_with_covariance.twist.linear.y = .2
        current.twist_with_covariance.twist.linear.z = .03
        controller.current_state_callback(current)
        command = controller.step()
        self.assertLess(controller.pid_error.y, -20)
        self.assertLess(command[1], 1480)
        self.assertAlmostEqual(controller.pid_error.z, -15)

    def test_manual_velocity_expires_before_output_and_reset_clears_yaw(self):
        controller = self.controller()
        command = twist()
        command.linear.x, command.angular.z = .5, 20
        controller.desired_twist_callback(command)
        # No earlier PID cycle: expiry must start at command receipt.
        self.ros_now += .3
        output = controller.step()
        self.assertEqual(output[:3], [1500, 1500, 1500])
        command.linear.x, command.linear.z = 0, .1
        controller.desired_twist_callback(command)  # No division by zero for vertical-only input.
        controller.reset()
        self.assertEqual(controller.desired_yaw_velocity, 0)
        self.assertIsNone(controller.velocity_deadline)
        self.assertIsNone(controller.yaw_deadline)

    def test_xy_brakes_after_sustained_velocity_correction(self):
        controller = self.controller()
        request = twist()
        request.linear.x, request.linear.y = .1, -.1
        current = controller.current_state
        velocity = current.twist_with_covariance.twist.linear
        velocity.x = velocity.y = 0
        # Eight seconds of correction must not overpower subsequent braking.
        for _ in range(481):
            current.header.stamp = Stamp(self.ros_now)
            controller.current_state_callback(current)
            controller.desired_twist_callback(request)
            controller.step()
            self.ros_now += 1 / 60.0
        # Sustained error should learn useful trim within a short hover, but the
        # trim must stay small enough for velocity feedback to reverse the output.
        self.assertGreater(controller.pid.roll_low._i, 5)
        self.assertLessEqual(controller.pid.roll_low._i, 10)
        self.assertLess(controller.pid.pitch_low._i, -5)
        self.assertGreaterEqual(controller.pid.pitch_low._i, -10)
        velocity.x, velocity.y = .2, -.2
        current.header.stamp = Stamp(self.ros_now)
        controller.current_state_callback(current)
        controller.desired_twist_callback(request)
        output = controller.step()
        self.assertLess(output[0], 1500, 'brake when rightward velocity exceeds target')
        self.assertGreater(output[1], 1500, 'brake when backward velocity exceeds target')
        self.assertGreater(output[0], 1480, 'a 0.10 m/s error needs a modest correction')
        self.assertLess(output[1], 1520, 'a 0.10 m/s error needs a modest correction')

    def test_xy_trim_cannot_exceed_output_limits_or_wind_up_at_saturation(self):
        pid = self.controller().pid
        pid.roll_low._i, pid.pitch_low._i = 5., -5.
        error = NS(x=100., y=-100., z=0.)
        for _ in range(61):
            output = pid.step(error)
            self.assertLessEqual(output[0], 1530)
            self.assertGreaterEqual(output[1], 1470)
            self.ros_now += 1 / 60.0
        self.assertEqual(pid.roll_low._i, 5.)
        self.assertEqual(pid.pitch_low._i, -5.)
        # Existing trim unwinds when the velocity error reverses.
        pid.roll_low._i, pid.pitch_low._i = 10., -10.
        output = pid.step(NS(x=-1., y=1., z=0.))
        self.assertLessEqual(output[0], pid.roll.control_range[1])
        self.assertGreaterEqual(output[1], pid.pitch.control_range[0])
        self.assertLess(pid.roll_low._i, 10.)
        self.assertGreater(pid.pitch_low._i, -10.)


if __name__ == '__main__':
    unittest.main()
