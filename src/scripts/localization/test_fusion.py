"""Behavioral checks for fusion, coordinate axes and controller integration."""
import math
import os
import runpy
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest import mock

import numpy as np

from localization.fusion import (VelocityFilter, VerticalFilter, body_angle, horizontal_acceleration,
                                 rotate, target_velocity, vertical_acceleration)
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
    def test_acceleration_axes_include_tilt_without_subtracting_gravity_twice(self):
        np.testing.assert_allclose(horizontal_acceleration((1, 0, 0), (0, 0, 0)), (0, -1), atol=1e-12)
        np.testing.assert_allclose(horizontal_acceleration((0, 1, 0), (0, 0, math.pi/2)), (0, 1), atol=1e-12)
        tilt = math.pi/6
        # A world-horizontal acceleration expressed in a pitched body.
        np.testing.assert_allclose(horizontal_acceleration((0, math.cos(tilt), math.sin(tilt)),
                                   (0, tilt, 0)), (1, 0), atol=1e-12)
        # A world-vertical acceleration must not leak into planar velocity.
        np.testing.assert_allclose(horizontal_acceleration((0, -math.sin(tilt), math.cos(tilt)),
                                   (0, tilt, 0)), (0, 0), atol=1e-12)
        np.testing.assert_allclose(horizontal_acceleration((-math.sin(tilt), 0, math.cos(tilt)),
                                   (tilt, 0, 0)), (0, 0), atol=1e-12)

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

    def fly(self, seconds=20, bias=(.17, -.1), delay=.05, gust_at=None, imu_clip=None,
            flow_gap=None, seed=1):
        """Independent horizontal flight: truth, a biased noisy IMU and delayed noisy 30 Hz flow."""
        random = np.random.RandomState(seed)
        f, bias = VelocityFilter(), np.array(bias)
        velocity, position, errors, history = np.zeros(2), np.zeros(2), [], []
        for i in range(1, int(60*seconds)):
            t = 10 + i/60.0
            acceleration = np.array((.3*math.sin(t), .2*math.cos(1.3*t)))
            if gust_at is not None and gust_at <= t < gust_at + .1:
                acceleration = acceleration + (8, 0)  # +0.8 m/s in 0.1 s.
            velocity += acceleration/60.0
            position += velocity/60.0
            history.append((t, velocity.copy()))
            measured_acceleration = acceleration + bias + random.normal(0, .05, 2)
            if imu_clip is not None:
                measured_acceleration = np.clip(measured_acceleration, -imu_clip, imu_clip)
            f.predict(measured_acceleration, t)
            dropped = flow_gap is not None and (t % 1.0) < flow_gap
            if i % 2 == 0 and t - delay >= history[0][0] and not dropped:
                past = [v for stamp, v in history if stamp <= t - delay][-1]
                f.correct(past + random.normal(0, .1, 2), .15**2, t - delay)
            errors.append((t, np.linalg.norm(f.velocity - velocity), f.ready(t)))
        return f, errors, position

    def test_velocity_tracks_truth_through_bias_delay_dropouts_and_gusts(self):
        # Integrating a 1 degree tilt error alone would be about 4 m/s wrong after 20 s.
        for name, kwargs in [('1 deg tilt error', {}), ('3 deg tilt error', dict(bias=(.5, -.3))),
                             ('100 ms camera latency', dict(delay=.1)),
                             ('0.3 s flow gaps every second', dict(flow_gap=.3))]:
            with self.subTest(name):
                f, errors, position = self.fly(**kwargs)
                settled = [e for t, e, _ in errors if t > 15]
                self.assertLess(np.sqrt(np.mean(np.square(settled))), .1)
                self.assertTrue(all(ready for t, _, ready in errors if t > 11), 'X/Y dropped out')
                self.assertLess(np.linalg.norm(f.odom - position), .3)
        for name, clip in [('gust seen by IMU', None), ('gust clipped by IMU', 2.)]:
            with self.subTest(name):
                _, errors, _ = self.fly(seconds=15, gust_at=12., imu_clip=clip)
                after = [e for t, e, _ in errors if 12.4 <= t <= 13]
                self.assertLess(max(after), .15, 'velocity still %.2f m/s wrong after a gust' % max(after))

    def test_delayed_flow_is_compared_with_velocity_at_its_own_stamp(self):
        f = VelocityFilter()
        f.predict((0, 0), 10)
        f.correct((0, 0), .0001, 10, gated=False)
        for i in range(1, 7):
            f.predict((1, 0), 10 + i*.05)  # Accelerating at 1 m/s^2.
        velocity = f.velocity.copy()
        # At 10.1 the true velocity was 0.1; a correct delayed sample changes nothing.
        self.assertTrue(f.correct((.1, 0), .01, 10.1)[0])
        np.testing.assert_allclose(f.velocity, velocity, atol=1e-9)
        odom = f.odom.copy()
        f.correct((.3, 0), .01, 10.2)
        np.testing.assert_array_equal(f.odom, odom, 'corrections never move odometry')

    def test_imu_bridges_a_short_flow_gap_then_velocity_expires_but_bias_is_kept(self):
        f = VelocityFilter(timeout=.5)
        for i in range(1, 121):  # Two seconds on the ground learn part of the bias.
            t = 10 + i/60.0
            f.predict((.1, 0), t)
            f.correct((0, 0), .02**2, t, gated=False)
        self.assertTrue(f.ready(t))
        self.assertGreater(f.state[2], .02)
        bias, start = f.state[2:].copy(), t
        while t < start + .45:
            t += 1/60.0
            f.predict((.1, 0), t)
        self.assertTrue(f.ready(t))
        odom = f.odom.copy()
        t = start + .55
        f.predict((.1, 0), t)
        self.assertFalse(f.ready(t))
        np.testing.assert_array_equal(f.velocity, (0, 0))
        np.testing.assert_array_equal(f.state[2:], bias)
        f.predict((.1, 0), t + .01)
        np.testing.assert_array_equal(f.odom, odom, 'unknown motion is not integrated')
        self.assertEqual(f.correct((.2, 0), .01, t + .01), (True, 'Velocity initialized'))
        np.testing.assert_allclose(f.velocity, (.2, 0))

    def test_flow_outliers_are_rejected_until_persistent_disagreement_resets_velocity(self):
        f = VelocityFilter(max_rejections=5)
        f.predict((0, 0), 10)
        f.correct((0, 0), .01, 10)
        for i in range(1, 5):
            f.predict((0, 0), 10 + i*.03)
            self.assertEqual(f.correct((1.5, 0), .01, 10 + i*.03), (False, 'Optical-flow innovation rejected'))
            np.testing.assert_array_equal(f.velocity, (0, 0))
        f.predict((0, 0), 10.15)
        f.correct((.05, 0), .01, 10.15)  # One consistent sample clears the count.
        for i in range(6, 11):
            f.predict((0, 0), 10 + i*.03)
            accepted, reason = f.correct((1.5, 0), .01, 10 + i*.03)
        self.assertEqual((accepted, reason), (True, 'Velocity reset to measurement'))
        np.testing.assert_allclose(f.velocity, (1.5, 0))

    def test_invalid_repeated_and_gap_inputs_cannot_renew_velocity(self):
        f = VelocityFilter(max_gap=.5)
        self.assertEqual(f.correct((0, 0), .01, 10)[1], 'Velocity timestamp is outside IMU history')
        f.predict((0, 0), 10)
        f.correct((0, 0), .01, 10)
        for velocity, variance, stamp in [((float('nan'), 0), .01, 10.1), ((0, 0), 0, 10.1),
                                          ((0, 0), .01, 10), ((0, 0), .01, 9)]:
            self.assertFalse(f.correct(velocity, variance, stamp)[0])
        self.assertFalse(f.predict((0, 0), 9.9))
        self.assertFalse(f.predict((float('inf'), 0), 10.1))
        self.assertTrue(f.predict(None, 10.1))  # Missing acceleration holds velocity.
        self.assertTrue(f.predict((0, 0), 11))   # An IMU gap stops the estimate.
        self.assertFalse(f.initialized)
        for kwargs in (dict(acceleration_stddev=0), dict(timeout=float('nan')), dict(max_rejections=0)):
            with self.assertRaises(ValueError):
                VelocityFilter(**kwargs)

    def test_vertical_acceleration_is_world_up_at_any_tilt(self):
        tilt = math.pi/6
        self.assertAlmostEqual(vertical_acceleration((0, 0, 1), (0, 0, 2)), 1)
        self.assertAlmostEqual(vertical_acceleration((0, -math.sin(tilt), math.cos(tilt)), (0, tilt, 0)), 1)
        self.assertAlmostEqual(vertical_acceleration((0, math.cos(tilt), math.sin(tilt)), (0, tilt, 0)), 0)
        self.assertAlmostEqual(vertical_acceleration((-math.sin(tilt), 0, math.cos(tilt)), (tilt, 0, 0)), 1)

    def test_vertical_filter_tracks_climb_rate_from_biased_imu_and_delayed_tof(self):
        random = np.random.RandomState(5)
        f, bias, errors, raw = VerticalFilter(), .3, [], []
        height, climb, previous = .25, 0., None
        for i in range(1, 60*15):
            t = 10 + i/60.0
            acceleration = .4*math.sin(1.5*t)
            climb += acceleration/60.0
            height += climb/60.0
            self.assertTrue(f.predict(acceleration + bias + random.normal(0, .1), t))
            if i % 6 == 0:  # 10 Hz ToF, measured 50 ms before arrival.
                measured = height - climb*.05 + random.normal(0, .02)
                f.correct(measured, .02**2, t - .05)
                if previous is not None and t > 15:
                    raw.append((measured - previous)/.1 - climb)
                previous = measured
            if t > 15:
                errors.append(f.climb_rate - climb)
        rms = np.sqrt(np.mean(np.square(errors)))
        self.assertLess(rms, .08)
        self.assertLess(rms, .3*np.sqrt(np.mean(np.square(raw))), 'much cleaner than differentiating ToF')
        self.assertAlmostEqual(f.state[2], bias, delta=.1)
        self.assertFalse(f.correct(float('nan'), .01, t))
        self.assertTrue(f.predict(None, t + .6))  # A gap stops the estimate.
        self.assertEqual(f.climb_rate, 0)

class FusionNodeTest(unittest.TestCase):
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
        sys.modules['pidrone_pkg.msg'].FlightReadiness = lambda: NS(
            header=NS(stamp=Stamp(0), frame_id=''), mode='', paired=False, ready=False, reason='')
        sys.modules['pidrone_pkg.msg'].Mode = sys.modules['pidrone_pkg.msg'].RC = NS
        sys.modules['geometry_msgs.msg'].TwistStamped = NS
        sys.modules['geometry_msgs.msg'].PoseStamped = NS
        sys.modules['geometry_msgs.msg'].Pose = sys.modules['geometry_msgs.msg'].Twist = NS
        script = os.path.join(os.path.dirname(__file__), '..', 'PALS', 'fusion_localization_node')
        self.ros.Subscriber.reset_mock()
        self.node = runpy.run_path(script)['FusionLocalizationNode']()
        self.node.step.__globals__['monotonic'] = lambda: self.now
        self.refresh()
        self.report_anchors()
        self.node.step()

    def refresh(self, yaw=math.pi/2, height=.5, velocity=(0, 0, 0), omega=(0, 0, 0),
                position=(1, 2), roll=0, pitch=0, ready=True):
        self.node.range_callback(NS(range=height, min_range=.01, max_range=4,
                                   header=NS(stamp=Stamp(self.ros_now))))
        msg = self.ekf_message(yaw, height, velocity, omega, position, roll, pitch)
        self.node.update_ekf(msg)
        self.node.ekf_ready_callback(NS(data=ready))

    def ekf_message(self, yaw=math.pi/2, height=.5, velocity=(0, 0, 0), omega=(0, 0, 0),
                    position=(1, 2), roll=0, pitch=0):
        msg = state()
        msg.header = NS(frame_id='map', stamp=Stamp(self.ros_now))
        msg.child_frame_id = 'pidrone_body'
        msg.pose, msg.twist = msg.pose_with_covariance, msg.twist_with_covariance
        q = multiply(quaternion(roll, pitch, yaw), quaternion(yaw=-math.pi/2))
        msg.pose.pose.orientation = NS(**dict(zip('xyzw', q)))
        msg.pose.pose.position = NS(x=position[0], y=position[1], z=height)
        msg.twist.twist.linear = NS(**dict(zip('xyz', velocity)))
        msg.twist.twist.angular = NS(**dict(zip('xyz', omega)))
        return msg


    def tick(self, seconds=1/60.0, **kwargs):
        self.now += seconds
        self.ros_now += seconds
        self.refresh(**kwargs)

    def test_ekf_motion_and_readiness_without_anchors(self):
        self.node = type(self.node)()
        self.tick(height=.05)
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertEqual(self.messages['pidrone/localization/fused'], [])
        self.tick(yaw=math.pi/2, velocity=(.1, .2, .03))
        self.node.step()
        msg = self.messages['pidrone/state'][-1]
        self.assertEqual(msg.header.frame_id, 'map')
        self.assertAlmostEqual(msg.twist_with_covariance.twist.linear.x, .1)
        self.assertAlmostEqual(msg.twist_with_covariance.twist.linear.y, .2)
        self.assertAlmostEqual(msg.twist_with_covariance.twist.linear.z, .03)
        self.assertAlmostEqual(msg.pose_with_covariance.pose.position.x, 1, delta=.005)
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.tick(ready=False, position=(.4, .5))
        self.node.step()
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertEqual(self.messages['pidrone/state'][-1].pose_with_covariance.pose.position.x, .4)
        count = len(self.messages['pidrone/state'])
        self.ros_now += .2
        self.node.step()
        self.assertEqual(len(self.messages['pidrone/state']), count)
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)


    def report_anchors(self, shift=0):
        msg = Layout()
        msg.header.frame_id, msg.header.stamp = 'map', Stamp(self.ros_now)
        msg.anchor_ids = ['0001', '0002', '0003', '0004']
        msg.positions = [NS(x=x+shift, y=y, z=2) for x, y in [(0, 0), (3, 0), (3, 6), (0, 6)]]
        self.node.anchors_callback(msg)
        return msg


    def test_ekf_body_velocity_and_covariance_are_leveled_for_the_controller(self):
        from localization.rotations import quaternion_matrix
        self.ros_now += .1
        msg = self.ekf_message(yaw=.7, roll=.2, pitch=-.1, velocity=(.1, .2, .3))
        msg.twist.covariance = np.diag([1., 2., 3., 4., 5., 6.]).ravel().tolist()
        self.node.range_callback(NS(range=.5, min_range=.01, max_range=4,
                                   header=NS(stamp=Stamp(self.ros_now-.05))))
        self.node.ekf_ready_callback(NS(data=True))
        self.node.ekf_callback(msg)
        q = msg.pose.pose.orientation
        angles = euler((q.x, q.y, q.z, q.w))
        level = quaternion_matrix(quaternion(yaw=-angles[2]))
        rotation = level.dot(quaternion_matrix((q.x, q.y, q.z, q.w)))
        expected = rotation.dot([.1, .2, .3])
        output = self.messages['pidrone/state'][-1]
        v = output.twist_with_covariance.twist.linear
        np.testing.assert_allclose([v.x, v.y, v.z], expected)
        transform = np.eye(6)
        transform[:3, :3] = rotation
        expected_cov = transform.dot(np.diag([1., 2., 3., 4., 5., 6.])).dot(transform.T)
        np.testing.assert_allclose(np.asarray(output.twist_with_covariance.covariance).reshape(6, 6), expected_cov)
        self.assertAlmostEqual(output.header.stamp.to_sec(), self.ros_now-.05)
        odometry = self.messages['pidrone/localization/odometry'][-1]
        self.assertEqual(odometry.twist, msg.twist)
        self.assertEqual(odometry.pose.pose.orientation, msg.pose.pose.orientation)
        self.assertEqual(odometry.pose.pose.position, output.pose_with_covariance.pose.position)
        self.assertEqual(odometry.header.stamp.to_sec(), msg.header.stamp.to_sec())
        count = len(self.messages['pidrone/state'])
        self.node.step()
        self.assertEqual(len(self.messages['pidrone/state']), count)


    def test_ekf_reaches_controller_and_invalid_outputs_do_not_renew_it(self):
        self.tick(yaw=math.pi/2, velocity=(.1, .2, 0))
        self.node.step()
        controller = self.controller()
        output = controller.step()
        self.assertLess(output[0], 1500)
        self.assertLess(output[1], 1500)
        accepted = self.node.ekf_odom
        self.ros_now += .05
        for change in ('stale', 'future', 'frame', 'body', 'nan', 'quaternion', 'covariance'):
            msg = self.ekf_message()
            if change == 'stale': msg.header.stamp = Stamp(self.ros_now-1)
            elif change == 'future': msg.header.stamp = Stamp(self.ros_now+.1)
            elif change == 'frame': msg.header.frame_id = 'ekf_odom'
            elif change == 'body': msg.child_frame_id = 'wrong_body'
            elif change == 'nan': msg.twist.twist.linear.x = float('nan')
            elif change == 'quaternion': msg.pose.pose.orientation.w = 2
            elif change == 'covariance': msg.twist.covariance[0] = float('nan')
            self.node.ekf_callback(msg)
            self.assertIs(self.node.ekf_odom, accepted)
        self.node.ekf_callback(accepted)
        self.assertIs(self.node.ekf_odom, accepted)
        self.node.ekf_ready_callback(NS(data=False))
        self.node.step()
        controller.local_ready_callback(self.messages['pidrone/localization/local_ready'][-1])
        self.assertFalse(controller.local_ready())
        self.assertEqual(controller.step()[:2], [1500, 1500])


    def test_climb_rate_reaches_controller_and_throttle_uses_climb_and_tilt(self):
        for i in range(60):  # ToF rises at 0.1 m/s; IMU reports no acceleration.
            self.tick(height=.5 + .1*i/60.0, velocity=(0, 0, .1))
            self.node.step()
        state_msg = self.messages['pidrone/state'][-1]
        self.assertAlmostEqual(state_msg.twist_with_covariance.twist.linear.z, .1, delta=.02)
        controller = self.controller()
        controller.desired_position.z = controller.current_position.z
        climbing = controller.step()[3]
        controller.current_velocity.z = 0
        level = controller.step()[3]
        self.assertLess(climbing, level, 'brake the measured climb')
        controller.current_state.pose_with_covariance.pose.orientation = NS(
            **dict(zip('xyzw', quaternion(roll=math.radians(20)))))
        controller.state_to_three_dim_vec_structs()
        controller.current_velocity.z = 0
        self.assertGreater(controller.step()[3], level, 'angle boost while leaning')

    def test_tof_remains_z_and_room_pose_matches_controller_pose(self):
        state_msg = self.messages['pidrone/state'][-1]
        fused = self.messages['pidrone/localization/fused'][-1]
        self.assertEqual(state_msg.header.frame_id, 'map')
        self.assertEqual(state_msg.pose_with_covariance.pose.position.z, .5)
        self.assertEqual(fused.pose.pose.position.z, .5)
        self.assertEqual(fused.pose.pose.position.x, state_msg.pose_with_covariance.pose.position.x)
        self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
        self.node.height.floor_z = .7
        self.tick()
        self.node.step()
        self.assertAlmostEqual(self.messages['pidrone/state'][-1].pose_with_covariance.pose.position.z, .5)
        self.assertAlmostEqual(self.messages['pidrone/localization/fused'][-1].pose.pose.position.z, 1.2)
        self.assertAlmostEqual(self.node.broadcaster.sendTransform.call_args[0][0][2], 1.2)

    def test_xy_corrections_are_rate_limited_but_motion_and_velocity_pass_through(self):
        speed = self.node.max_correction_speed
        self.tick(position=(1.3, 2))  # A 0.3 m EKF correction from UWB.
        self.node.step()
        x = self.messages['pidrone/state'][-1].pose_with_covariance.pose.position.x
        self.assertAlmostEqual(x, 1 + speed/60.0)
        self.assertGreater(self.messages['pidrone/localization/fused'][-1].pose.covariance[0], .25**2)
        for _ in range(59):
            self.tick(position=(1.3, 2))
            self.node.step()
        state_msg = self.messages['pidrone/state'][-1]
        self.assertAlmostEqual(state_msg.pose_with_covariance.pose.position.x, 1 + speed, places=6)
        self.assertEqual(self.messages['pidrone/localization/odometry'][-1].pose.pose.position.x,
                         state_msg.pose_with_covariance.pose.position.x)
        # Motion is followed at once; only the remaining gap closes at the limit.
        before = state_msg.pose_with_covariance.pose.position.x
        self.tick(yaw=0, position=(1.3 + .5/60.0, 2), velocity=(0, .5, 0))  # Forward is +X at yaw 0.
        self.node.step()
        state_msg = self.messages['pidrone/state'][-1]
        self.assertAlmostEqual(state_msg.pose_with_covariance.pose.position.x - before, (.5 + speed)/60.0)
        self.assertAlmostEqual(state_msg.twist_with_covariance.twist.linear.y, .5)
        # Losing readiness discards the gap rather than carrying it into the next flight.
        self.tick(ready=False, position=(1.3, 2))
        self.node.step()
        self.tick(position=(1.3, 2))
        self.node.step()
        self.assertEqual(self.messages['pidrone/state'][-1].pose_with_covariance.pose.position.x, 1.3)

    def test_ekf_keeps_local_origin_but_withholds_state_when_tof_is_missing(self):
        topics = [call.args[0] for call in self.ros.Subscriber.call_args_list]
        self.assertIn('pidrone/ekf/odometry', topics)
        self.assertNotIn('pidrone/picamera/twist', topics)
        self.assertNotIn('pidrone/imu', topics)
        generation = self.node.generation
        odometry = self.node.ekf_odom
        count = len(self.messages['pidrone/state'])
        self.node.height.range = None
        self.node.step()
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertEqual(len(self.messages['pidrone/state']), count)
        self.assertIs(self.node.ekf_odom, odometry)
        self.assertEqual(self.node.generation, generation)
        self.tick()
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertEqual(len(self.messages['pidrone/state']), count+1)


    def test_imu_and_height_freshness_are_required(self):
        for missing in ('imu', 'height'):
            with self.subTest(missing=missing):
                self.node = type(self.node)()
                self.refresh()
                self.report_anchors()
                self.node.step()
                controller = self.controller()
                for _ in range(7):
                    old_imu = self.node.attitude_stamp
                    old_attitude = self.node.height.attitude
                    old_range = self.node.height.range
                    self.ros_now += .1
                    self.now += .1
                    self.refresh()
                    if missing == 'imu':
                        self.node.attitude_stamp, self.node.height.attitude = old_imu, old_attitude
                    else:
                        self.node.height.range = old_range
                    self.node.step()
                    controller.current_state_callback(self.messages['pidrone/state'][-1])
                    controller.local_ready_callback(self.messages['pidrone/localization/local_ready'][-1])
                self.assertFalse(controller.local_ready())
                controller.desired_velocity.x = .1
                self.assertEqual(controller.step()[:2], [1500, 1500])

    def test_layout_change_and_clock_reset_invalidate_xy(self):
        generation = self.node.generation
        self.ros_now += .1
        self.report_anchors(shift=.1)
        self.assertGreater(self.node.generation, generation)
        self.assertTrue(self.node.layout_changed)
        self.refresh()
        self.node.step()
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.ros_now = 50
        self.node.step()
        self.assertTrue(self.node.ekf_reference_lost)
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertIsNone(self.node.height.range)

    def test_partial_anchor_reports_keep_surveyed_geometry(self):
        report = self.report_anchors()
        report.anchor_ids.pop()
        report.positions.pop()
        self.node.anchors_callback(report)
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)

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
                    pose=NS(position=NS(x=1.2, y=2, z=999)))
        controller.room_target_callback(target)
        controller.calc_error()
        self.assertAlmostEqual(controller.desired_velocity.x, .06)
        target.pose.position.x = 2
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
        self.assertLess(command[1], 1490)
        # 0.15 m above the 0.35 m target and climbing: the cascade reduces throttle.
        self.assertAlmostEqual(controller.altitude.target, .35)
        self.assertLess(command[3], controller.altitude.hover_throttle)

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

    def test_xy_learns_a_constant_push_from_local_flow_without_uwb(self):
        controller = self.controller()
        controller.room_ready_callback(NS(data=False))  # No UWB anchors.
        current = controller.current_state
        velocity = current.twist_with_covariance.twist.linear
        velocity.x, velocity.y = .08, -.08  # Steady drift right and backward, as seen in flight.
        for _ in range(300):
            current.header.stamp = Stamp(self.ros_now)
            controller.current_state_callback(current)
            controller.step()
            self.ros_now += 1 / 60.0
        self.assertLess(controller.velocity.integral[0], -.05, 'learn to push left')
        self.assertGreater(controller.velocity.integral[1], .05, 'learn to push forward')
        learned = list(controller.velocity.integral)
        # Estimate lost (e.g. below flow height while landing): keep cancelling the push
        # with the learned trim, but do not learn from stale data.
        controller.local_ready_callback(NS(data=False))
        for _ in range(60):
            current.header.stamp = Stamp(self.ros_now)
            controller.current_state_callback(current)
            output = controller.step()
            self.ros_now += 1 / 60.0
        self.assertEqual(controller.velocity.integral, learned)
        self.assertLess(output[0], 1500, 'still leaning left against the push')
        self.assertGreater(output[1], 1500, 'still leaning forward against the push')
        controller.current_mode_callback(NS(mode='ARMED'))  # Next flight starts fresh.
        self.assertEqual(controller.velocity.integral, [0.0, 0.0])

    def test_xy_output_limits(self):
        controller = self.controller()
        controller.current_state.twist_with_covariance.twist.linear.x = -10.
        controller.current_state.twist_with_covariance.twist.linear.y = 10.
        controller.current_state_callback(controller.current_state)
        roll, pitch = controller.step()[:2]
        limit = 500 * 5.0 / 60.0
        self.assertAlmostEqual(roll, 1500 + limit)
        self.assertAlmostEqual(pitch, 1500 - limit)

if __name__ == '__main__':
    unittest.main()
