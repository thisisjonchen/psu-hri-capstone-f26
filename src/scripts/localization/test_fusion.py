"""Behavioral checks for fusion, coordinate axes and controller integration."""
import math
import os
import runpy
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest import mock

import numpy as np

from localization.fusion import (RoomOffset, VelocityFilter, VerticalFilter, body_angle, horizontal_acceleration,
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

    def test_flow_and_biased_imu_track_velocity_and_learn_the_bias(self):
        random = np.random.RandomState(1)
        f, bias = VelocityFilter(), np.array((.17, -.1))  # About a 1 degree tilt error.
        velocity, position, errors = np.zeros(2), np.zeros(2), []
        for i in range(1, 60*20):
            t = 10 + i/60.0
            acceleration = np.array((.3*math.sin(t), .2*math.cos(1.3*t)))
            velocity += acceleration/60.0
            position += velocity/60.0
            self.assertTrue(f.predict(acceleration + bias + random.normal(0, .05, 2), t))
            if i % 2 == 0:  # 30 Hz flow, measured 50 ms before receipt.
                measured = velocity - acceleration*.05 + random.normal(0, .1, 2)
                accepted, reason = f.correct(measured, .15**2, t - .05)
                self.assertTrue(accepted or t < 10.1, reason)
            if t > 15:
                errors.append(np.linalg.norm(f.velocity - velocity))
        # Integrating this IMU alone would be about 4 m/s wrong after 20 s.
        self.assertLess(np.sqrt(np.mean(np.square(errors))), .08)
        np.testing.assert_allclose(f.state[2:], bias, atol=.04)
        self.assertLess(np.linalg.norm(f.odom - position), .3)
        self.assertGreater(np.linalg.eigvalsh(f.covariance).min(), 0)

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

    def test_uwb_corrects_only_the_room_offset_and_filters_its_noise(self):
        random = np.random.RandomState(3)
        room, errors = RoomOffset(yaw_offset=.3), []
        origin = np.array((1., 2.))
        for i in range(200):
            t, odom = 10 + i*.05, np.array((.01*i, 0.))
            truth = origin + rotate(odom, .3)
            self.assertTrue(room.update(truth + random.normal(0, .1, 2), np.eye(2)*.15**2, t, odom)[0])
            if i > 40:
                errors.append(room.position(odom) - truth)
        self.assertLess(np.std(errors), .05)  # From 0.10 m raw UWB noise.
        offset = room.offset.copy()
        for position, covariance, stamp in [((20, 20), np.eye(2)*.0225, t + .05),
                                            (truth, np.zeros((2, 2)), t + .05), (truth, np.eye(2), t)]:
            self.assertFalse(room.update(position, covariance, stamp, odom)[0])
            np.testing.assert_array_equal(room.offset, offset)
        self.assertFalse(room.update(truth, np.eye(2), t + .1, None)[0])
        self.assertTrue(room.ready(t + 1.5))
        self.assertFalse(room.ready(t + 1.51))
        self.assertGreater(room.covariance(t + 1)[0, 0], room.covariance(t)[0, 0])

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

    def test_yaw_correction_keeps_the_current_room_position(self):
        room = RoomOffset(yaw_offset=.3)
        odom = np.array((.4, -.2))
        room.update((1, 2), np.eye(2)*.01, 10, odom)
        before = room.position(odom)
        self.assertTrue(room.correct_yaw(.2, odom))
        self.assertAlmostEqual(room.yaw_offset, .5)
        np.testing.assert_allclose(room.position(odom), before)
        # Later motion is rotated by the corrected heading.
        np.testing.assert_allclose(room.position(odom + (1, 0)) - before, rotate((1, 0), .5))
        self.assertFalse(RoomOffset().correct_yaw(.1, odom))


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
        sys.modules['pidrone_pkg.msg'].TagCorners = NS
        sys.modules['sensor_msgs.msg'].CameraInfo = NS
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

    def refresh(self, yaw=0, acceleration=(0, 0, 0), omega=(0, 0, 0), acceleration_valid=True, height=.5):
        self.node.range_callback(NS(range=height, min_range=.05, max_range=4, header=NS(stamp=Stamp(self.ros_now))))
        q = quaternion(yaw=yaw)
        self.node.imu_callback(NS(orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]),
            angular_velocity=NS(**dict(zip(('x', 'y', 'z'), omega))),
            linear_acceleration=NS(**dict(zip(('x', 'y', 'z'), acceleration))),
            linear_acceleration_covariance=[0 if acceleration_valid else -1]*9,
            orientation_covariance=[0]*9, header=NS(stamp=Stamp(self.ros_now))))

    def tick(self, seconds=1/60.0, **kwargs):
        self.now += seconds
        self.ros_now += seconds
        self.refresh(**kwargs)

    def tag_reference(self):
        from localization.apriltag_geometry import camera_rotation
        self.node.tag_rotation = camera_rotation(90)
        self.node.camera_callback(NS(header=NS(frame_id='camera'), width=320, height=240,
            K=[300, 0, 160, 0, 300, 120, 0, 0, 1], D=[0]*5, distortion_model='plumb_bob'))
        return NS(header=NS(frame_id='camera', stamp=Stamp(self.ros_now)), tag_id=5,
                  image_width=320, image_height=240,
                  corner_x=[130, 190, 190, 130], corner_y=[90, 90, 150, 150])

    def test_tag5_corrects_in_all_modes_without_jumps_or_losing_hold_and_freezes_on_loss(self):
        controller = self.controller()
        controller.room_target = (1.2, 2.1, .5)
        generation, offset = self.node.generation, self.node.room.yaw_offset
        reference = self.tag_reference()
        for mode in ('DISARMED', 'ARMED', 'FLYING'):
            for i in range(6):
                self.ros_now += .1
                self.refresh(yaw=.2)  # IMU drift; tag orientation remains unchanged.
                self.node.mode_callback(NS(mode=mode))
                self.flow(0, 0)
                self.uwb()
                self.node.step()
                reference.header.stamp = Stamp(self.ros_now)
                before = self.node.room.yaw_offset
                odom = self.node.velocity.odom.copy()
                room = self.node.room.position(odom)
                self.node.tag_callback(reference)
                delta = self.node.room.yaw_offset - before
                self.assertLessEqual(abs(delta), math.radians(.2)+1e-12)
                if i >= 2:
                    self.assertLess(delta, 0)
                np.testing.assert_allclose(self.node.room.position(odom), room, atol=1e-12)
                np.testing.assert_array_equal(self.node.velocity.odom, odom)
                self.assertEqual(self.node.generation, generation)
                after = self.node.room.yaw_offset
                self.node.tag_callback(reference)
                self.assertEqual(self.node.room.yaw_offset, after, 'cached frame must not correct twice')
                self.node.step()
                controller.current_state_callback(self.messages['pidrone/state'][-1])
                controller.room_pose_callback(self.messages['pidrone/localization/fused'][-1])
                controller.room_generation_callback(NS(data=self.node.generation))
                self.assertEqual(controller.room_target, (1.2, 2.1, .5))
        self.assertLess(self.node.room.yaw_offset, offset)
        offset = self.node.room.yaw_offset
        for _ in range(8):
            self.ros_now += .1
            self.refresh(yaw=.3)
            self.flow(0, 0)
            self.uwb()
            self.node.step()
        self.node.tag_callback(reference)  # Delayed stale observation.
        self.assertEqual(self.node.room.yaw_offset, offset)
        self.assertEqual(self.params['~room_yaw_offset'], offset)

    def test_tag_yaw_rejects_bad_observations_and_matches_historical_imu_across_wrap(self):
        reference = self.tag_reference()
        offset = self.node.room.yaw_offset
        # Every invalid condition is repeated across enough frames to calibrate.
        for field, bad in [('tag_id', 4), ('image_width', 240), ('corner_x', [0]*3),
                           ('corner_x', [float('nan')]*4), ('corner_y', [-1]*4)]:
            original = getattr(reference, field)
            setattr(reference, field, bad)
            for _ in range(3):
                self.ros_now += .02
                self.refresh()
                reference.header.stamp = Stamp(self.ros_now)
                self.node.tag_callback(reference)
            self.assertEqual(self.node.room.yaw_offset, offset)
            setattr(reference, field, original)
        for focal_length, model in ((0, 'plumb_bob'), (float('nan'), 'plumb_bob'), (300, 'unknown')):
            self.node.camera_callback(NS(header=NS(frame_id='camera'), width=320, height=240,
                K=[focal_length, 0, 160, 0, 300, 120, 0, 0, 1], D=[0]*5, distortion_model=model))
            for _ in range(3):
                self.ros_now += .02
                self.refresh()
                reference.header.stamp = Stamp(self.ros_now)
                self.node.tag_callback(reference)
            self.assertEqual(self.node.room.yaw_offset, offset)
        self.tag_reference()  # Restore valid calibration.
        for invalid_stamp in (0, self.ros_now+1, self.ros_now-1):
            reference.header.stamp = Stamp(invalid_stamp)
            self.node.tag_callback(reference)
        self.assertEqual(self.node.room.yaw_offset, offset)
        self.ros_now += .1  # No matching IMU, though the last IMU is still fresh.
        reference.header.stamp = Stamp(self.ros_now)
        self.node.tag_callback(reference)
        self.assertEqual(self.node.room.yaw_offset, offset)
        # Observations straddle +/-pi, and detection arrives after a newer turn.
        self.node.room.yaw_offset = math.pi-.03
        for bias in (-.01, .01, -.01):
            self.ros_now += .1
            self.refresh(yaw=-math.pi/2+bias)
            reference.header.stamp = Stamp(self.ros_now)
            self.ros_now += .03
            self.refresh(yaw=-math.pi/2+.4)
            self.node.tag_callback(reference)
        self.assertGreater(self.node.room.yaw_offset, math.pi-.03)
        self.assertLess(self.node.room.yaw_offset, math.pi)
        self.assertEqual(len(self.node.tag_samples), 3)
        self.assertAlmostEqual(abs(self.node.tag_samples[-1][1]), math.pi-.01)

    def test_ground_zero_velocity_and_flight_flow_work_without_uwb(self):
        self.node = type(self.node)()  # No anchors or UWB at all.
        self.node.mode_callback(NS(mode='DISARMED'))
        for _ in range(30):
            self.tick()
            self.flow(.5, .5)  # The near-floor camera is ignored on the ground.
            self.node.step()
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        velocity = self.messages['pidrone/state'][-1].twist_with_covariance.twist.linear
        self.assertAlmostEqual(velocity.x, 0, places=3)
        self.assertAlmostEqual(velocity.y, 0, places=3)
        self.assertTrue(self.messages['pidrone/localization/odometry'])
        self.assertEqual(self.messages['pidrone/localization/fused'], [])
        self.node.mode_callback(NS(mode='FLYING'))
        for _ in range(30):
            self.tick(yaw=math.pi/2)
            self.flow(.1, .2)
            self.node.step()
        velocity = self.messages['pidrone/state'][-1].twist_with_covariance.twist.linear
        self.assertAlmostEqual(velocity.x, .1, delta=.01)
        self.assertAlmostEqual(velocity.y, .2, delta=.01)
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertEqual(self.messages['pidrone/localization/fused'], [])
        values = {v.key: v.value for v in self.messages['pidrone/localization/fusion_status'][-1].status[0].values}
        self.assertEqual(values['velocity_source'], 'IMU + optical flow')
        for _ in range(36):  # 0.6 s without flow: coast, then expire.
            self.tick(yaw=math.pi/2)
            self.node.step()
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        velocity = self.messages['pidrone/state'][-1].twist_with_covariance.twist.linear
        self.assertEqual((velocity.x, velocity.y), (0, 0))
        self.assertEqual(self.messages['pidrone/state'][-1].pose_with_covariance.pose.position.z, .5)

    def test_startup_heading_waits_for_disarmed_mode_and_fresh_imu(self):
        self.node.room.yaw_offset = None
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
                self.assertIsNone(self.node.room.yaw_offset)
                # Velocity needs no room heading, but the room pose does.
                self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)

    def test_startup_heading_aligns_once_and_preserves_position_height_and_turns(self):
        self.node = type(self.node)()
        self.assertIsNone(self.node.room.yaw_offset)
        self.node.mode_callback(NS(mode='DISARMED'))
        self.refresh(yaw=math.radians(64))
        self.report_anchors()
        self.uwb()
        self.node.step()
        self.assertAlmostEqual(self.node.room.yaw_offset, math.radians(26))
        self.assertEqual(self.params['~room_yaw_offset'], self.node.room.yaw_offset)
        pose = self.messages['pidrone/localization/fused'][-1].pose.pose
        self.assertEqual((pose.position.x, pose.position.y, pose.position.z), (1, 2, .5))
        self.assertAlmostEqual(pose.orientation.z, 0)
        self.assertAlmostEqual(pose.orientation.w, 1)
        np.testing.assert_array_equal(self.node.velocity.odom, (0, 0))
        # Turning, disarming and losing UWB must not recapture heading.
        for mode in ('FLYING', 'DISARMED'):
            self.ros_now += .05
            self.refresh(yaw=math.radians(74))
            self.node.mode_callback(NS(mode=mode))
            self.node.invalidate_room()
            self.uwb()
            self.node.step()
            self.assertAlmostEqual(self.node.room.yaw_offset, math.radians(26))
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

    def flow(self, x=.1, y=.2, stamp=None, frame='pidrone_body'):
        msg = NS(header=NS(frame_id=frame, stamp=Stamp(self.ros_now if stamp is None else stamp)),
                 twist=twist())
        msg.twist.linear.x, msg.twist.linear.y = x, y
        self.node.flow_callback(msg)

    def test_flow_compensation_reaches_controller_and_camera_gaps_expire_velocity(self):
        self.node.mode_callback(NS(mode='FLYING'))
        for _ in range(30):
            self.tick(yaw=math.pi/2, omega=(.2, -.4, 0))
            # At 0.5 m, the 0.2 angular factor removes (+.02,-.04) m/s.
            self.flow(.12, .16)
            self.uwb()
            self.node.step()
        velocity = self.messages['pidrone/state'][-1].twist_with_covariance.twist.linear
        self.assertAlmostEqual(velocity.x, .1, delta=.01)
        self.assertAlmostEqual(velocity.y, .2, delta=.01)
        self.assertEqual(self.node.velocity_reason, 'OK')
        controller = self.controller()
        output = controller.step()
        self.assertLess(output[0], 1500)
        self.assertLess(output[1], 1500)
        accepted_stamp = self.node.velocity.corrected_stamp
        for x, y, stamp, frame in [(.1, .2, self.ros_now-1, 'pidrone_body'),
                (.1, .2, self.ros_now+.1, 'pidrone_body'), (.1, .2, self.ros_now, 'map'),
                (float('nan'), 0, self.ros_now, 'pidrone_body'), (3, 0, self.ros_now, 'pidrone_body'),
                (.12, .16, self.ros_now, 'pidrone_body')]:
            self.flow(x, y, stamp, frame)
            self.assertEqual(self.node.velocity.corrected_stamp, accepted_stamp)
        # A camera sample can be fresh yet too far from the matching attitude.
        self.ros_now += .25
        self.refresh()
        self.flow(stamp=self.ros_now-.2)
        self.assertEqual(self.node.velocity.corrected_stamp, accepted_stamp)
        for _ in range(4):
            self.ros_now += .1
            self.refresh()
            self.uwb()
            self.node.step()
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        velocity = self.messages['pidrone/state'][-1].twist_with_covariance.twist.linear
        self.assertEqual((velocity.x, velocity.y), (0, 0))

    def test_climb_rate_reaches_controller_and_throttle_uses_climb_and_tilt(self):
        for i in range(60):  # ToF rises at 0.1 m/s; IMU reports no acceleration.
            self.tick(height=.5 + .1*i/60.0)
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

    def test_uwb_never_changes_velocity_and_room_pose_survives_a_1_5_s_gap(self):
        generation = self.node.generation
        self.node.mode_callback(NS(mode='FLYING'))
        for _ in range(20):
            self.tick()
            self.flow(0, 0)
            self.uwb()
            self.node.step()
        self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
        self.tick()
        velocity = self.node.velocity.velocity.copy()
        self.uwb(x=1.3)  # A 0.3 m UWB jump.
        self.node.step()
        np.testing.assert_array_equal(self.node.velocity.velocity, velocity)
        x = self.messages['pidrone/localization/fused'][-1].pose.pose.position.x
        self.assertGreater(x, 1)
        self.assertLess(x, 1.3)  # Filtered, not copied.
        controller = self.controller()
        last_fix = self.node.room.stamp
        while self.ros_now - last_fix < 1.45:
            self.tick()
            self.flow(0, 0)
            self.node.step()
            self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
        controller.current_state_callback(self.messages['pidrone/state'][-1])
        controller.room_pose_callback(self.messages['pidrone/localization/fused'][-1])
        self.assertTrue(controller.room_ready())
        self.assertFalse(controller.room_ready(allow_prediction=False))
        for _ in range(6):
            self.tick()
            self.flow(0, 0)
            self.node.step()
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        # New UWB restores the same frame without a generation change.
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
        self.assertIn('pidrone/picamera/twist', topics)
        generation = self.node.generation
        count = len(self.messages['pidrone/state'])
        self.node.height.range = None
        self.flow()  # Camera data cannot substitute for missing ToF height.
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertEqual(len(self.messages['pidrone/state']), count)
        self.assertIsNotNone(self.node.room.offset)
        self.assertEqual(self.node.generation, generation)
        self.refresh()
        self.node.step()
        self.assertEqual(len(self.messages['pidrone/state']), count + 1)

    def test_uwb_grace_does_not_extend_imu_or_height_freshness(self):
        for missing in ('imu', 'height'):
            with self.subTest(missing=missing):
                self.node = type(self.node)()
                self.node.mode_callback(NS(mode='DISARMED'))
                self.refresh()
                self.report_anchors()
                self.uwb()
                self.node.step()
                controller = self.controller()
                for _ in range(7):
                    old_imu = self.node.imu
                    old_attitude = self.node.height.attitude
                    old_range = self.node.height.range
                    self.ros_now += .1
                    self.now += .1
                    self.refresh()
                    if missing == 'imu':
                        self.node.imu, self.node.height.attitude = old_imu, old_attitude
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
        self.assertIsNone(self.node.room.offset)
        self.refresh()
        self.node.step()
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.ros_now = 50
        self.node.step()
        self.assertIsNone(self.node.velocity.stamp)
        self.assertIsNone(self.node.height.range)

    def test_partial_anchor_reports_keep_surveyed_geometry(self):
        report = self.report_anchors()
        report.anchor_ids.pop()
        report.positions.pop()
        self.node.anchors_callback(report)
        self.node.step()
        self.assertTrue(self.messages['pidrone/localization/ready'][-1].data)
        self.assertIsNotNone(self.node.room.offset)

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
        self.assertAlmostEqual(controller.desired_velocity.x, .04)
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
            controller.room_pose.header.stamp = Stamp(self.ros_now)
            controller.current_state_callback(current)
            controller.desired_twist_callback(request)
            controller.step()
            self.ros_now += 1 / 60.0
        # Default tuning must not accumulate trim, even after sustained error.
        self.assertEqual(controller.pid.roll_low._i, 0)
        self.assertEqual(controller.pid.pitch_low._i, 0)
        velocity.x, velocity.y = .12, -.12
        current.header.stamp = Stamp(self.ros_now)
        controller.current_state_callback(current)
        controller.desired_twist_callback(request)
        output = controller.step()
        self.assertLess(output[0], 1500, 'brake when rightward velocity exceeds target')
        self.assertGreater(output[1], 1500, 'brake when backward velocity exceeds target')
        self.assertGreater(output[0], 1495, 'a 0.02 m/s error needs a modest correction')
        self.assertLess(output[1], 1505, 'a 0.02 m/s error needs a modest correction')

    def test_xy_output_limits(self):
        output = self.controller().pid.step(NS(x=100., y=-100., z=0.))
        self.assertLessEqual(output[0], 1530)
        self.assertGreaterEqual(output[1], 1470)


if __name__ == '__main__':
    unittest.main()
