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

    def test_delayed_uwb_corrects_room_without_jumping_local_odometry(self):
        f = PlanarFusion(yaw_offset=math.pi/2)
        f.update_flow((0, 1), 0, 10)
        for time in (10.1, 10.2):
            f.update_flow((0, 1), 0, time)
        self.assertTrue(f.update_uwb((2, 3), np.eye(2)*0.0225, 10.1)[0])
        np.testing.assert_allclose(f.room, (2, 3.1), atol=1e-12)
        odom = f.odom.copy()
        before = f.room.copy()
        self.assertTrue(f.update_uwb((2.1, 3.2), np.eye(2)*0.0225, 10.2)[0])
        np.testing.assert_array_equal(f.odom, odom)
        self.assertTrue(before[0] < f.room[0] < 2.1)
        self.assertGreater(np.linalg.eigvalsh(f.covariance).min(), 0)

    def test_outliers_invalid_covariance_and_duplicate_fixes_cannot_renew_solution(self):
        f = PlanarFusion(yaw_offset=0)
        f.update_flow((0, 0), 0, 10)
        f.update_uwb((1, 2), np.eye(2)*0.0225, 10)
        f.update_flow((0, 0), 0, 10.1)
        for position, noise, stamp in [((20, 20), np.eye(2)*.0225, 10.1),
                ((1, 2), np.zeros((2, 2)), 10.1), ((1, 2), np.eye(2), 10)]:
            self.assertFalse(f.update_uwb(position, noise, stamp)[0])
        self.assertEqual(f.uwb_stamp, 10)
        self.assertFalse(f.update_flow((0, 0), 0, 10.05))

    def test_camera_gap_requires_new_room_fix_and_does_not_integrate_missing_motion(self):
        f = PlanarFusion(yaw_offset=0)
        f.update_flow((0, 1), 0, 10)
        f.update_uwb((1, 2), np.eye(2), 10)
        f.update_flow((0, 1), 0, 11)
        self.assertIsNone(f.room)
        np.testing.assert_array_equal(f.odom, (0, 0))
        self.assertFalse(f.update_uwb((1, 2), np.eye(2), 10.9)[0])


class FusionNodeTest(unittest.TestCase):
    def setUp(self):
        # Reuse the existing transport doubles; execute the real node/controller.
        ros_doubles.NodeTest.setUp(self)
        self.params['~room_yaw_offset'] = '0'
        self.ros.get_time = lambda: self.ros_now
        self.ros.loginfo = self.ros.logwarn = mock.Mock()
        self.ros.loginfo_throttle = mock.Mock()
        self.ros.init_node = mock.Mock()
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
        self.refresh()
        self.report_anchors()
        self.flow()
        self.uwb()
        self.node.step()

    def refresh(self):
        self.node.range_callback(NS(range=.5, min_range=.05, max_range=4, header=NS(stamp=Stamp(self.ros_now))))
        self.node.imu_callback(NS(orientation=NS(x=0, y=0, z=0, w=1), angular_velocity=vector(),
            orientation_covariance=[0]*9, header=NS(stamp=Stamp(self.ros_now))))

    def flow(self, x=0, y=0, frame='pidrone_body'):
        msg = NS(header=NS(stamp=Stamp(self.ros_now), frame_id=frame), twist=twist())
        msg.twist.linear.x, msg.twist.linear.y = x, y
        self.node.flow_callback(msg)

    def uwb(self):
        msg = Pose()
        msg.header.frame_id, msg.header.stamp = 'map', Stamp(self.ros_now)
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 1, 2, 999
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
        self.node.filter.yaw_offset = None
        self.node.invalidate_room()
        self.node.step()
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertIn('heading', self.messages['pidrone/localization/fusion_status'][-1].status[0].message)

    def test_stale_camera_cancels_room_but_keeps_valid_altitude_control_state(self):
        before = len(self.messages['pidrone/localization/fused'])
        self.now += .3
        self.ros_now += .3
        self.refresh()
        self.node.step()
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.assertEqual(len(self.messages['pidrone/localization/fused']), before)
        self.assertEqual(self.messages['pidrone/state'][-1].pose_with_covariance.pose.position.z, .5)
        self.assertIsNone(self.node.filter.room)

    def test_layout_change_clock_reset_and_wrong_flow_frame_invalidate_room(self):
        generation = self.node.generation
        self.ros_now += .1
        self.report_anchors(shift=.1)
        self.assertGreater(self.node.generation, generation)
        self.assertIsNone(self.node.filter.room)
        self.refresh()
        self.flow(frame='camera')
        self.node.step()
        self.assertFalse(self.messages['pidrone/localization/local_ready'][-1].data)
        self.ros_now = 50
        self.node.step()
        self.assertIsNone(self.node.filter.stamp)
        self.assertIsNone(self.node.height.range)

    def test_uwb_expiry_and_partial_anchors_never_fall_back_to_camera_room_position(self):
        for _ in range(6):
            self.now += .1
            self.ros_now += .1
            self.refresh()
            self.flow()
            self.report_anchors()
            self.node.step()
        self.assertTrue(self.messages['pidrone/localization/local_ready'][-1].data)
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
        self.assertFalse(self.messages['pidrone/localization/ready'][-1].data)
        self.assertIsNone(self.node.filter.room)

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

    def test_room_navigation_rotates_and_limits_speed_and_cancels_on_manual_or_sensor_loss(self):
        controller = self.controller()
        target = NS(header=NS(frame_id='map', stamp=Stamp(self.ros_now)),
                    pose=NS(position=NS(x=2, y=2, z=999)))
        controller.room_target_callback(target)
        controller.calc_error()
        self.assertAlmostEqual(controller.desired_velocity.x, 0, places=10)
        self.assertAlmostEqual(controller.desired_velocity.y, .2)
        self.assertEqual(controller.desired_position.z, .25)
        controller.room_ready_callback(NS(data=False))
        controller.calc_error()
        self.assertIsNone(controller.room_target)
        self.assertEqual(controller.desired_velocity.planar_magnitude(), 0)
        self.assertTrue(controller.position_control)
        controller.room_ready_callback(NS(data=True))
        controller.room_target_callback(target)
        controller.desired_twist_callback(twist())
        self.assertIsNone(controller.room_target)
        controller.local_ready_callback(NS(data=False))
        controller.calc_error()
        self.assertFalse(controller.position_control)
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


if __name__ == '__main__':
    unittest.main()
