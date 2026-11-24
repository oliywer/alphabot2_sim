#!/usr/bin/env python3
"""Map a VR head pose (e.g. Meta Quest 3 via WebXR + rosbridge) to the AlphaBot2 pan-tilt camera.

Subscribes:  <pose_topic>   geometry_msgs/PoseStamped  (head orientation)
Publishes:   /pan_cmd, /tilt_cmd   std_msgs/Float64  (joint angles in rad)

Parameters:
  pose_topic   (str)   default '/quest/head_pose'  -> change to your real topic
  convention   (str)   'webxr' (Y-up, -Z forward, raw WebXR pose) or 'ros' (Z-up, X-forward)
  pan_limit    (float) default 1.57 rad
  tilt_limit   (float) default 1.04 rad
  invert_pan / invert_tilt (bool) flip direction if needed
  recenter_on_start (bool) default True: the first received pose becomes "straight ahead"
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class HeadToPanTilt(Node):
    def __init__(self):
        super().__init__('head_to_pantilt')
        self.declare_parameter('pose_topic', '/quest/head_pose')
        self.declare_parameter('convention', 'webxr')
        self.declare_parameter('pan_limit', 1.57)
        self.declare_parameter('tilt_limit', 1.04)
        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', False)
        self.declare_parameter('recenter_on_start', True)

        topic = self.get_parameter('pose_topic').value
        self.yaw0 = None
        self.pan_pub = self.create_publisher(Float64, '/pan_cmd', 10)
        self.tilt_pub = self.create_publisher(Float64, '/tilt_cmd', 10)
        self.create_subscription(PoseStamped, topic, self.on_pose, 10)
        self.get_logger().info(f'Listening for head pose on {topic}')

    def yaw_pitch(self, q):
        """Return (yaw_left_positive, pitch_up_positive) in rad."""
        x, y, z, w = q.x, q.y, q.z, q.w
        if self.get_parameter('convention').value == 'webxr':
            # Y-up, -Z forward. Yaw about +Y (left positive), pitch about +X (up positive)
            yaw = math.atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (x * x + y * y))
            pitch = math.asin(clamp(2.0 * (w * x - y * z), -1.0, 1.0))
        else:
            # ROS: Z-up, X forward. Yaw about +Z (left positive), pitch about +Y (down positive)
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            pitch = -math.asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0))
        return yaw, pitch

    def on_pose(self, msg: PoseStamped):
        yaw, pitch = self.yaw_pitch(msg.pose.orientation)

        if self.yaw0 is None:
            self.yaw0 = yaw if self.get_parameter('recenter_on_start').value else 0.0
        yaw = wrap(yaw - self.yaw0)

        pan = yaw          # pan_joint: +Z axis -> positive = turn left
        tilt = -pitch      # tilt_joint: +Y axis -> positive = look down
        if self.get_parameter('invert_pan').value:
            pan = -pan
        if self.get_parameter('invert_tilt').value:
            tilt = -tilt

        pl = self.get_parameter('pan_limit').value
        tl = self.get_parameter('tilt_limit').value
        self.pan_pub.publish(Float64(data=clamp(pan, -pl, pl)))
        self.tilt_pub.publish(Float64(data=clamp(tilt, -tl, tl)))


def main():
    rclpy.init()
    node = HeadToPanTilt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
