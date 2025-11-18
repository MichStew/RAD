#!/usr/bin/env python3
"""
OdomNode - simple accessor for robot odometry (encoder-like)

Reads /odom (nav_msgs/msg/Odometry) and provides:
  - get_pose()     -> (x, y, theta)  in meters and radians
  - get_velocity() -> (vx, vy, omega) in m/s and rad/s
"""

from typing import Optional, Tuple
import math

from rclpy.node import Node
from nav_msgs.msg import Odometry
from boustrophedon.qos_profile import sensor_qos  # your project QoS

class OdomNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_node')
        self._latest: Optional[Odometry] = None
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, sensor_qos)
        self.get_logger().info('OdomNode ready (listening to /odom).')


    def _odom_cb(self, msg: Odometry) -> None:
        """Cache latest odometry message for polling by other code."""
        self._latest = msg


    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        Return (x, y, theta) where theta is yaw in radians.
        Returns None if no odom message received yet.
        """
        if self._latest is None:
            return None
        p = self._latest.pose.pose.position
        q = self._latest.pose.pose.orientation
        # convert quaternion to yaw (4D quaternion into 2D heading)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (float(p.x), float(p.y), float(yaw))


    def get_velocity(self) -> Optional[Tuple[float, float, float]]:
        """
        Return (vx, vy, omega_z) in m/s and rad/s from odom.twist.
        Returns None if no odom message received yet.
        """
        if self._latest is None:
            return None
        lin = self._latest.twist.twist.linear
        ang = self._latest.twist.twist.angular
        return (float(lin.x), float(lin.y), float(ang.z))

