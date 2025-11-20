#!/usr/bin/env python3
"""
HazardNode (simple boolean version)
-----------------------------------
Subscribes to /hazard_detection and calls a user-supplied
callback with True if a bump or stall is detected.
"""

from rclpy.node import Node
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from boustrophedon.qos_profile import sensor_qos


class HazardNode(Node):
    def __init__(self, user_callback=None):
        """
        Args:
            user_callback (callable): Function called when bump/stall detected.
        """
        super().__init__('hazard_node')
        self.user_callback = user_callback

        self.sub = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self._hazard_callback,
            sensor_qos
        )

        self.get_logger().info('HazardNode ready.')

    def _hazard_callback(self, msg: HazardDetectionVector):
        """Check if hazard message contains a bump or stall."""
        try:
            if self.user_callback:
                hazard_detected = any(
                    d.type in (HazardDetection.BUMP, HazardDetection.STALL)
                    for d in msg.detections
                )
                if hazard_detected:
                    self.user_callback()
        except Exception as e:
            self.get_logger().error(f'Exception in user hazard callback: {e}')
