#!/usr/bin/env python3
"""
IrNode - ordered infrared (IR) intensity reader with optional remapping
-----------------------------------------------------------------------

Purpose (the *why*)
-------------------
This node listens to the robot’s IR proximity sensors and stores the most
recent readings as a 7-element list representing left→right across the
robot’s front bumper.

Real robots and simulators sometimes publish these sensors in different
orders.  The real Create 3 publishes them in physical order; some
simulators do not.  We expose an optional "mapping" list so you can
experiment, discover, and apply the correct remapping themselves.

Why this matters
----------------
The IR sensors measure reflected infrared light — higher values mean
closer obstacles.  By keeping these values in physical left->right order,
students can easily reason about what the robot "sees" in space.

Public API
----------
- get_readings() -> list[float] | None
    Returns the latest 7 IR readings ordered left->-right, or None if no data.

Optional parameter
------------------
mapping : list[int] | None
    A list of 7 indices describing how to reorder incoming data
    to match the robot’s physical left→right layout.

    Example simulator mapping (to be discovered by you):
        # incoming sim order:
        #   0 right-center-front
        #   1 right-front
        #   2 right
        #   3 left-center-front
        #   4 left-front
        #   5 left
        #   6 front
    Once determined, you can pass:
        mapping = [5, 4, 3, 6, 0, 1, 2]
    or similar, depending on what is found experimentally.
"""
from typing import Optional, List
from rclpy.node import Node
from irobot_create_msgs.msg import IrIntensityVector
from boustrophedon.qos_profile import sensor_qos


class IrNode(Node):
    """Caches the latest IR intensity readings in left→right order."""

    def __init__(self, mapping: Optional[List[int]] = None):
        """
        Args:
            mapping (list[int] | None): Optional list of indices that remap
                incoming readings into physical left→right order.
                If None, readings are used in published order.
        """
        super().__init__('ir_node')

        self._mapping = mapping
        self._latest_ir: Optional[IrIntensityVector] = None
        self._readings: Optional[List[float]] = None
        self._started = False

        # Subscribe to IR topic
        self.sub = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self._ir_callback,
            sensor_qos
        )

        if self._mapping:
            self.get_logger().info(f'IrNode ready with custom mapping: {self._mapping}')
        else:
            self.get_logger().info('IrNode ready (using native sensor order).')


    def _ir_callback(self, msg: IrIntensityVector) -> None:
        """
        Store latest IR readings, optionally applying a left→right mapping.

        Notes
        -----
        The message’s `readings` field contains a list of IRIntensity values.
        If a mapping is provided, we reorder them accordingly.
        """
        self._latest_ir = msg
        values = [r.value for r in msg.readings]

        if self._mapping:
            # Remap incoming indices to desired left→right order
            self._readings = [values[i] for i in self._mapping]
        else:
            self._readings = values

        if not self._started:
            self._started = True
            self.get_logger().info('IR intensity messages are coming in!')


    def get_readings(self) -> Optional[List[float]]:
        """
        Return the latest 7 IR intensity values ordered left→right.

        Returns
        -------
        list[float] | None
            None if no IR data received yet; otherwise a list of 7 floats.
        """
        return self._readings
