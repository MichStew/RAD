#!/usr/bin/env python3
"""
ButtonNode for iRobot Create 3
------------------------------
Listens to /interface_buttons and calls user-supplied callbacks
when buttons are pressed.

- power_button_callback: required (called when Power button pressed)
- button_1_callback: optional (called when Button 1 pressed)
- button_2_callback: optional (called when Button 2 pressed)

Each callback should take no arguments.
"""
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import InterfaceButtons


class ButtonNode(Node):
    def __init__(
        self,
        power_button_callback,
        button_1_callback=None,
        button_2_callback=None,
        demo_mode: bool = False
    ):
        """
        Args:
            power_button_callback (callable): Function called when Power button pressed.
            button_1_callback (callable, optional): Called when Button 1 pressed.
            button_2_callback (callable, optional): Called when Button 2 pressed.
            demo_mode (bool): If True, suppress beeps/logging for testing.
        """
        super().__init__('button_node')

        if power_button_callback is None:
            raise ValueError("power_button_callback must be provided")

        self.demo_mode = demo_mode
        self.power_button_callback = power_button_callback
        self.button_1_callback = button_1_callback
        self.button_2_callback = button_2_callback

        # Subscribe to button state messages
        self.sub = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self._button_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('ButtonNode ready.')

    def _button_callback(self, msg: InterfaceButtons):
        """Call appropriate user callback for pressed button."""
        try:
            if msg.button_power.is_pressed:
                if not self.demo_mode:
                    self.get_logger().info('Power button pressed.')
                self.power_button_callback()

            elif msg.button_1.is_pressed and self.button_1_callback:
                if not self.demo_mode:
                    self.get_logger().info('Button 1 pressed.')
                self.button_1_callback()

            elif msg.button_2.is_pressed and self.button_2_callback:
                if not self.demo_mode:
                    self.get_logger().info('Button 2 pressed.')
                self.button_2_callback()

            # If no button pressed, do nothing
        except Exception as e:
            self.get_logger().error(f'Exception in _button_callback: {e}')
