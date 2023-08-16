#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

class GantryController(Node):
    def __init__(self):
        super().__init__('gantry_controller')

        self._jog_topic = "containerbruecke/joint_jog"
        self._joint_state_topic = "containerbruecke/joint_state"
        self._joint_jog_publisher = None

        # JointState in order: [Achse1, Achse2_A, Achse2_B, Achse6, Achse4, Achse5, Achse3, Achse7]
        self._joint_states = [0.0]*3
    
        #  [boom tilt, extension, pitcher, yaw, pitcher extension] 
        self.max_velocities = [66.65, 125.0, 95.8]   #EFFETIVE MAX VELOCITIES

        self._joint_jog_msg = JointJog()
        self._joint_jog_msg.header.frame_id = "JointJog"
        self._joint_jog_msg.joint_names = [
            "Achse1",
            "Achse2",
            "Achse3",
        ]

        # Flag for pause state
        self.is_paused = False

        # Initialize the counter for 50Hz callback calls
        self.callback_counter = 0
        # Counter for custom value sets
        self.value_set_counter = 0

        # Define custom values with x seconds duration
        self.custom_values = [
            {"values": [65.0, 0.0, 0.0], "duration": 20},
            {"values": [0.0, 110.0, 0.0], "duration": 25}, 
            {"values": [0.0, 0.0, -65.0], "duration": 25}, 
            {"values": [0.0, 0.0, 65.0], "duration": 25}, 
            {"values": [0.0, -110.0, 0.0], "duration": 25}, 
            {"values": [-60.0, 0.0, 0.0], "duration": 20}, 
            {"values": [0.0, 0.0, -65.0], "duration": 25},
        ]

        # Set timer for sending joint jog at 50hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self._register_subscribers()
        self._register_publishers()

    def timer_callback(self):
        """Sending velocity of the axes at 50Hz"""
        # If all custom values have been sent, send [0.0, 0.0, 0.0] and stop the timer
        if self.value_set_counter >= len(self.custom_values):
            self._send_joint_jog([], [0.0, 0.0, 0.0])
            self.timer.cancel()
            return

        # If in pause state, just return
        if self.is_paused:
            return

        # Send the values based on the current value set counter
        current_values = self.custom_values[self.value_set_counter]["values"]
        self._send_joint_jog([], current_values)

        # Increment the callback counter
        self.callback_counter += 1

        # Calculate the number of callbacks for the current set's duration
        num_callbacks_for_duration = self.custom_values[self.value_set_counter]["duration"] * 50  # 50Hz

        # If it's time to switch to the next set of values
        if self.callback_counter >= num_callbacks_for_duration:
            self.callback_counter = 0
            self.is_paused = True
            # Pause for 3 seconds
            self.pause_timer = self.create_timer(3, self.pause_callback)

    def pause_callback(self):
        """Callback after pause duration"""
        self.is_paused = False
        self.value_set_counter += 1
        self.pause_timer.cancel()

    def _send_joint_jog(self, targets, velocities):
        if self._joint_jog_publisher is None:
            return
        self._joint_jog_msg.header.stamp = self.get_clock().now().to_msg()
        self._joint_jog_msg.displacements = targets
        self._joint_jog_msg.velocities = velocities
        self._joint_jog_publisher.publish(self._joint_jog_msg)

    def _register_subscribers(self):
        pass
        # self._state_subscriber = self.create_subscription(
        #     JointState,
        #     self._joint_state_topic,
        #     self._state_subscription_callback,
        #     qos_profile=1,
        # )

    def _register_publishers(self):
        self._unregister_publishers()
        self._joint_jog_publisher = self.create_publisher(
            JointJog, self._jog_topic, qos_profile=QoSProfile(depth=1)
        )

    def _unregister_publishers(self):
        if self._joint_jog_publisher is not None:
            self.destroy_publisher(self._joint_jog_publisher)
            self._joint_jog_publisher = None

def main(args=None):
    rclpy.init(args=args)

    gantry_controller = GantryController()
    rclpy.spin(gantry_controller)

    gantry_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
