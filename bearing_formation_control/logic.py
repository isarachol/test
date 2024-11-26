#!/usr/bin/env python3

"""
A file to compute the control command of a given robot, and then publish
the command to the "commands" topic
"""

import rclpy
from rclpy.node import Node
import numpy as np

from me570_final_project.msg import command, targets, sensordata
from turtlesim.msg import Pose

ROBOT_ID = 0

current_angle = 0
tag_angles = np.array([])
target_tag_angles = np.array([])
tags_visible = np.array([])
F_X = 25.05  # horizonatal focal length in mm (using the color FOV of an ORBEC DaBai Stereo Depth Camera and assuming a sensor size of 36mm by 20.25mm)
C_X = 0  # horizontal center of the camera in mm


class RobotControlPublisher(Node):

    def __init__(self):
        super().__init__("robot_control_publisher")
        self.publisher_ = self.create_publisher(command, "turtle1/commands", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = command()
        msg.robotid = ROBOT_ID
        msg.controlcommand = get_command(self)
        self.publisher_.publish(msg)


class SensorDataSubscriber(Node):

    def __init__(self):
        super().__init__("sensor_data_subscriber")
        self.subscription = self.create_subscription(
            sensordata, "turtle1/sensor", self.listener_callback, 10
        )
        self.subscription

    def listener_callback(self, msg):
        global tag_locations, tags_visible, tag_angles
        tag_locations = np.array(msg.tag_coords_x)
        tag_angles = np.empty_like(tag_locations)
        tags_visible = np.array(msg.tags_visible)
        calc_angles()


class TargetAngleSubscriber(Node):

    def __init__(self):
        super().__init__("target_angle_subscriber")
        self.subscription = self.create_subscription(
            targets, "turtle1/targets", self.listener_callback2, 10
        )
        self.subscription

    def listener_callback2(self, msg):
        global target_tag_angles
        if msg.robotid == ROBOT_ID:
            target_tag_angles = np.array(msg.target_tag_angles)


class RobotPoseSubscriber(Node):

    def __init__(self):
        super().__init__("robot_pose_subscriber")
        self.subscription = self.create_subscription(
            Pose, "turtle1/pose", self.listener_callback2, 10
        )
        self.subscription

    def listener_callback2(self, msg):
        global current_angle
        if msg.theta < (np.pi / 2.0):
            current_angle = ((3.0 * np.pi) / 2.0) + msg.theta
            return
        current_angle = msg.theta - (np.pi / 2.0)


def get_command(publisher):
    global current_angle

    angle_tol = 0.01
    epsilon = 1e-6  # Small value to avoid divide by 0
    try:
        if target_tag_angles.size == 0 or tag_angles.size == 0:
            return "s"
        target_tag_indicies = np.where(target_tag_angles >= 0)
        current_tag_angles = tag_angles[target_tag_indicies]
        if any(current_tag_angles < 0):
            return "r"
        if all(
            abs(target_tag_angles[target_tag_indicies] - current_tag_angles)
            <= angle_tol
        ):
            return "s"

        theta_1_c = current_tag_angles[0]
        theta_2_c = current_tag_angles[1]

        theta_1_t = target_tag_angles[target_tag_indicies][0]
        theta_2_t = target_tag_angles[target_tag_indicies][1]

        x_current = (-np.tan(theta_1_c)) / (
            np.tan(theta_1_c) - np.tan(theta_2_c) + epsilon
        )
        y_current = (-np.tan(theta_2_c) * np.tan(theta_1_c)) / (
            np.tan(theta_1_c) - np.tan(theta_2_c) + epsilon
        )

        x_target = (-np.tan(theta_1_t)) / (
            np.tan(theta_1_t) - np.tan(theta_2_t) + epsilon
        )
        y_target = (-np.tan(theta_2_t) * np.tan(theta_1_t)) / (
            np.tan(theta_1_t) - np.tan(theta_2_t) + epsilon
        )

        delta_x = x_target - x_current
        delta_y = y_target - y_current

        nav_angle = np.arctan(delta_x / delta_y + epsilon)

        if nav_angle < 0:
            nav_angle += 360

        if current_angle < nav_angle + angle_tol:
            current_angle += 1
            if current_angle >= 360.0:
                current_angle -= 360
            return "l"

        if current_angle > nav_angle - angle_tol:
            current_angle -= 1
            if current_angle < 0.0:
                current_angle += 360
            return "r"

        if abs(current_angle - nav_angle) < angle_tol:
            current_tag_angles.fill(-1)
            return "f"

    except Exception as e:
        publisher.get_logger().error(f"Error in get_command: {e}")
        return "s"


def calc_angles():
    visible_tags = np.where(tags_visible)[0]
    for i in visible_tags:
        angle = np.arctan((tag_locations[i] - C_X) / F_X)
        tag_angles[i] = angle + current_angle


def main(args=None):
    rclpy.init(args=args)

    robot_control_publisher = RobotControlPublisher()

    rclpy.spin(robot_control_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_control_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
