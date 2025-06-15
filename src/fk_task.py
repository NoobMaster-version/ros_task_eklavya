#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''

TODO: Implement a ROS 2 node that reads joint angles and gripper states from angles.txt
and publishes them to the /forward_position_controller/commands topic.

Instructions:
1. Read the joint angles and gripper state from the file 'angles.txt' located in the package's share directory.
   Each line in the file contains: theta_base, theta_shoulder, theta_elbow, gripper_open.
2. Publish the joint angles and gripper state to the topic '/forward_position_controller/commands'.
   The message should contain 5 values in radians: [theta_base, theta_shoulder, theta_elbow, gripper_right, gripper_left].
   Set gripper_right and gripper_left to 0.8 if gripper_open is 1, else 0.0.
3. Add a delay of 2 seconds between publishing each set of joint values.

Your implementation goes below.

'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
from rclpy import qos
from ament_index_python.packages import get_package_share_directory
import os

def dh_transform(theta, d, alpha, a):
    """
    Compute transformation matrix from DH parameters.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics(theta_base, theta_shoulder, theta_elbow):
    """
    Compute forward kinematics for the 4-DOF manipulator.
    Input: joint angles in radians
    Output: end-effector position (x, y, z)
    """
    # Define DH parameters
    d1, a1, alpha1 = 12, 0, np.pi/2
    d2, a2, alpha2 = 0, 7, 0
    d3, a3, alpha3 = 0, 0, np.pi/2
    d4, a4, alpha4 = 10, 0, 0
    theta4 = 0  # Fixed joint angle

    # Compute transformation matrices for each joint
    T1 = dh_transform(theta_base, d1, alpha1, a1)
    T2 = dh_transform(theta_shoulder, d2, alpha2, a2)
    T3 = dh_transform(theta_elbow, d3, alpha3, a3)
    T4 = dh_transform(theta4, d4, alpha4, a4)

    # Overall transformation matrix from base to end-effector
    T_final = T1 @ T2 @ T3 @ T4

    # Extract end-effector position
    return T_final[0:3, 3]

class ForwardKinematicsPublisher(Node):
    def __init__(self):
        super().__init__('forward_kinematics_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            qos_profile=qos.qos_profile_parameter_events
        )
        self.timer = self.create_timer(1.0, self.publish_kinematics)
        self.angle_sets = []
        self.current_index = 0
        self.read_angles_from_file()

    def read_angles_from_file(self):
        """
        Read joint angles and gripper states from angles.txt using ROS 2 package share directory.
        """
        try:
            pkg_share = get_package_share_directory('ros_task_eklavya')
            angles_path = os.path.join(pkg_share, 'ros_task', 'angles.txt')
            with open(angles_path, 'r') as file:
                for line in file:
                    # Parse each line: theta_base,theta_shoulder,theta_elbow,gripper_open
                    angles = [float(x) for x in line.strip().split(',')]
                    if len(angles) != 4:
                        continue  # Skip invalid lines
                    self.angle_sets.append(angles)
        except FileNotFoundError:
            self.get_logger().error("angles.txt not found at expected path")
            rclpy.shutdown()
        except ValueError:
            self.get_logger().error("Invalid format in angles.txt")
            rclpy.shutdown()

    def publish_kinematics(self):
        if self.current_index >= len(self.angle_sets):
            rclpy.shutdown()
            return

        try:
            # Get angles from current set
            theta_base, theta_shoulder, theta_elbow, gripper_open = self.angle_sets[self.current_index]

            # Validate angle range
            if not (0.0 <= theta_base <= 180.0 and 0.0 <= theta_shoulder <= 180.0 and 0.0 <= theta_elbow <= 180.0):
                self.get_logger().warn(f"Skipping invalid angles at index {self.current_index}: {theta_base}, {theta_shoulder}, {theta_elbow}")
                self.current_index += 1
                return

            # Convert to radians
            theta_base_rad = math.radians(theta_base)
            theta_shoulder_rad = math.radians(theta_shoulder)
            theta_elbow_rad = math.radians(theta_elbow)

            # Compute end-effector position
            position = forward_kinematics(theta_base_rad, theta_shoulder_rad, theta_elbow_rad)

            # Print end-effector position in X,Y,Z format
            print(f"{position[0]:.5f},{position[1]:.5f},{position[2]:.5f}")

            # Publish joint angles and gripper state
            joint_msg = Float64MultiArray()
            joint_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            joint_msg.data[0] = theta_base_rad
            joint_msg.data[1] = theta_shoulder_rad
            joint_msg.data[2] = theta_elbow_rad
            joint_msg.data[3] = 0.8 if gripper_open == 1 else 0.0
            joint_msg.data[4] = 0.8 if gripper_open == 1 else 0.0

            self.publisher_.publish(joint_msg)
            self.current_index += 1

        except Exception as e:
            self.get_logger().error(f"Error processing angles: {e}")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
