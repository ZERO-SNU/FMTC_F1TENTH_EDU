#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class WallFollow(Node):
    """
    Implement Wall Following on the car - follows left wall
    """
    def __init__(self):
        super().__init__('wall_follow_node')
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        # Create subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10)
            
        # Improved PID gains for stability
        self.kp = 1.0  # Proportional gain (reduced from 1.0)
        self.kd = 0.1  # Derivative gain (increased for damping)
        self.ki = 0.0001  # Integral gain (reduced to prevent oscillation)
        
        # Store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        self.exec_mode = None # "sim" or "real"
        
        # Parameters
        self.desired_distance = 1.0  # Desired distance from wall in meters (increased for safety)
        self.lookahead_dist = 1.0  # Distance to look ahead for prediction
        self.lookahead_angle = 90.0 * (math.pi / 180.0)  # 45 degrees in radians
        self.max_integral = 5.0  # Integral windup prevention
        self.max_steering_angle = 0.4  # Maximum steering angle in radians
        self.prev_time = self.get_clock().now()
        
        # Parameters for velocity control - more conservative
        self.max_velocity = 2.0  # Maximum velocity (reduced)
        self.min_velocity = 0.3  # Minimum velocity (reduced)
        self.error_threshold = 0.3  # Error threshold for velocity scaling (reduced)
        
        # Safety parameters - enhanced
        self.min_valid_measurements = 15  # Minimum number of valid measurements required
        self.safety_timeout = 0.5  # Timeout in seconds for safety checks (reduced)
        self.last_valid_scan_time = self.get_clock().now()
        
        # Angles for left wall following (in radians)
        self.left_angle = math.pi/2  # 90 degrees - left side of the car
        self.left_forward_angle = math.pi/4  # 45 degrees - forward-left of the car
        
        self.get_logger().info('Wall Follow Node has been initialized - Following LEFT wall')

    def get_range(self, range_data, angle, angle_min, angle_increment):
        """
        Simple helper to return the corresponding range measurement at a given angle.
        Takes care of NaNs and infs.
        
        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR
            angle_min: minimum angle of the LiDAR
            angle_increment: angle increment between LiDAR measurements
            
        Returns:
            range: range measurement in meters at the given angle
        """
        range_val = 0
        
        return range_val

    def find_valid_range(self, range_data, angle, angle_min, angle_increment, search_width=0.35):
        """
        Find a valid range measurement around the specified angle
        
        Args:
            range_data: LiDAR range data
            angle: target angle
            angle_min: minimum angle of the LiDAR
            angle_increment: angle increment
            search_width: how far to search in radians
            
        Returns:
            valid_range: valid range measurement or inf if none found
            actual_angle: actual angle where measurement was found
        """

        angle = 0

        return float('inf'), angle

    def get_error(self, range_data, dist, angle_min, angle_increment):
        """
        Calculates the error to the left wall (going counter-clockwise)
        
        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall
            angle_min: minimum angle of the LiDAR
            angle_increment: angle increment between LiDAR measurements
            
        Returns:
            error: calculated error
        """

        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control
        
        Args:
            error: calculated error
            velocity: desired velocity
            
        Returns:
            None
        """

    def publish_stop(self):
        """
        Publish a stop command to the vehicle
        """

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish
        the drive message in this function.
        
        Args:
            msg: Incoming LaserScan message
            
        Returns:
            None
        """
        
    def check_safety_timeout(self):
        """
        Check if we've gone too long without a valid scan
        """
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_valid_scan_time).nanoseconds / 1e9
        
        if elapsed > self.safety_timeout:
            self.get_logger().error(f'Safety timeout: {elapsed:.2f}s without valid scans')
            self.publish_stop()
            return True
        
        return False

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized - Following Left Wall")
    wall_follow_node = WallFollow()
    
    try:
        # Use a timer to periodically check safety conditions
        safety_timer = wall_follow_node.create_timer(0.1, wall_follow_node.check_safety_timeout)
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        print("Wall follow node terminated by user")
    except Exception as e:
        print(f"Exception in wall follow node: {e}")
    finally:
        # Destroy the node explicitly
        wall_follow_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
