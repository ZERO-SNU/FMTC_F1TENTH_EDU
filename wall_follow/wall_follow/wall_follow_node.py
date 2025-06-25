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
        # Calculate the index corresponding to the angle
        index = int((angle - angle_min) / angle_increment)
        
        # Ensure index is within bounds
        if index < 0 or index >= len(range_data):
            return float('inf')
            
        # Get the range at the calculated index
        range_val = range_data[index]
        
        # Handle NaN and inf values
        if math.isnan(range_val) or math.isinf(range_val):
            return float('inf')
            
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
        # First try the exact angle
        range_val = self.get_range(range_data, angle, angle_min, angle_increment)
        if not math.isinf(range_val):
            return range_val, angle
            
        # If exact angle didn't work, search around it
        steps = 10  # Number of steps to search on each side
        for i in range(1, steps + 1):
            # Calculate offset proportional to step
            offset = (i / steps) * search_width
            
            # Try both positive and negative offsets
            for direction in [-1, 1]:
                test_angle = angle + direction * offset
                range_val = self.get_range(range_data, test_angle, angle_min, angle_increment)
                
                if not math.isinf(range_val):
                    return range_val, test_angle
                    
        # If no valid measurement found
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
        # For left wall following, we need measurements at the left side
        # and a forward-left angle for prediction
        
        # Get perpendicular distance (90 degrees to left)
        perp_dist, perp_angle = self.find_valid_range(
            range_data, self.left_angle, angle_min, angle_increment)
        
        # Get forward-angled distance (for wall orientation)
        forward_dist, forward_angle = self.find_valid_range(
            range_data, self.left_forward_angle, angle_min, angle_increment)
            
        # Log measurements periodically
        if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:
            self.get_logger().debug(
                f'Left: {perp_dist:.2f}m at {perp_angle:.2f}rad, '
                f'Forward-Left: {forward_dist:.2f}m at {forward_angle:.2f}rad')
        
        # If we couldn't get valid measurements, handle the error case
        if math.isinf(perp_dist) or math.isinf(forward_dist):
            self.get_logger().warn('Invalid range measurements for error calculation')
            # Return a safe default that will slightly turn away from the likely direction of the wall
            return 0.5  # Positive error means we're too close to the wall
            
        # Calculate the wall orientation based on the two measurements
        # For left wall following, we expect forward_dist >= perp_dist if wall is straight
        angle_diff = abs(forward_angle - perp_angle)
        if angle_diff == 0:  # Avoid division by zero
            wall_angle = 0
        else:
            # Calculate wall angle relative to car's heading
            # Positive angle means wall is angling away from car
            wall_angle = math.atan2(forward_dist - perp_dist, self.lookahead_dist)
        
        # Calculate primary error: difference between desired and actual distance
        # Positive error means we're too close to the wall
        distance_error = dist - perp_dist
        
        # Calculate predictive component to react to wall orientation
        # If wall is angling toward us (negative wall_angle), we should turn right
        # If wall is angling away (positive wall_angle), we should turn left
        predictive_component = -wall_angle * 0.6  # Scale factor for prediction
        
        # Combined error - primary distance error with predictive orientation component
        error = distance_error + predictive_component
        
        # Debug info
        if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:
            self.get_logger().debug(
                f'Wall angle: {wall_angle:.3f}, Distance error: {distance_error:.3f}, '
                f'Combined error: {error:.3f}')
                
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
        # Calculate dt (time difference)
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
        self.prev_time = current_time
        
        # Prevent division by zero or unrealistic dt values
        if dt < 0.001 or dt > 1.0:
            dt = 0.1
            
        # Calculate the components of PID
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        i_term = self.ki * self.integral
        
        # Derivative term with filtering
        if abs(dt) > 0.001:  # Avoid division by very small numbers
            derivative = (error - self.prev_error) / dt
            # Simple low-pass filter for derivative to reduce noise
            derivative = 0.8 * derivative + 0.2 * (self.prev_error / dt if dt > 0.001 else 0)
        else:
            derivative = 0
            
        d_term = self.kd * derivative
        
        # Combine the terms to get the steering angle
        # For left wall following, positive error should result in right turn (negative steering)
        steering_angle = -(p_term + i_term + d_term)
        
        # Limit the steering angle
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
        
        # Update previous error
        self.prev_error = error
        
        # Create and fill drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        
        # Set the steering angle
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.steering_angle_velocity = 0.0
        
        # Set the velocity based on error magnitude (slow down on large errors)
        error_magnitude = abs(error)
        adjusted_velocity = velocity
        
        if error_magnitude > self.error_threshold:
            # Reduce speed when error is large - smoother scaling function
            velocity_scale = 1.0 / (1.0 + (error_magnitude - self.error_threshold))
            adjusted_velocity = max(self.min_velocity, velocity * velocity_scale)
            
        # Further reduce speed when turning sharply
        steering_magnitude = abs(steering_angle)
        if steering_magnitude > 0.2:  # If turning more than ~11 degrees
            steering_factor = 1.0 - (steering_magnitude - 0.2) / 0.4
            adjusted_velocity = adjusted_velocity * max(0.5, steering_factor)
            
        drive_msg.drive.speed = max(self.min_velocity, adjusted_velocity)
        drive_msg.drive.acceleration = 0.0
        drive_msg.drive.jerk = 0.0
        
        # Publish the drive message
        self.drive_pub.publish(drive_msg)
        
        # Log debug information periodically
        if int(current_time.nanoseconds / 1e9) % 2 == 0:  # Log every 2 seconds
            self.get_logger().info(
                f'Error: {error:.2f}, Steering: {steering_angle:.2f}, Velocity: {adjusted_velocity:.2f}')

        return
    
    def publish_stop(self):
        """
        Publish a stop command to the vehicle
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)
        self.get_logger().warn('Emergency stop triggered')

        return

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish
        the drive message in this function.
        
        Args:
            msg: Incoming LaserScan message
            
        Returns:
            None
        """
        
        # Find current env (Sim vs. Real)
        if self.exec_mode == None:
            self.get_logger().info('Finding current env... (Sim vs. Real)')

            node = rclpy.create_node("list_nodes_example")
            available_nodes = get_node_names(node=node, include_hidden_nodes=False)
            for nodes in available_nodes:
                if nodes.full_name == "/ego_robot_state_publisher":
                    self.get_logger().info('Current env is SIM')
                    self.exec_mode = "sim"
                    break
            else:
                self.get_logger().info('Current env is REAL')
                self.exec_mode = "real"

            node.destroy_node()

        # Extract LaserScan data
        ranges = None
        if self.exec_mode == "sim":
            ranges = msg.ranges
        elif self.exec_mode == "real":
            half_index = len(msg.ranges) // 2
            ranges = msg.ranges[half_index+1:-1]
            ranges.extend(msg.ranges[0:half_index])
        
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Check if we have enough valid measurements
        valid_ranges = [r for r in ranges if not (math.isnan(r) or math.isinf(r))]
        
        if len(valid_ranges) < self.min_valid_measurements:
            self.get_logger().warn(f'Not enough valid range measurements: {len(valid_ranges)}/{self.min_valid_measurements}')
            # Trigger emergency stop due to insufficient valid data
            self.publish_stop()
            return
        
        # Update the last valid scan time
        self.last_valid_scan_time = self.get_clock().now()
        
        try:
            # Calculate error
            error = self.get_error(ranges, self.desired_distance, angle_min, angle_increment)
            
            # Check if error calculation returned a valid result
            if math.isnan(error) or math.isinf(error):
                self.get_logger().warn('Invalid error calculated')
                self.publish_stop()
                return
            
            # Calculate desired velocity based on error
            # Base velocity with conservative value
            base_velocity = self.max_velocity
            
            # Call PID control with the calculated error and velocity
            self.pid_control(error, base_velocity)
            
        except Exception as e:
            self.get_logger().error(f'Exception in scan callback: {str(e)}')
            self.publish_stop()
        
        return
    
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
