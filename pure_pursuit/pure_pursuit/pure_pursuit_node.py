#!/usr/bin/env python3
"""
pure_pursuit_node.py  ─ ROS 2 Foxy  (rclpy)

• 구조 : ego_racecar/odom       (nav_msgs/msg/Odometry)
• 발화 : /drive        (ackermann_msgs/msg/AckermannDriveStamped)
         /env_viz    (visualization_msgs/msg/Marker)   – 전체 경로
         /dynamic_viz(visualization_msgs/msg/Marker)   – 현재 look-ahead 점
"""

# ─────────── Imports ─────────── 
import csv
import math
import threading
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

# ─────────── Parameters ───────────
LOOKAHEAD_DISTANCE = 1.5
angle_P = 1.0
K_P = 0.5
K_I = 0.0
K_D = 0.1
PI = math.pi
WB = 0.27
DATA_CSV_PATH = Path("/home/car3/garage/path_clean.csv").with_name('path_clean.csv')
speed_max = 1.7
speed_min = 1.0

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.pub_nav = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.pub_envviz = self.create_publisher(Marker, '/env_viz', 1)
        self.pub_dynviz = self.create_publisher(Marker, '/dynamic_viz', 1)

        self.create_subscription(PoseStamped, '/pf/viz/inferred_pose', self.odom_cb, 10)            #real
        #self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)          #simulation

        self.xs, self.ys = self._load_waypoints(DATA_CSV_PATH)
        self.env_marker = self._make_marker(self.xs, self.ys, color=(0, 0, 1), scale=0.1)

        
        self.temp_idx = 0
        self.temp_idx_sub1 = 0
        self.current_idx = 0
        self.angle_cmd = 0.0

        self.x_cur = None
        self.y_cur = None
        self.yaw_cur = None
        self.x_temp = None
        self.y_temp = None

        self.prev_error = 0.0
        self.integral_error = 0.0

        self.get_logger().info(f'Loaded {len(self.xs)} way-points from {DATA_CSV_PATH}')

        self.ready_to_drive = False
        self.prestart_duration = 3.0
        self.prestart_elapsed = 0.0
        self.pre_start_timer = self.create_timer(0.2, self._publish_prestart_drive)

        threading.Thread(target=self._wait_to_start, daemon=True).start()

    def _wait_to_start(self):
        input("\n[PurePursuitNode] Press ENTER to start the node...\n")
        self.get_logger().info("Starting Pure Pursuit logic after user input.")
        self.ready_to_drive = True
        self.pre_start_timer.cancel()

    def _publish_prestart_drive(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        self.pub_nav.publish(msg)

        self.prestart_elapsed += 0.2
        if self.prestart_elapsed >= self.prestart_duration:
            self.pre_start_timer.cancel()
            self.get_logger().info("Pre-start drive publishing complete.")

    def odom_cb(self, odom: PoseStamped):           #real
    #def odom_cb(self,odom: Odometry):          #simulation
        if not self.ready_to_drive:
            return
        
        q = odom.pose.orientation           #real
        #q = odom.pose.pose.orientation         #simulation
        _, _, self.yaw_cur = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.x_cur = odom.pose.position.x - 0.27 * np.cos(self.yaw_cur)         #real
        self.y_cur = odom.pose.position.y - 0.27 * np.sin(self.yaw_cur)         #real
        #self.x_cur = odom.pose.pose.position.x - 0.27 * np.cos(self.yaw_cur)           #simulation
        #self.y_cur = odom.pose.pose.position.y - 0.27 * np.sin(self.yaw_cur)           #simulation

        d_min = float('inf')
        for i, (xw, yw) in enumerate(zip(self.xs, self.ys)):
            d = (xw - self.x_cur)**2 + (yw - self.y_cur)**2
            if d < d_min:
                d_min = d
                self.temp_idx = i

        self.temp_idx_sub1 = (self.temp_idx - 1) % len(self.xs)
        while self._distance(self.xs[self.temp_idx], self.ys[self.temp_idx],
                     self.x_cur, self.y_cur) < LOOKAHEAD_DISTANCE:
              self.temp_idx = (self.temp_idx + 1) % len(self.xs)
        self.temp_idx_sub1 = (self.temp_idx - 1) % len(self.xs)
        x1, y1 = self.xs[self.temp_idx_sub1], self.ys[self.temp_idx_sub1]
        x2, y2 = self.xs[self.temp_idx], self.ys[self.temp_idx]  
        
        self.x_temp = np.linspace(x1, x2, 10)
        self.y_temp = np.linspace(y1, y2, 10)

        for i in range(10):
            if self._distance(self.x_temp[i], self.y_temp[i], self.x_cur, self.y_cur) >= LOOKAHEAD_DISTANCE:
                self.current_idx = i
                break
        else:
            self.current_idx = 9

        x_goal = self.x_temp[self.current_idx]
        y_goal = self.y_temp[self.current_idx]
        Ld = max(self._distance(x_goal, y_goal, self.x_cur, self.y_cur), 1e-4)
        global_ang = math.atan2(y_goal - self.y_cur, x_goal - self.x_cur)
        alpha = ((global_ang - self.yaw_cur + PI) % (2*PI)) - PI
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Ld, 1.0)
        d_y = Ld * math.sin(alpha)

        error = d_y
        self.integral_error += error
        derivative = error - self.prev_error
        self.prev_error = error
        pid_term = (K_P * error + K_I * self.integral_error + K_D * derivative)

        self.angle_cmd = float(angle_P * delta + pid_term * 2.0 / (Ld * Ld))

        self._publish_drive()
        self.pub_envviz.publish(self.env_marker)
        self.pub_dynviz.publish(
            self._make_marker([x_goal], [y_goal], color=(1, 0, 0), scale=0.2)
        )

    def _publish_drive(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(self.angle_cmd)

        # 속도 조절
        steer_max = math.radians(30.0)  # 최대 조향각 기준

        ratio = min(abs(self.angle_cmd) / steer_max, 1.0)  # [0, 1]로 제한
        msg.drive.speed = speed_max - (speed_max - speed_min) * ratio

        self.pub_nav.publish(msg)

    @staticmethod
    def _distance(x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)

    @staticmethod
    def _load_waypoints(DATA_CSV_PATH):
        xs = []
        ys = []
        with open(DATA_CSV_PATH, newline='') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for row in reader:
                x = float(row[0])
                y = float(row[1])
                xs.append(x)
                ys.append(y)
        return xs, ys

    @staticmethod
    def _make_marker(xs, ys, color=(0, 0, 1), scale=0.1):
        m = Marker()
        m.header.frame_id = 'map'
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.color.a = 1.0
        m.color.r = float(color[0])
        m.color.g = float(color[1])
        m.color.b = float(color[2])
        m.points = [Point(x=x, y=y, z=0.0) for x, y in zip(xs, ys)]
        return m


def main():
    rclpy.init()
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
