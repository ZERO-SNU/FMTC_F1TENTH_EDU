#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class FollowTheGapImproved(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.exec_mode = None

        # 차량 파라미터
        self.declare_parameter('max_steering_angle', 0.85)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('safe_distance', 1.0)
        self.declare_parameter('bubble_radius', 0.5)

        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
        self.bubble_radius = self.get_parameter('bubble_radius').get_parameter_value().double_value

        # 주행 파라미터
        self.declare_parameter('front_angle_range', 120)
        self.declare_parameter('side_angle_range', 60)
        self.declare_parameter('gap_threshold', 0.5)

        self.front_angle_range = self.get_parameter('front_angle_range').get_parameter_value().integer_value
        self.side_angle_range = self.get_parameter('side_angle_range').get_parameter_value().integer_value
        self.gap_threshold = self.get_parameter('gap_threshold').get_parameter_value().double_value

        self.get_logger().info('Improved FollowTheGap node started!')

    def scan_callback(self, msg: LaserScan):
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
            ranges = np.array(msg.ranges)
        elif self.exec_mode == "real":
            half_index = len(msg.ranges) // 2
            ranges = np.concatenate((msg.ranges[half_index+1:-1], msg.ranges[0:half_index]))

        # 라이다 데이터 처리
        angle_min, angle_max = msg.angle_min, msg.angle_max
        angle_increment = msg.angle_increment

        # 유효하지 않은 값 처리 (NaN, Inf)
        ranges = np.nan_to_num(ranges, nan=msg.range_max + 1.0, posinf=msg.range_max + 1.0)

        # 유효한 범위로 필터링
        valid_ranges = np.clip(ranges, msg.range_min, msg.range_max)

        # 전방 인덱스 계산 (중앙 기준으로 ±front_angle_range/2)
        total_angles = len(valid_ranges)
        front_indices_span = int(self.front_angle_range / 360 * total_angles)
        center_idx = total_angles // 2
        front_start = max(0, center_idx - front_indices_span // 2)
        front_end = min(total_angles, center_idx + front_indices_span // 2)

        # 전방 데이터 추출
        front_ranges = valid_ranges[front_start:front_end]

        if not front_ranges.size:
            self.publish_drive(0.0, self.min_speed)
            self.get_logger().warn('No front ranges data available.')
            return

        # 가장 가까운 장애물 위치 파악
        min_dist_idx = np.argmin(front_ranges)
        min_dist = front_ranges[min_dist_idx]

        # 장애물 주변에 안전 영역(bubble) 설정
        bubble_width_rad = self.bubble_radius / np.mean(front_ranges) if np.mean(front_ranges) > 0 else 0
        bubble_width_indices = int(bubble_width_rad / angle_increment)
        masked_ranges = np.copy(front_ranges)

        # 가장 가까운 장애물 주변에 버블 적용
        bubble_start = max(0, min_dist_idx - bubble_width_indices)
        bubble_end = min(len(front_ranges), min_dist_idx + bubble_width_indices + 1)
        masked_ranges[bubble_start:bubble_end] = 0.0  # 0으로 마스크

        # Gap 찾기 (safe_distance 이상의 거리)
        masked_ranges[masked_ranges < self.safe_distance] = 0.0

        # 가장 큰 gap 찾기
        max_gap_start, max_gap_end = self.find_largest_gap(masked_ranges)

        if max_gap_end - max_gap_start < 5:  # gap이 너무 작으면
            # 좌우 측면 거리를 비교하여 더 넓은 쪽으로 이동
            side_indices_span = int(self.side_angle_range / 360 * total_angles)
            left_side_idx = max(0, center_idx - total_angles // 4 - side_indices_span // 2)
            left_side_end = min(total_angles, center_idx - total_angles // 4 + side_indices_span // 2)
            right_side_idx = max(0, center_idx + total_angles // 4 - side_indices_span // 2)
            right_side_end = min(total_angles, center_idx + total_angles // 4 + side_indices_span // 2)

            left_side_dist = np.mean(valid_ranges[left_side_idx:left_side_end]) if left_side_idx < left_side_end else 0.0
            right_side_dist = np.mean(valid_ranges[right_side_idx:right_side_end]) if right_side_idx < right_side_end else 0.0

            if left_side_dist > right_side_dist:
                steering_angle = self.max_steering_angle  # 왼쪽으로 조향
            else:
                steering_angle = -self.max_steering_angle  # 오른쪽으로 조향

            speed = self.min_speed * 0.8  # 좁은 공간에서는 속도 감소
        else:
            # gap의 중앙 인덱스 계산 (전방 범위 내)
            best_gap_local_idx = (max_gap_start + max_gap_end) // 2

            # 실제 각도로 변환 (front_ranges에서의 인덱스를 전체 각도로 변환)
            gap_angle_idx = best_gap_local_idx + front_start
            gap_angle = angle_min + gap_angle_idx * angle_increment

            # 조향각 계산 (중앙(0도)에서 얼마나 떨어져 있는지)
            center_angle = (angle_min + angle_max) / 2
            steering_angle = gap_angle - center_angle

            # 조향각 제한
            steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

            # 회전각에 따른 속도 조절 (선형 감소)
            speed = self.max_speed - (abs(steering_angle) / self.max_steering_angle) * (self.max_speed - self.min_speed)

        # 전방 장애물이 너무 가까우면 추가 감속
        if min_dist < self.safe_distance / 2:
            speed = min(speed, self.min_speed * 0.7) # 더 강하게 감속

        # 너무 느리면 차가 앞으로 나가지 않음
        if speed < 1:
            speed = 1

        # 최종 제어 명령 발행
        self.publish_drive(steering_angle, speed)

        # 로깅 (더 자세한 정보)
        self.get_logger().info(
            f'Min dist: {min_dist:.2f}m, '
            f'Front Range [{front_start}:{front_end}], '
            f'Gap Local [{max_gap_start}:{max_gap_end}], '
            f'Gap Global Index: {max_gap_start + front_start}:{max_gap_end + front_start}, '
            f'Steering: {steering_angle:.2f}, Speed: {speed:.2f}'
        )

    def find_largest_gap(self, ranges):
        """ 가장 큰 연속된 빈 공간 (gap) 찾기 """
        masked = ranges > 0.0  # 안전 거리 이상이면 True

        # 연속된 True 구간 찾기
        gaps = np.diff(np.concatenate(([False], masked, [False])))
        gap_starts = np.where(gaps)[0][::2]
        gap_ends = np.where(gaps)[0][1::2]

        if not gap_starts.size:
            return 0, len(ranges) - 1 if ranges.size > 0 else (0, 0)

        # gap 크기 계산하고 가장 큰 gap 찾기
        gap_sizes = gap_ends - gap_starts
        largest_gap_idx = np.argmax(gap_sizes)

        return gap_starts[largest_gap_idx], gap_ends[largest_gap_idx]

    def publish_drive(self, steering_angle, speed):
        """ 차량 제어 명령 발행 """
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"  # 적절한 frame_id 설정
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapImproved()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
