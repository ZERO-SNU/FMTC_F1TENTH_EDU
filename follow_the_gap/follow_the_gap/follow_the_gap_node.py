#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class FollowTheGapImproved(Node):
    def __init__(self):
        super().__init__('follow_the_gap_ref')

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

        # TODO: 유효하지 않은 값 처리 (NaN, Inf)
        # HINT: np.nan_to_num() 함수를 사용하면 유용함

        # TODO: 유효한 범위로 필터링
        # HINT: np.clip() 함수를 사용하면 유용함

        # TODO: 전방 인덱스 계산 (중앙 기준으로 ±self.front_angle_range / 2)

        # TODO: 전방 데이터 추출

        # TODO: 가장 가까운 장애물 위치 파악

        # TODO: 장애물 주변에 안전 영역(bubble) 설정

        # TODO: 가장 가까운 장애물 주변에 버블 적용

        # TODO: Gap 찾기 (safe_distance 이상의 거리)

        # TODO: 가장 큰 gap 찾기

        # TODO: gap이 너무 작으면 좌우 측면 거리를 비교하여 더 넓은 쪽으로 이동
        #       그렇지 않다면
        #       1. gap의 중앙 인덱스 계산 (전방 범위 내)
        #       2. 실제 각도로 변환 (front_ranges에서의 인덱스를 전체 각도로 변환)
        #       3. 조향각 계산 (중앙(0도)에서 얼마나 떨어져 있는지)
        #       4. 계산된 조향각을 실제 자동차의 조향 한계에 맞춰 제한
        #       5. (필수 X) 회전각에 따른 속도 조절 (선형 감소)
        steering_angle = 0.0
        speed = 1.0

        # TODO: 전방 장애물이 너무 가까우면 추가 감속
        #       (필수 X)

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
        # TODO: 안전 거리 이상이면 True를 반환

        # TODO: 연속된 True 구간 찾기

        # TODO: gap 크기 계산하고 가장 큰 gap 찾기

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
