import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
        
        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # 차량 파라미터
        self.max_steering_angle = 0.85 * 2     # 최대 조향각 (라디안)
        self.max_speed = 0.75               # 최대 속도 (m/s)
        self.safe_distance = 0.7           # 안전 거리 (m)

        self.get_logger().info('start!')
        
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angle_min, angle_max = msg.angle_min, msg.angle_max
        angle_increment = msg.angle_increment
        
        # 장애물 데이터 필터링 (레이저 데이터 중 0 값 제거)
        # valid_ranges = np.where((ranges > msg.range_min) & (ranges < msg.range_max), ranges, msg.range_max)
        ranges[ranges < msg.range_min] = msg.range_min
        ranges[ranges > msg.range_max] = msg.range_max
        valid_ranges = ranges

        side_dist_1 = np.mean(valid_ranges[len(valid_ranges) * 1 // 8 - 2:len(valid_ranges) * 1 // 8 + 2])
        side_dist_2 = np.mean(valid_ranges[len(valid_ranges) * 2 // 8 - 2:len(valid_ranges) * 2 // 8 + 2])
        side_dist_3 = np.mean(valid_ranges[len(valid_ranges) * 3 // 8 - 2:len(valid_ranges) * 3 // 8 + 2])
        side_dist_4 = np.mean(valid_ranges[len(valid_ranges) * 4 // 8 - 2:len(valid_ranges) * 4 // 8 + 2])
        side_dist_5 = np.mean(valid_ranges[len(valid_ranges) * 5 // 8 - 2:len(valid_ranges) * 5 // 8 + 2])
        side_dist_6 = np.mean(valid_ranges[len(valid_ranges) * 6 // 8 - 2:len(valid_ranges) * 6 // 8 + 2])
        side_dist_7 = np.mean(valid_ranges[len(valid_ranges) * 7 // 8 - 2:len(valid_ranges) * 7 // 4 + 2])
        # self.get_logger().info(f"left_side_dist: {left_side_dist} m, right_side_dist: {right_side_dist} m")
        self.get_logger().info(f"side_dist_1: {side_dist_1} m")
        self.get_logger().info(f"side_dist_2: {side_dist_2} m")
        self.get_logger().info(f"side_dist_3: {side_dist_3} m")
        self.get_logger().info(f"side_dist_4: {side_dist_4} m")
        self.get_logger().info(f"side_dist_5: {side_dist_5} m")
        self.get_logger().info(f"side_dist_6: {side_dist_6} m")
        self.get_logger().info(f"side_dist_7: {side_dist_7} m")

        # # 좌우 90도를 기준으로 뒷부분은 0(=장애물 있음)으로 설정
        # masked_ranges = np.copy(valid_ranges)
        # # masked_ranges[0:(len(ranges) // 4)] = 0
        # # masked_ranges[(len(ranges) * 3 // 4):(len(ranges) - 1)] = 0

        # # 장애물이 있는 곳의 값을 0으로 설정
        # masked_ranges[masked_ranges < self.safe_distance] = 0
        
        # # 가장 큰 연속된 gap(빈 공간) 찾기
        # max_gap_start, max_gap_end = self.find_largest_gap(masked_ranges)
        
        # # 가장 큰 gap 중앙의 방향 계산
        # best_index = (max_gap_start + max_gap_end) // 2
        # best_angle = angle_min + best_index * angle_increment
        # self.get_logger().info(f'best angle: {best_angle * 180 / np.pi}')
        
        # 조향값 및 속도 설정
        # best_angle = 0.0 if abs(best_angle) > np.pi / 2 else best_angle # 목표하는 방향은 뒤쪽이 아니어야 함
        # steering_angle = np.clip(best_angle, -self.max_steering_angle, self.max_steering_angle)
        
        steering_angle = 0.0
        if (side_dist_2 - side_dist_6) > 0.25:
            steering_angle = -self.max_steering_angle # 오른쪽으로 조향
        elif (side_dist_2 - side_dist_6) < -0.25:
            steering_angle = self.max_steering_angle # 왼쪽으로 조향
            
        speed = self.max_speed
        speed = self.max_speed if abs(steering_angle) < 0.2 else self.max_speed - 0.25  # 회전 시 속도 감소

        # 제어 명령 발행
        if abs(steering_angle / 2) > 0.15: # min_steering_angle
            self.publish_drive(-steering_angle / 2, speed)
        else:
            self.publish_drive(0.0, speed)
    
    def find_largest_gap(self, ranges):
        """ 가장 큰 연속된 빈 공간 (gap) 찾기 """
        masked = ranges > 0 # 장애물이 없으면 True = 1
        gaps = np.diff(np.concatenate(([0], masked.astype(int), [0])))  # 경계점 찾기
        gap_starts = np.where(gaps == 1)[0] # 장애물 X<-O (1 - 0 =  1)
        gap_ends = np.where(gaps == -1)[0]  # 장애물 O<-X (0 - 1 = -1)

        self.get_logger().info(str(gap_starts))
        self.get_logger().info(str(gap_ends))
        self.get_logger().info(str(gap_ends - gap_starts))
        
        if len(gap_starts) == 0 or len(gap_ends) == 0:
            return 0, len(ranges) - 1  # 장애물이 없으면 전체 사용
        
        largest_gap = np.argmax(gap_ends - gap_starts)
        return gap_starts[largest_gap], gap_ends[largest_gap]
    
    def publish_drive(self, steering_angle, speed):
        """ 차량 제어 명령 발행 """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        # drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)
        
        self.get_logger().info(f'Steering: {steering_angle:.2f}, Speed: {speed:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
