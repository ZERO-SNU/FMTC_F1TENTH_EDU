import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class TestLiDAR(Node):
    def __init__(self):
        super().__init__('test_lidar')
        
        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info('start!')
        
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0)  # NaN 값 처리
        angle_min, angle_max = msg.angle_min, msg.angle_max
        angle_inc = msg.angle_increment
        # range_max = msg.range_max
        range_max = 2.0
        num_points = len(ranges)
        
        # ASCII 그래프 크기 설정
        size = 26  # 원형 그래프 크기 (터미널에서 조정 가능)
        canvas = [[' ' for _ in range(2 * size + 1)] for _ in range(2 * size + 1)]

        # LIDAR 데이터 변환
        num_points = len(ranges)
        for i in range(num_points):
            # angle = angle_min + i * angle_inc  # 라디안 단위
            angle = i * angle_inc  # 라디안 단위, 0도부터 시작
            distance = ranges[i]

            # 거리를 화면 크기에 맞게 정규화
            norm_dist = min(distance / range_max, 1.0) * size

            # 좌표 변환 (정면부터 시작, 시계 방향으로 진행)
            x = int(size - norm_dist * math.cos(angle))
            y = int(size - norm_dist * math.sin(angle))

            # 범위 내에 있는지 확인
            if 0 <= x < 2 * size - 1 and 0 <= y < 2 * size - 1:
                canvas[x][y] = '*'

        # 거리 추가
        for i in range(1, int(range_max + 1)):
            pos = int(size * i / range_max)
            canvas[size][size + pos] = str(i % 10)
            if i // 10 != 0:
                canvas[size - pos][size - 1] = str(i // 10)
            canvas[size - pos][size] = str(i % 10)
        
        # 중심점 추가
        canvas[size][size] = 'O'
        canvas[size][size - 1] = 'y'
        canvas[size - 1][size] = 'x'

        # 출력
        for row in canvas:
            print(''.join(row))
    
    @staticmethod
    def clear_screen():
        print("\033[H\033[J", end="")  # ANSI escape 코드로 화면 클리어


def main(args=None):
    rclpy.init(args=args)
    node = TestLiDAR()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
