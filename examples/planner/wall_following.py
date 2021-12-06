import numpy as np

class wallPlanner:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 100  # PREPROCESS_consecutive_SIZE
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    def init(self, params, robot_scale):
        self.robot_scale = robot_scale
        self.radians_per_elem = None
        self.STRAIGHTS_SPEED = params.max_speed
        self.CORNERS_SPEED = params.min_speed


    def plan(self, scan_data, odom_data):

        ranges = scan_data['ranges']
        front = ranges[540]
        left = ranges[270]
        right = ranges[810]
        speed = 10

        if left >= right:
            steering_angle = -0.2
            speed = 10
        elif left < right:
            steering_angle = 0.2
            speed = 10

        return speed, steering_angle
