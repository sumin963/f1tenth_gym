class wallPlanner:

    def __init__(self, params, robot_scale=0.3302):
        self.parms = params
        self.robot_scale = robot_scale

    def plan(self, obs):
        ranges = obs['scans']
        front = ranges[540]
        left = ranges[270]
        right = ranges[810]

        if left >= right:
            steering_angle = -0.2
            speed = 3
        elif left < right:
            steering_angle = 0.2
            speed = 3

        return speed, steering_angle
