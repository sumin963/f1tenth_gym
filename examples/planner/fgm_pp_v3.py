import numpy as np
import math
import threading
from queue import Queue
import time
from multiprocessing import shared_memory, Lock



from planner.sub_planner.fgm_shm import FGM
from planner.sub_planner.pp_shm import PP
#from planner.sub_planner.speed_controller import SpeedController as SC

class FGM_PP_V3:
    def __init__(self, params):
        self.params = params
        self.RACECAR_LENGTH = params.robot_length
        self.ROBOT_LENGTH = params.robot_length
        self.SPEED_MAX = params.max_speed
        self.SPEED_MIN = params.min_speed

        self.MU = params.mu
        self.GRAVITY_ACC = params.g
        self.PI = params.pi
        self.ROBOT_SCALE = params.robot_scale
        self.waypoint_real_path = params.wpt_path
        self.wp_num = 1
        self.waypoint_delimeter = params.wpt_delimeter
        self.waypoints = self.get_waypoint()

        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.nearest_distance = 0
        self.lookahead_desired = 0
        self.desired_point = 0
        self.actual_lookahead = 0
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.transformed_desired_point = [0]*3

        self.steering_direction = 0
        self.goal_path_radius = 0
        self.goal_path_theta = 0

        self.current_speed = 5.0
        self.scan_filtered = [0]*1080
        self.scan_origin = []
        self.scan_range = 0

        self.radians_per_elem = 0
        self.PREPROCESS_CONV_SIZE = 3
        self.BEST_POINT_CONV_SIZE = 80
        self.MAX_LIDAR_DIST = 3000000

        self.scan_obs = []
        self.dect_obs = []
        self.len_obs = []
        self.obs = False
        self.past_obs = False
        self.obs_y = 0

        self.speed = 0
        self.steer = 0

        self.lock = Lock()
        self.current_position_shm = shared_memory.ShareableList(self.current_position)
        self.current_speed_shm = shared_memory.ShareableList([self.current_speed])
        self.scan_filtered_shm = shared_memory.ShareableList(self.scan_filtered)
        self.transformed_desired_point_shm = shared_memory.ShareableList(self.transformed_desired_point)
        self.steer_global_shm = shared_memory.ShareableList([self.steer])
        self.steer_local_shm = shared_memory.ShareableList([self.steer])
        self.sp_shm = shared_memory.ShareableList([0])

        # self.main_global_q = Queue(1)
        # self.global_main_q = Queue(1)
        #
        # self.main_local_q = Queue(1)
        # self.local_main_q = Queue(1)
        #self.speed_control = SC(params)

        self.global_t = PP(self.current_position_shm, self.current_speed_shm, self.transformed_desired_point_shm, self.steer_global_shm, self.lock, self.sp_shm)
        self.local_t = FGM(self.current_position_shm, self.current_speed_shm, self.scan_filtered_shm, self.transformed_desired_point_shm, self.steer_local_shm, self.lock, self.sp_shm)

        self.global_t.start()
        self.local_t.start()

    def get_waypoint(self):
        file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.waypoint_delimeter, dtype='float')

        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0], i[1], 0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        # print("wp_num",self.wp_num)
        return temp_waypoint

    def find_nearest_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        while True:
            wp_index_temp += 1
            if wp_index_temp >= len(self.waypoints) - 1:
                wp_index_temp = 0

            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif temp_distance > (self.nearest_distance + self.CURRENT_WP_CHECK_OFFSET) or (
                    wp_index_temp == self.wp_index_current):
                break

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[self.wp_index_current])
        if (transformed_nearest_point[0] < 0): self.nearest_distance *= -1

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        while (1):
            if (wp_index_temp >= len(self.waypoints) - 1): wp_index_temp = 0
            distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)
            if distance >= self.lookahead_desired:
                if wp_index_temp - 2 >= 0 and wp_index_temp + 2 < len(self.waypoints) - 1:
                    self.waypoints[wp_index_temp][2] = np.arctan(
                        (self.waypoints[wp_index_temp + 2][1] - self.waypoints[wp_index_temp - 2][1]) /
                        self.waypoints[wp_index_temp + 2][0] - self.waypoints[wp_index_temp - 2][0])
                self.desired_point = self.waypoints[wp_index_temp]
                self.actual_lookahead = distance
                break
            wp_index_temp += 1

    def get_lookahead_desired(self):
        _vel = self.current_speed
        # self.lookahead_desired = 0.5 + (0.3 * _vel)
        self.lookahead_desired = 0.5 + (0.3 * _vel)

    def getDistance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return np.sqrt(dx ** 2 + dy ** 2)

    def transformPoint(self, origin, target):
        theta = self.PI / 2 - origin[2]

        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dtheta = target[2] + theta

        tf_point_x = dx * np.cos(theta) - dy * np.sin(theta)
        tf_point_y = dx * np.sin(theta) + dy * np.cos(theta)
        tf_point_theta = dtheta
        tf_point = [tf_point_x, tf_point_y, tf_point_theta]

        return tf_point

    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        # rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x * x + y * y))
        rtpoint.append(np.arctan2(y, x) - (self.PI / 2))
        return rtpoint

    def preprocess_lidar(self, scan_data):
        self.scan_range = len(scan_data)
        self.scan_filtered = [0] * self.scan_range
        self.scan_origin = [0] * self.scan_range
        for i in range(self.scan_range):
            self.scan_filtered[i] = scan_data[i]
            self.scan_origin[i] = scan_data[i]
        self.radians_per_elem = 0.00435
        proc_ranges = np.convolve(scan_data, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def obs_dect(self):
        # for i in range(1, self.scan_range - 1):
        self.scan_obs = []
        i = 1
        d_group = 2
        d_pi = 0.00628
        while (self.scan_range - 1 > i):
            start_idx_temp = i
            end_idx_temp = i
            max_idx_temp = i
            min_idx_temp = i
            i = i + 1
            while math.sqrt(math.pow(self.scan_origin[i] * math.sin(math.radians(0.25)), 2) + math.pow(
                    self.scan_origin[i - 1] - self.scan_origin[i] * math.cos(math.radians(0.25)), 2)) < d_group + \
                    self.scan_origin[i] * d_pi and (i + 1 < self.scan_range):
                if self.scan_origin[i] > self.scan_origin[max_idx_temp]:
                    max_idx_temp = i
                if self.scan_origin[i] < self.scan_origin[min_idx_temp]:
                    min_idx_temp = i
                i = i + 1
            end_idx_temp = i - 1
            obs_temp = [0] * 6
            obs_temp[0] = start_idx_temp
            obs_temp[1] = end_idx_temp
            obs_temp[2] = max_idx_temp
            obs_temp[3] = min_idx_temp
            obs_temp[4] = self.scan_origin[max_idx_temp]
            obs_temp[5] = self.scan_origin[min_idx_temp]
            self.scan_obs.append(obs_temp)
            i += 1

        self.dect_obs = []
        for i in range(len(self.scan_obs)):
            if self.scan_obs[i][5] < 4 and self.scan_obs[i][5] > 0:
                obs_temp = [0] * 6
                obs_temp[0] = self.scan_obs[i][0]
                obs_temp[1] = self.scan_obs[i][1]
                obs_temp[2] = self.scan_obs[i][2]
                obs_temp[3] = self.scan_obs[i][3]
                obs_temp[4] = self.scan_obs[i][4]
                obs_temp[5] = self.scan_obs[i][5]
                self.dect_obs.append(obs_temp)
        # print(self.dect_obs)
        self.len_obs = []

        for i in range(len(self.dect_obs)):
            theta = (self.dect_obs[i][1] - self.dect_obs[i][0]) * 0.25
            lengh = math.sqrt(
                math.pow(self.scan_origin[self.dect_obs[i][1]] * math.sin(math.radians(theta)), 2) + math.pow(
                    self.scan_origin[self.dect_obs[i][0]] - self.scan_origin[self.dect_obs[i][1]] * math.cos(
                        math.radians(theta)), 2))
            # print(i,lengh)
            if lengh < 1 and lengh > 0:
                obs_temp = [0] * 6
                obs_temp[0] = self.dect_obs[i][0]
                obs_temp[1] = self.dect_obs[i][1]
                obs_temp[2] = self.dect_obs[i][2]
                obs_temp[3] = self.dect_obs[i][3]
                obs_temp[4] = self.dect_obs[i][4]
                obs_temp[5] = self.dect_obs[i][5]
                self.len_obs.append(obs_temp)

        self.obs = False
        for i in range(len(self.len_obs)):
            if self.len_obs[i][0] > 680 or self.len_obs[i][1] < 400:
                # print(self.len_obs)
                self.obs = False
            else:
                # print("cnt_obs : ", len(self.len_obs))
                self.obs = True
                # theta = (int((self.len_obs[i][1] + self.len_obs[i][0]) / 2) - 180) * (4.7 / 1080)
                # # print(theta)
                # _x = self.len_obs[i][5] * np.cos(theta)
                # _y = self.len_obs[i][5] * np.sin(theta)
                # print(f"x: {_x}, y: {_y}")
                break

    def speed_controller(self):
        current_distance = np.fabs(np.average(self.scan_filtered[499:580]))
        if np.isnan(current_distance):
            print("SCAN ERR")
            current_distance = 1.0

        if self.current_speed > 10:
            current_distance -= self.current_speed * 0.7

        maximum_speed = np.sqrt(2 * self.MU * self.GRAVITY_ACC * current_distance) - 2

        if maximum_speed >= self.SPEED_MAX:
            maximum_speed = self.SPEED_MAX

        if self.current_speed <= maximum_speed:
            # ACC
            if self.current_speed >= 10:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * 0.8)
            else:
                set_speed = self.current_speed + np.fabs((maximum_speed - self.current_speed) * self.ROBOT_LENGTH)
        else:
            # set_speed = 0
            set_speed = self.current_speed - np.fabs((maximum_speed - self.current_speed) * 0.2)
        # print("speed :", set_speed, "current", maximum_speed)
        return set_speed


    def driving(self, scan_data, odom_data):

        """
        :param scan_data: scan data
        :param odom_data: odom data
        :return: steer, speed
        """
        self.current_position = [odom_data['x'], odom_data['y'], odom_data['theta']]
        self.current_speed = odom_data['linear_vel']
        self.scan_origin = scan_data

        self.scan_filtered = self.preprocess_lidar(scan_data)

        self.find_nearest_wp()
        self.get_lookahead_desired()
        self.find_desired_wp()
        self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)

        with self.lock:
            # self.current_position_shm = self.current_position.copy[:]
            # self.transformed_desired_point_shm =  self.transformed_desired_point.copy[:]
            # self.scan_filtered_shm =self.scan_filtered

            for i in range(len(self.current_position)):
                self.current_position_shm[i] = float(self.current_position[i])
                self.transformed_desired_point_shm[i] =  float(self.transformed_desired_point[i])
            for i in range(len(self.scan_filtered)):
                self.scan_filtered_shm[i] = float(self.scan_filtered[i])
            self.current_speed_shm[0] = float(self.current_speed)
            self.sp_shm[0] = float(1)

        self.obs_dect()
        # if self.obs:
        #     print(self.obs)
        # self.obs = True


        if self.obs:
            self.steer = self.steer_local_shm[0]
        else:
            self.steer = self.steer_global_shm[0]

        self.speed = self.speed_controller()
        #print(self.speed, self.steer)
        return self.speed, self.steer
