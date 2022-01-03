import numpy as np
import math
import time
#from .sub_planner.speed_controller import SpeedController as SC

class FGM_PP_V1:
    def __init__(self, conf, robot_scale=0.3302):
        self.conf = conf
        self.SPEED_MAX = self.conf.max_speed
        self.SPEED_MIN = self.conf.min_speed

        self.PI = 3.141592

        self.RACECAR_LENGTH = 0.325
        self.ROBOT_SCALE = robot_scale
        self.waypoint_real_path = self.conf.wpt_path
        self.wp_num = 1
        self.waypoint_delimeter = self.conf.wpt_delim
        self.waypoints = self.get_waypoint()
        self.max_angle = 0

        self.wp_index_current = 0
        self.current_position = [0] * 3
        self.nearest_distance = 0
        self.lookahead_desired = 0
        self.desired_point = 0
        self.actual_lookahead = 0
        self.CURRENT_WP_CHECK_OFFSET = 2
        self.transformed_desired_point = 0

        self.steering_direction = 0
        self.goal_path_radius = 0
        self.goal_path_theta = 0

        self.current_speed = 5.0
        self.scan_filtered = []
        self.scan_range = 0

        self.radians_per_elem = 0
        self.PREPROCESS_CONV_SIZE = 100
        self.BEST_POINT_CONV_SIZE = 80
        self.MAX_LIDAR_DIST = 3000000

        self.scan_obs = []
        self.dect_obs = []
        self.len_obs = []
        self.obs = False
        self.obs_split = []

        self.BUBBLE_RADIUS = 160
        #self.speed_control = SC(params)


    def get_waypoint(self):
        file_wps = np.loadtxt(self.conf.wpt_path, delimiter=self.conf.wpt_delim,
                              skiprows=self.conf.wpt_rowskip)
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[self.conf.wpt_xind], i[self.conf.wpt_yind], 0]
            temp_waypoint.append(wps_point)

        return temp_waypoint
        # file_wps = np.genfromtxt(self.waypoint_real_path, delimiter=self.waypoint_delimeter, dtype='float')
        #
        # temp_waypoint = []
        # for i in file_wps:
        #     wps_point = [i[0], i[1], 0]
        #     temp_waypoint.append(wps_point)
        #     self.wp_num += 1
        # # print("wp_num",self.wp_num)
        # return temp_waypoint

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
        # _vel = self.current_speed
        # self.lookahead_desired = 0.5 + (0.3 * _vel)
        self.lookahead_desired = 2#0.5 + (0.1 * _vel)

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
        for i in range(self.scan_range):
            self.scan_filtered[i] = scan_data[i]
        self.radians_per_elem = 0.00435
        proc_ranges = np.convolve(scan_data, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def obs_dect(self):
        #for i in range(1, self.scan_range - 1):
        self.scan_obs = []
        i=1
        d_group = 1.5
        d_pi = 0.00628
        while(self.scan_range - 1>i):
            start_idx_temp = i
            end_idx_temp = i
            max_idx_temp = i
            min_idx_temp = i
            i = i+1
            while  math.sqrt(math.pow(self.scan_filtered[i]*math.sin(math.radians(0.25)),2) + math.pow(self.scan_filtered[i-1]-self.scan_filtered[i]*math.cos(math.radians(0.25)),2)) < d_group + self.scan_filtered[i]*d_pi and (i+1 < self.scan_range ):
                if self.scan_filtered[i] > self.scan_filtered[max_idx_temp]:
                    max_idx_temp = i
                if self.scan_filtered[i] < self.scan_filtered[min_idx_temp]:
                    min_idx_temp = i
                i = i+1
            end_idx_temp = i-1
            obs_temp = [0]*5
            obs_temp[0] = start_idx_temp
            obs_temp[1] = end_idx_temp
            obs_temp[2] = max_idx_temp
            obs_temp[3] = self.scan_filtered[max_idx_temp]
            obs_temp[4] = self.scan_filtered[min_idx_temp]
            self.scan_obs.append(obs_temp)
            i+=1

        self.obs_split = []
        for i in range(len(self.scan_obs)):
            if self.scan_obs[i][3] < 4 and self.scan_obs[i][3] > 0:
                obs_temp = [0] * 5
                obs_temp[0] = self.scan_obs[i][0]
                obs_temp[1] = self.scan_obs[i][1]
                obs_temp[2] = self.scan_obs[i][2]
                obs_temp[3] = self.scan_obs[i][3]
                obs_temp[4] = self.scan_obs[i][4]
                self.obs_split.append(obs_temp)
        # print(self.dect_obs)


        self.dect_obs=[]
        for i in range(len(self.scan_obs)):
            if self.scan_obs[i][3] < 4 and self.scan_obs[i][3] > 0:
                obs_temp = [0]*5
                obs_temp[0] = self.scan_obs[i][0]
                obs_temp[1] = self.scan_obs[i][1]
                obs_temp[2] = self.scan_obs[i][2]
                obs_temp[3] = self.scan_obs[i][3]
                obs_temp[4] = self.scan_obs[i][4]
                self.dect_obs.append(obs_temp)
        #print(self.dect_obs)
        self.len_obs=[]

        for i in range(len(self.dect_obs)):
            theta = (self.dect_obs[i][1] - self.dect_obs[i][0])*0.25
            lengh = math.sqrt(math.pow(self.scan_filtered[self.dect_obs[i][1]]*math.sin(math.radians(theta)),2) + math.pow(self.scan_filtered[self.dect_obs[i][0]]-self.scan_filtered[self.dect_obs[i][1]]*math.cos(math.radians(theta)),2))
            #print(i,lengh)
            if lengh < 1:
                obs_temp = [0]*5
                obs_temp[0] = self.dect_obs[i][0]
                obs_temp[1] = self.dect_obs[i][1]
                obs_temp[2] = self.dect_obs[i][2]
                obs_temp[3] = self.dect_obs[i][3]
                obs_temp[4] = self.dect_obs[i][4]
                self.len_obs.append(obs_temp)
        #print(self.len_obs)

        self.obs = False
        for i in range(len(self.len_obs)):
            if self.len_obs[i][0] > 680 or self.len_obs[i][1] < 400:
                self.obs = False
            else:
                print(True)
                self.obs= True
                break

    def find_path(self):
        #right cornering
        if self.transformed_desired_point[0] > 0:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/(2*self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1]/self.goal_path_radius)
            self.steering_direction = -1

        #left cornering
        else:
            self.goal_path_radius = pow(self.actual_lookahead, 2)/((-2)*self.transformed_desired_point[0])
            self.goal_path_theta = np.arcsin(self.transformed_desired_point[1]/self.goal_path_radius)
            self.steering_direction = 1

    def setSteeringAngle(self):
         steering_angle = np.arctan2(self.RACECAR_LENGTH, self.goal_path_radius)
         steer = self.steering_direction * steering_angle
         return steer

    def find_max_gap(self, free_space_ranges):
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        # I think we will only ever have a maximum of 2 slices but will handle an
        # indefinitely sized list for portablility
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
            # do a sliding window average over the data in the max gap, this will
            # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

    def conv_fgm(self):
        proc_ranges = self.scan_filtered[135:-135]
        closest = proc_ranges.argmin()

        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Publish Drive message
        steering_angle = self.get_angle(best, len(proc_ranges))
        return  steering_angle


    def driving(self, obs):
        """

        :param scan_data: scan data
        :param odom_data: odom data
        :return: steer, speed
        """

        self.current_position = np.array([obs['poses_x'], obs['poses_y'], obs['poses_theta']])
        self.scan_filtered = self.preprocess_lidar(obs['scans'])

        self.find_nearest_wp()
        self.get_lookahead_desired()
        self.find_desired_wp()
        self.transformed_desired_point = self.transformPoint(self.current_position, self.desired_point)
        self.obs_dect()
        # if self.obs:
        #     print(self.obs)
        self.obs = True

        if self.obs:
            steer = self.conv_fgm()
            # print("True")
        else:
            self.find_path()
            steer = self.setSteeringAngle()

        if np.fabs(steer) > self.PI / 8:
            speed = self.SPEED_MIN

        else:
            angle_control = self.max_angle * 4.25
            if np.fabs(angle_control) > 1:
                angle_control = 1
            speed = float(
                -(3 / self.PI) * (self.SPEED_MAX - self.SPEED_MIN) * np.fabs(angle_control) + self.SPEED_MAX)
            speed = np.fabs(speed)

        if self.SPEED_MAX < speed or speed < self.SPEED_MIN:
            print(speed, angle_control)

        #print(speed, steer)
        #speed = self.speed_control.routine(self.scan_filtered,self.current_speed,steer,self.wp_index_current)
        return speed, steer
