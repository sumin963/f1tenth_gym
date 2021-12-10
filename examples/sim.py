import gym
import numpy as np
import yaml
import time
from concurrent.futures import ThreadPoolExecutor

from argparse import Namespace
from pyglet.gl import GL_POINTS

from planner import fgm_convolution as fc
from planner import wall_following as wf
from planner import pure_pursuit as pp
from planner import fgm_gnu as fg

class EnvProcess:
    def __init__(self, conf, planners):
        self.conf = conf
        self.planners = planners
        self.env = gym.make('f110_gym:f110-v0', map=self.conf.map_path, map_ext=self.conf.map_ext,
                            num_agents=len(self.planners))

        if self.conf.render_waypoints:
            self.drawn_waypoints = list()
            self.waypoints = np.loadtxt(self.conf.wpt_path, delimiter=self.conf.wpt_delim,
                                        skiprows=self.conf.wpt_rowskip)
            self.env.add_render_callback(self._render_callback)

        self.obs, self.step_reward, self.done, self.info = self.reset()

    # 시뮬레이터 실행 시 맵에 웨이포이트를 표시하거나 첫 번째 Agent를 기준으로 렌더링 하는 함수입니다.
    def _render_callback(self, e):
        if self.conf.render_center:
            x = e.cars[0].vertices[::2]
            y = e.cars[0].vertices[1::2]
            top, bottom, left, right = max(y), min(y), min(x), max(x)
            e.score_label.x = left
            e.score_label.y = top - 700
            e.left = left - 800
            e.right = right + 800
            e.top = top + 800
            e.bottom = bottom - 800

        if self.conf.render_waypoints:
            points = np.vstack((self.waypoints[:, self.conf.wpt_xind], self.waypoints[:, self.conf.wpt_yind])).T
            scaled_points = 50. * points
            for i in range(points.shape[0]):
                if len(self.drawn_waypoints) < points.shape[0]:
                    b = e.batch.add(1, GL_POINTS, None, ('v3f/stream', [scaled_points[i, 0], scaled_points[i, 1], 0.]),
                                    ('c3B/stream', [183, 193, 222]))
                    self.drawn_waypoints.append(b)
                else:
                    self.drawn_waypoints[i].vertices = [scaled_points[i, 0], scaled_points[i, 1], 0.]

    # index를 받아 각 자동차의 상태를 포함하는 Observation을 리패키징 하여 반환하는 함수입니다.
    def _pack_obs(self, i):
        obs = {
            'scans': self.obs['scans'][i],
            'poses_x': self.obs['poses_x'][i],
            'poses_y': self.obs['poses_y'][i],
            'poses_theta': self.obs['poses_theta'][i],
            'linear_vels_x': self.obs['linear_vels_x'][i],
            'linear_vels_y': self.obs['linear_vels_y'][i],
            'ang_vels_z': self.obs['ang_vels_z'][i],
            'collisions': self.obs['collisions'][i],
            'lap_times': self.obs['lap_times'][i],
            'lap_counts': self.obs['lap_counts'][i]
        }
        return obs

    # 시뮬레이터를 초기화하는 함수 입니다.
    def reset(self):
        obj = []
        for i in range(1, len(self.planners) + 1):
            if hasattr(self.conf, f'p{i}'):
                p_obj = getattr(self.conf, f'p{i}')
                obj.append([p_obj['sx'], p_obj['sy'], p_obj['stheta']])

        return self.env.reset(np.array(obj))

    # 시뮬레이터를 한 스텝 실행하는 함수 입니다.
    def step(self, actions):
        actions = np.array(actions)
        self.obs, self.step_reward, self.done, self.info = self.env.step(actions)

    # 시뮬레이터를 렌더링 하는 함수입니다.
    def render(self):
        self.env.render(mode=self.conf.render_mode)

    # 환경의 메인 함수입니다.
    def main(self):
        self.render()
        start = time.time()
        laptime = 0.0
        while self.done == False or isinstance(self.done, float):
            actions = []
            futures = []
            with ThreadPoolExecutor() as executor:  # Parallelize the planning
                for i, p in enumerate(self.planners):
                    if hasattr(p, 'plan'):
                        futures.append(executor.submit(p.plan, self._pack_obs(i)))
                    elif hasattr(p, 'driving'):
                        futures.append(executor.submit(p.driving, self._pack_obs(i)))

            for future in futures:
                speed, steer = future.result()  # Wait for the result
                actions.append([steer, speed])

            self.step(actions)  # Step the simulator
            self.render()
            laptime += self.step_reward

        print(f'Sim elapsed time: {laptime} Real elapsed time: {time.time() - start}')


if __name__ == "__main__":
    with open('params.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)

    # planners = [fc.FgPlanner(conf, 0.3302), pp.PurePursuitPlanner(conf, 0.3302), wf.WallPlanner(conf, 0.3302)]
    # planners = [fc.FgPlanner(conf, 0.3302), pp.PurePursuitPlanner(conf, 0.3302)]
    planners = [fg.FGMPlanner(conf, 0.3302)]
    env_process = EnvProcess(conf, planners)
    env_process.main()
