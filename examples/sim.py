import gym
import numpy as np
import yaml
import time

from argparse import Namespace
from pyglet.gl import GL_POINTS

from planner import waypoint_follow as wf
from planner import fgm_convolution as fc


def main():
    """
    main entry point
    """

    work = {'mass': 3.463388126201571, 'lf': 0.15597534362552312, 'tlad': 0.82461887897713965,
            'vgain': 0.90338203837889}

    with open('params.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)

    wfp = wf.PurePursuitPlanner(conf, 0.17145 + 0.15875)
    fgp = fc.FgPlanner(conf, 0.17145 + 0.15875)

    # planner = fgp
    planner = wfp

    waypoints = np.loadtxt(conf.wpt_path, delimiter=conf.wpt_delim, skiprows=conf.wpt_rowskip)
    drawn_waypoints = list()

    def render_waypoints(e):
        """
        update waypoints being drawn by EnvRenderer
        """
        points = np.vstack((waypoints[:, conf.wpt_xind], waypoints[:, conf.wpt_yind])).T

        scaled_points = 50. * points

        for i in range(points.shape[0]):
            if len(drawn_waypoints) < points.shape[0]:
                b = e.batch.add(1, GL_POINTS, None, ('v3f/stream', [scaled_points[i, 0], scaled_points[i, 1], 0.]),
                                ('c3B/stream', [183, 193, 222]))
                drawn_waypoints.append(b)
            else:
                drawn_waypoints[i].vertices = [scaled_points[i, 0], scaled_points[i, 1], 0.]

    def render_callback(env_renderer):
        e = env_renderer
        # update camera to follow car
        x = e.cars[0].vertices[::2]
        y = e.cars[0].vertices[1::2]
        top, bottom, left, right = max(y), min(y), min(x), max(x)
        e.score_label.x = left
        e.score_label.y = top - 700
        e.left = left - 800
        e.right = right + 800
        e.top = top + 800
        e.bottom = bottom - 800

        render_waypoints(env_renderer)

    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1)
    env.add_render_callback(render_callback)

    obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))
    env.render()

    laptime = 0.0
    start = time.time()

    while not done:
        scan_data = {'ranges': obs['scans'][0]}
        odom_data = {
            'pose_x': obs['poses_x'][0],
            'pose_y': obs['poses_y'][0],
            'pose_theta': obs['poses_theta'][0],
            'linear_vels_x': obs['linear_vels_x'],
            'linear_vels_y': obs['linear_vels_y'],
            'angle_vels_z': obs['ang_vels_z'],
            'lookahead_distance': work['tlad'],
            'vgain': work['vgain']
        }
        speed, steer = planner.plan(scan_data, odom_data)

        obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
        laptime += step_reward
        env.render(mode='human')

    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time() - start)


if __name__ == '__main__':
    main()
