import numpy
import numpy as np
import matplotlib.pyplot as plt

import outcome_assessment


class model:
    def __init__(self):
        pass

    def step(self, s):
        pass

    def reset(self):
        pass


class platform_model(model):
    def __init__(self, target, velocity, turn, heading, position, noise):
        super(platform_model, self).__init__()
        self.target = target
        self.v = velocity
        self.r = turn
        self.theta = self.v / self.r
        self.h = heading
        self.pos = position
        self.tab = np.array
        self.noise = noise if noise is not None else 0.005

    def angle(self, v1, v2, acute):
        angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        if acute:
            return angle
        else:
            return 2 * np.pi - angle

    def angle_to_target(self):
        x = self.target[0] - self.pos[0]
        y = self.target[1] - self.pos[1]
        radianAngle = np.arctan2(y, x)

        # if radianAngle - self.h > +self.theta:
        #    return self.theta
        # if radianAngle - self.h < -self.theta:
        #    return -self.theta
        return radianAngle - self.h

    def distance_to_target(self):
        x = self.target[0] - self.pos[0]
        y = self.target[1] - self.pos[1]
        return np.sqrt(x ** 2 + y ** 2)

    def step(self, s):
        delta = self.angle_to_target()
        self.h = self.h + delta + np.random.normal(loc=0.0, scale=0.1)
        x = self.pos[0] + np.cos(self.h) * self.v + np.random.normal(loc=0.0, scale=self.noise)
        y = self.pos[1] + np.sin(self.h) * self.v + np.random.normal(loc=0.0, scale=self.noise)
        self.pos = [x, y]
        return [x, y]

    def reset(self):
        pass


class follower:
    def __init__(self, mymodel, points, axis, delta):
        self.model = mymodel
        self.points = points
        self.target = 0
        self.ax = axis
        self.delta = delta
        self.stop = False
        self.trajectory = []

    def step(self):
        if not self.stop:
            p = self.model.step(1)
            self.trajectory.append(p)
            if abs(self.model.pos[0] - self.model.target[0]) < self.delta and abs(
                    self.model.pos[1] - self.model.target[1]) < self.delta:
                self.target += 1
                if self.target >= len(self.points):
                    self.stop = True
                else:
                    self.model.target = self.points[self.target]

    def reset(self):
        self.model.reset()

    def render(self):
        plt.clf()
        plt.xlim([-1, 15])
        plt.ylim([-1, 15])
        [plt.scatter(*x, c='green') for x in self.points]
        plt.scatter(*self.model.pos, c='black')
        plot_arrow(*self.model.pos, self.model.h)
        if self.stop:
            d = np.array(self.trajectory)
            plt.plot(d[:, 0], d[:, 1], '--', c='black')
        plt.pause(0.001)


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw), fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def plot_sim(trajectories, ax):
    for t in trajectories:
        d = np.array(t)
        ax.scatter(d[:, 0], d[:, 1])


def simulate(_current_state, _waypoints, _num_simulations, _render=False, noise=None):
    _velocity = 0.1
    _heading = np.pi
    _turn_radius = 1
    _steps = 1000
    _trajectories = []

    for j in range(_num_simulations):
        u = platform_model(_waypoints[0], _velocity, _turn_radius, _heading, _current_state, noise=noise)
        f = follower(u, _waypoints, None, delta=0.5)
        for _ in range(_steps):
            f.step()
            if _render:
                f.render()
            if f.stop:
                _trajectories.append(np.array(f.trajectory))
                break
    return _trajectories


def main():
    start = [5, 0]
    velocity = 0.1
    heading = np.pi
    turn_radius = 1
    points = [[5, 5], [5, 8], [10, 5]]

    steps = 1000
    sims = 10
    trajectories = []
    for j in range(sims):
        u = platform_model(points[0], velocity, turn_radius, heading, start)
        # fig, ax = plt.subplots(1, 1)
        f = follower(u, points, None, delta=0.5)

        print("sim {}".format(j))
        for i in range(steps):
            f.step()
            f.render()
            if f.stop:
                trajectories.append(f.trajectory)
                break

    plt.ylim([-1, 15])
    for t in trajectories:
        d = np.array(t)
        # plt.plot(d[:, 0], d[:, 1], '--', c='black')
        plt.scatter(d[:, 0], d[:, 1])
    for t in points:
        plt.scatter(t[0], t[1])
    plt.show()


def crater_field(num_new_obstacles, real_state, obs_list, minx, maxx, miny, maxy, r=0.25):
    for _ in range(num_new_obstacles):
        crater = (
            minx + np.random.rand() * (maxx - minx), miny + np.random.rand() * (maxy - miny), np.random.rand() * r)
        if crater[0] != real_state[0] and crater[1] != real_state[1]:
            obs_list.append(crater)


def forecasts(goals, obstacles, area, num_forecasts):
    _assessments = []
    _trajectories = []
    for goal in goals:
        print("planning for goal: {}".format(goal))
        _waypoints = planner.rollout(1, s, goal, obstacles, area)[0]
        _sim_trajectories = simulate(s, _waypoints, num_forecasts, noise=0.1)
        _oa_g, _t_g = outcome_assessment.get_goa(_sim_trajectories, obstacles)
        _assessments.append((_oa_g, _t_g))
        _trajectories.append(_sim_trajectories)
    return _assessments, _trajectories


if __name__ == '__main__':
    import rrt_planner as planner
    import et_goa as et_goa
    import math

    obstacle_list = [(-1, 2.5, 0.5), (0, 2, 0.5), (0.5, 3, 0.8), (3, 7, 0.5), (0, 8, 0.3)]
    play_area = [-5, 5, 0, 10]
    s = [0, 0]
    g1 = [-4, 7]
    g2 = [-1, 8]
    g3 = [2, 8]
    goals = [g1, g2, g3]
    goal_idx = 2
    rollout_size = 10

    ############## Initial Plan #############
    crater_field(25, s, obstacle_list, minx=2, maxx=5, miny=3, maxy=6)
    crater_field(25, s, obstacle_list, minx=-2, maxx=-5, miny=3, maxy=6)
    ##### Plan the forecasted trajectories
    assessments, trajectories = forecasts(goals, obstacle_list, play_area, rollout_size)
    sim_trajectories = trajectories[goal_idx]
    assessment = assessments[goal_idx]
    ##### Plan the real trajectory
    waypoints = planner.rollout(1, s, goals[goal_idx], obstacle_list, play_area)[0]
    real_trajectory = simulate(s, waypoints, 1, noise=0.01)[0]
    ############## Initial Plan #############

    steps = len(real_trajectory)
    step_offset = 0
    event_offset = 0
    fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3)
    for step in range(steps):
        x_actual = np.array([real_trajectory[step][0]])
        y_actual = np.array([real_trajectory[step][1]])
        x_estimates = []
        y_estimates = []
        for t in sim_trajectories:
            try:
                x_estimates.append(t[step - step_offset][0])
                y_estimates.append(t[step - step_offset][1])
            except:
                pass

        ############## plots ####################
        ax1.scatter(x_estimates, y_estimates, c='blue')
        ax1.scatter(x_actual, y_actual, c='red')
        ax1.set_ylim([play_area[2], play_area[3]])
        ax1.set_xlim([play_area[0], play_area[1]])
        ############## plots ####################

        ############## ET-GOA ###################
        et_x = et_goa.segments(np.array(x_estimates), x_actual, min(x_estimates) - 5, max(x_estimates) + 5, 1000, ax2)
        et_y = et_goa.segments(np.array(y_estimates), y_actual, min(y_estimates) - 5, max(y_estimates) + 5, 1000, ax3)
        if et_x < 0.1 or et_y < 0.1:
            step_offset = step
            print("Triggered!")
            ##### Re forecast trajectories due to the event
            s = real_trajectory[step]
            obstacles_to_plan_with = []
            for o in obstacle_list:
                if not outcome_assessment.checkInCircle(o[0], o[1], o[2], s[0], s[1]):
                    obstacles_to_plan_with.append(o)
            sim_trajectories = simulate(s, waypoints, rollout_size, noise=0.1)
            outcomes = outcome_assessment.get_obstacles_hit_outcome(sim_trajectories, obstacle_list)
            oa_g, t_g = outcome_assessment.get_goa(sim_trajectories, obstacle_list)
        ############## ET-GOA ###################

        ############## E-1 ######################
        if step == 25:
            print("Event 1")
            ul = [real_trajectory[x] for x in range(step)]
            crater_field(10, s, obstacle_list, minx=-3, maxx=3, miny=3, maxy=6, r=1)
            s = real_trajectory[step]
            obstacles_to_plan_with = []
            for o in obstacle_list:
                if not outcome_assessment.checkInCircle(o[0], o[1], o[2], s[0], s[1]):
                    obstacles_to_plan_with.append(o)
            waypoints = planner.rollout(1, s, goals[goal_idx], obstacles_to_plan_with, play_area)[0]
            real_trajectory = simulate(s, waypoints, 1, noise=0.001)[0]
            ul = np.zeros_like(ul)
            real_trajectory = np.concatenate((ul, real_trajectory))
        ############## E-1 ######################

        ############## E-2 ######################
        if step == 40:
            print("Event 2")
            ul = [real_trajectory[x] for x in range(step)]
            for i in range(10):
                obstacle_list.pop(np.random.randint(0, len(obstacle_list) - i))
            s = real_trajectory[step]
            obstacles_to_plan_with = []
            for o in obstacle_list:
                if not outcome_assessment.checkInCircle(o[0], o[1], o[2], s[0], s[1]):
                    obstacles_to_plan_with.append(o)
            waypoints = planner.rollout(1, s, goals[goal_idx], obstacle_list, play_area)[0]
            real_trajectory = simulate(s, waypoints, 1, noise=0.01)[0]
            ul = np.zeros_like(ul)
            real_trajectory = np.concatenate((ul, real_trajectory))
        ############## E-2 ######################

        ############## plots ####################
        for trajectory in sim_trajectories:
            ax1.plot([x[0] for x in trajectory], [x[1] for x in trajectory])
        for (x, y, size) in obstacle_list:
            deg = list(range(0, 360, 5))
            deg.append(0)
            xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
            yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
            ax1.plot(xl, yl, color='black')
        ax1.set_ylim([play_area[2], play_area[3]])
        ax1.set_xlim([play_area[0], play_area[1]])
        ax1.set_title("H: {:.2f} T: {:.2f}".format(assessment[0], assessment[1]))
        for g in goals:
            ax1.scatter(g[0], g[1], color='red')
        plt.pause(0.01)
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ############## plots ####################
    outcomes = outcome_assessment.get_obstacles_hit_outcome([real_trajectory], obstacle_list)
    print("real outcomes ", outcomes)
