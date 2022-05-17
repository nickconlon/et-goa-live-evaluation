import numpy as np
import goa
import rrt_planner as planner


def checkInCircle(cx, cy, cr, x, y):
    """
    Is the point (x,y) contained within the circle (cx, cy, r) ?
    """
    return np.power(cx - x, 2) + np.power(cy - y, 2) < np.power(cr, 2)


def get_obstacles_hit_outcome(paths, obstacles):
    """
    Return a binary vector of obstacle hits
    """

    outcomes = np.zeros(len(paths))
    time_in_obstacles = np.zeros(len(paths))
    for pidx, path in enumerate(paths):
        obstacles_hit = np.zeros(len(obstacles))
        for p in path:
            for idx, o in enumerate(obstacles):
                if checkInCircle(o[0], o[1], o[2], p[0], p[1]):
                    obstacles_hit[idx] = 1
                    time_in_obstacles[pidx] +=1
        outcomes[pidx] = np.sum(obstacles_hit)
    return outcomes, time_in_obstacles


def get_goa(paths, obstacles):
    outcomes, times = get_obstacles_hit_outcome(paths, obstacles)
    z_star = 3
    bins = [-10, -5, -2.5, -0.5, 0.5]  # |9-6 | 5-3 | 1,2 | 0
    outcome_g = goa.general_oa(outcomes * -1, bins, z_star)
    print("GOA less than {} craters hit: {}".format(abs(np.ceil(bins[z_star-1])), goa.semantic_label(outcome_g)))
    z_star = 4
    bins = [-100, -50, -25, -10, -5, 1]  # |9-6 | 5-3 | 1,2 | 0
    time_g = goa.general_oa(times * -1, bins, z_star)
    print("GOA less than {} time in craters: {}".format(abs(bins[z_star]), goa.semantic_label(time_g)))
    return outcome_g, time_g


def get_self_confidence(state, goals, obstacles, play_area):
    """
    Calculate a new self-confidence for each goal

    :param state:       The platform state
    :param goals:       All goals
    :param obstacles:   All obstacles
    :param play_area:   The plat area bounds
    :return:
    """
    self_confidences = np.zeros(len(goals))
    for idx, goal in enumerate(goals):
        waypoints = planner.rollout(0, state, goal, obstacles, play_area)
        # do some SC calculations
        self_confidences[idx] = np.random.rand()
    return self_confidences


if __name__ == '__main__':
    z_star = 2  # 4
    outcomes = np.array([3., 4., 4., 4., 3., 4., 4., 3., 3., 3.,])
    outcomes = outcomes * -1
    bins = [-10, -5, -2.5, -0.5, 0.5]  # |9-6 | 5-3 | 1,2 | 0
    g = goa.general_oa(outcomes, bins, z_star)
    print(g)
