import numpy as np

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
    obstacles_hit = np.zeros(len(obstacles))
    for p in paths:
        for idx, o in enumerate(obstacles):
            if checkInCircle(o[0], o[1], o[2], p[0], p[1]):
                obstacles_hit[idx] = 1
    return obstacles_hit


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
