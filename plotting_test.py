import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import datetime

def send_map(paths, obstacles, state, goal, goals, play_area):
    """
    Create a map of the play area

    :param paths:       Some paths to display
    :param obstacles:   Obstacles to display
    :param state:       Current state of the platform
    :param goal:        Current goal of the platform
    :param play_area:   The play area
    :return:            A map of the play area as a numpy RBG array
    """
    fig, ax = plt.subplots(nrows=1, ncols=1)

    ax.set_axisbelow(True)
    for path in paths:
        ax.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')

    for (x, y, size) in obstacles:
        p=patches.Circle((x, y), radius=size, facecolor='grey', ec='black')
        ax.add_artist(p)

    for (x, y) in goals:
        p=patches.Circle((x, y), radius=0.5, facecolor='orange', ec='black')
        ax.add_artist(p)

    p = patches.Circle((goal[0], goal[1]), radius=0.5, facecolor='green', ec='black')
    ax.add_artist(p)

    ax.scatter(state[0], state[1], c='blue')
    ax.scatter(goal[0], goal[1], c='green')
    ax.grid()
    ax.set_axisbelow(True)
    ax.set_aspect('auto')
    ax.set_yticks(np.arange(play_area[2], play_area[3]))
    ax.set_xticks(np.arange(play_area[0], play_area[1]))
    ax.xaxis.set_ticklabels([])
    ax.yaxis.set_ticklabels([])

paths = [[[0,0], [0,1],[1,1],[2,2],[3,5],[5,5]]]
obstacles = [(6,3,1), (5,1,0.75)]
state = [0,0]
goal = [5,5]
goals = [(1,5),(3,5),(5,5)]
play_area = [-1, 10, -1, 10]
send_map(paths, obstacles, state, goal, goals, play_area)
plt.show()

