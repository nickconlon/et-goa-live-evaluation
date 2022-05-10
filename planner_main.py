import numpy as np
import base64
import cv2
import matplotlib.pyplot as plt
import zmq
import math

from outcome_assessment import get_self_confidence
from zmq_publisher import ZmqPublisher
from zmq_subscriber import ZmqSubscriber
import rrt_planner as planner

plt.switch_backend('Agg')


def canvas2rgb_array(canvas):
    """
    Convert a matplotlib canvas into a numpy RGB array
    Adapted from: https://stackoverflow.com/a/21940031/959926

    :param canvas:  The matplotlib canvas
    :return:        The canvas as a numpy RGB array
    """
    canvas.draw()
    buf = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
    ncols, nrows = canvas.get_width_height()
    scale = round(math.sqrt(buf.size / 3 / nrows / ncols))
    return buf.reshape(scale * nrows, scale * ncols, 3)


def get_map(paths, obstacles, state, goal, play_area):
    """
    Create a map of the play area

    :param paths:       Some paths to display
    :param obstacles:   Obstacles to display
    :param state:       Current state of the platform
    :param goal:        Current goal of the platform
    :return:            A map of the play area as a numpy RBG array
    """
    fig, ax = plt.subplots(nrows=1, ncols=1)

    for path in paths:
        ax.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')

    for (x, y, size) in obstacles:
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        ax.plot(xl, yl, color='black')

    ax.scatter(state[0], state[1], c='green')
    ax.scatter(goal[0], goal[1], c='blue')
    ax.grid()
    ax.set_aspect('auto')
    ax.set_yticks(np.arange(play_area[2], play_area[3]))
    ax.set_xticks(np.arange(play_area[0], play_area[1]))
    plt_array = canvas2rgb_array(fig.canvas)
    return plt_array


def check_new_goal(goal_sub, current_goal):
    """
    Check for any new goals

    :param goal_sub:        The subscriber we receive goals on
    :param current_goal:    The current goal
    :return:                A new goal index if received, else the old goal index
    """
    print("Checking for new goal")
    try:
        _new_goal = goal_sub.receive(flags=zmq.NOBLOCK)
        _new_goal = int(_new_goal.split()[1])
        return _new_goal
    except:
        pass
    return current_goal


if __name__ == "__main__":
    confidence_publisher = ZmqPublisher("*", "5556")
    map_publisher = ZmqPublisher("*", "5557")
    goal_subscriber = ZmqSubscriber("localhost", "5558", "goal")

    # Obstacles in the environment - can be updated in realtime
    obstacle_list = [(0, 2, 0.5), (0.5, 3, 0.8), (3, 7, 0.5), (0, 8, 0.3)]

    # The play area [minx, maxx, miny, maxy]
    play_area = [-5, 5, 0, 10]

    # State - can be updated in realtime
    s = [0, 0]

    # List of goals - can be updated in realtime
    goals = [[0, 0], [-4, 7], [-1, 8], [2, 8]]

    # Wait for the first goal from the UI
    print("Waiting for first goal")
    new_goal = goal_subscriber.receive()
    goal_idx = int(new_goal.split()[1])
    print(new_goal)

    for i in range(15):
        """
        Check for any new goals for the robot
        """
        goal_idx = check_new_goal(goal_subscriber, goal_idx)

        """
        Do robot GOA calculations and publish to the UI
        TODO implement this
        TODO event triggering
        """
        topic = "sc"
        scs = get_self_confidence(s, goals, obstacle_list, play_area)
        datastring = f"{topic} {scs[0]} {scs[1]} {scs[2]}"
        print("Publishing new self-confidence: ", datastring)
        confidence_publisher.publish(datastring)

        """
        Do a planning rollout, create a new image of the plan, publish the image to the UI
        """
        waypoints = planner.rollout(1, s, goals[goal_idx], obstacle_list, play_area)
        img = get_map(waypoints, obstacle_list, s, goals[goal_idx], play_area)
        string = base64.b64encode(cv2.imencode('.png', img)[1]).decode()
        print("Publishing new map")
        print(waypoints)
        map_publisher.publish(string)

        """
        TODO Publish the next waypoint to ROS
        """
        next_waypoint = waypoints[0][1]
        print("Next waypoint: ", next_waypoint)

        """
        For now - magically teleport to the next waypoint
        TODO get updated state from ROS
        TODO get updated obstacles from ROS
        """
        s = next_waypoint
        print(s)
