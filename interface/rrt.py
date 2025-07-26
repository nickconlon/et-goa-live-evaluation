import math
import random

import matplotlib.pyplot as plt
import numpy as np

#
# RRT class based on several different implementations I found online and some robotics classes.
#


class Node:
    """
    Node for RRT/RRT* Algorithm. This is what you'll make your graph with!
    """

    def __init__(self, pt, parent=None):
        self.point = pt  # n-Dimensional point
        self.parent = parent  # Parent node
        self.path_from_parent = []  # List of points along the way from the parent node (for visualization)


class Obstacle:
    circle = "CIRCLE"
    rectangle = "RECTANGLE"
    """
    Obstacles can be of type circle, or rectangle. Name should be unique but not enforced.
    circle: center (x,y), axis = [radius]
    rectangle: center (x,y), axis = [width, height]
    """

    def __init__(self, obs_type, obs_center, obs_axis, name, obs_angle=0, buffer=None):
        self.center = obs_center
        self.axis = obs_axis
        self.type = obs_type
        self.id = name
        self.angle = obs_angle
        self.buffer = buffer


class RRT:
    """
    Class to run Rapidly Expanding Random Tree algorithm. Includes both RRT and RRT*.
    """

    def __init__(self, world_bounds, dynamics_mode):
        self.state_bounds = world_bounds
        self.obstacles = {}
        self.dynamics_mode = dynamics_mode
        self.next_obs_id = 0

    def add_obstacle(self, obstacle):
        """
        Add an obstacle to the obstacle dict.
        """
        self.obstacles[obstacle.id] = obstacle

    def add_obstacles(self, obstacle_list):
        """
        Add a list of obstacles to the obstacle dict.
        """
        for obs in obstacle_list:
            self.obstacles[obs.id] = obs

    def remove_obstacle(self, obs_id):
        """
        Remove an obstacle from the obstacles dict.
        """
        self.obstacles.pop(obs_id)

    def get_obstacles_list(self):
        """
        Get the obstacles [[x,y],radius] as a list
        """
        return list(self.obstacles.values())

    def get_obstacles_dict(self):
        """
        Get the obstacles [[x,y],radius] as a list
        """
        return self.obstacles

    def get_random_valid_vertex(self):
        """
        Returns a random valid vertex in the state bounds.
        """
        vertex = None
        while vertex is None:
            pt = (
                np.random.rand(self.state_bounds.shape[0])
                * (self.state_bounds[:, 1] - self.state_bounds[:, 0])
                + self.state_bounds[:, 0]
            )
            if self._valid(pt):
                vertex = pt
        return vertex

    def _valid(self, state):
        """
        Is the state is valid?
        @return True if the state is within the state bounds and not obstructed by an obstacle, or False otherwise.
        """
        for dim in range(self.state_bounds.shape[0]):
            if state[dim] < self.state_bounds[dim][0]:
                return False
            if state[dim] >= self.state_bounds[dim][1]:
                return False
        for obs_id, obs in self.obstacles.items():
            buffer = 0.0
            if obs.buffer is not None:
                buffer = obs.buffer

            if obs.type is Obstacle.circle:
                if np.linalg.norm(state - obs.center) <= obs.axis[0] + buffer:
                    return False
            elif obs.type is Obstacle.rectangle:
                if (state[0] > obs.center[0] - obs.axis[0] / 2) and (
                    state[0] < obs.center[0] + obs.axis[0] / 2
                ):
                    if (state[1] > obs.center[1] - obs.axis[1] / 2) and (
                        state[1] < obs.center[1] + obs.axis[1] / 2
                    ):
                        return False
        return True

    def _get_non_holonomic_actions(self):
        """
        Returns a valis list of actions for the non-holonomic model.
        """
        actions = np.array([[-1.0, -1.0], [1.0, 1.0], [-1.0, 1.0], [1.0, -1.0]])
        action_list = list(actions)
        for action in actions:
            action_list.append(action * 0.4 * np.random.random())
        for action in actions:
            action_list.append(action * 0.1 * np.random.random())
        return action_list

    def _simulate_non_holonomic_action(self, state, action):
        """
        Returns a discretized path along which the agent moves when performing an action in a state.
        """
        path = np.linspace(state, state + action, 10)
        return path

    def _get_nearest_vertex(self, node_list, q_point):
        """
        @param node_list: List of Node objects
        @param q_point: Query vertex
        @return Node in node_list with closest node.point to query q_point
        """
        d_min = 100000
        n_min = node_list[0]
        for node in node_list:
            d = np.linalg.norm(node.point - q_point)
            if d < d_min:
                d_min = d
                n_min = node
        return n_min

    def _steer_non_holonomic(self, from_point, to_point, delta_q):
        """
        @param from_point: Point where the path to "to_point" is originating from
        @param to_point: n-Dimensional point (array) indicating destination
        @param delta_q: Max path-length to cover, possibly resulting in changes to "to_point"
        @returns path: list of points leading from "from_node" to "to_point" (inclusive of endpoints)
        """
        if self.dynamics_mode == "holonomic":
            return self._steer_holonomic(from_point, to_point, delta_q)
        elif self.dynamics_mode == "discrete_non_holonomic":
            return self._steer_discrete_non_holonomic(from_point, to_point)
        elif self.dynamics_mode == "continuous_non_holonomic":
            return self._steer_continuous_non_holonomic(from_point, to_point)

    def _steer_holonomic(self, from_point, to_point, delta_q):
        """
        @param from_point: Point where the path to "to_point" is originating from
        @param to_point: n-Dimensional point (array) indicating destination
        @param delta_q: Max path-length to cover, possibly resulting in changes to "to_point"
        @returns path: list of points leading from "from_node" to "to_point" (inclusive of endpoints)
        """
        length = np.linalg.norm(from_point - to_point)
        if length > delta_q:
            direction = (to_point - from_point) / length
            to_point = from_point + direction * delta_q
        xs = np.linspace(from_point[0], to_point[0], num=10)
        if from_point[0] < to_point[0]:
            ys = np.interp(
                xs, [from_point[0], to_point[0]], [from_point[1], to_point[1]]
            )
        else:
            ys = np.interp(
                xs, [to_point[0], from_point[0]], [to_point[1], from_point[1]]
            )
        path = []

        for x, y in zip(xs, ys):
            path.append([x, y])

        return path

    def _steer_discrete_non_holonomic(self, from_point, to_point):
        """
        Given a fixed discrete action space and dynamics model,
        choose the action that gets you closest to "to_point" when executing it from "from_point"

        @param from_point: Point where the path to "to_point" is originating from
        @param to_point: n-Dimensional point (array) indicating destination
        @returns path: list of points leading from "from_node" to "to_point" (inclusive of endpoints)
        """
        actions_list = self._get_non_holonomic_actions()
        paths = []
        for action in actions_list:
            paths.append(self._simulate_non_holonomic_action(from_point, action))

        closest = 100000
        closest_idx = None
        for i, p in enumerate(paths):
            end = p[-1]
            length = np.linalg.norm(end - to_point)
            if length < closest:
                closest = length
                closest_idx = i

        return paths[closest_idx]

    def _steer_continuous_non_holonomic(self, from_point, to_point):
        raise NotImplementedError

    ##################################################################################
    # Start helper functions for cost, get_neighbors, collision_free, and obstacle_free
    def _cost(self, node1, node2):
        """
        Get the cost of traveling from node1 to node2.

        @param node1 The first node.
        @param node2 The second node.
        @return The cost traveling from node1 to node 2
        """
        cost = 0
        parent = node1
        while parent is not node2:
            cost += np.linalg.norm(parent.point - parent.path_from_parent[0])
            parent = parent.parent
        return cost

    def _get_neighbors(self, graph, x_new, radius):
        """
        Find all neighbors of x_new within some radius.

        @param graph The graph.
        @param x_new The node whose neighbors we check.
        @param radius The radius to check for neighbors
        @return List of neighbors.
        """
        neighbors = []
        for x_near in graph:
            if x_near is x_new:
                continue
            d = np.linalg.norm(x_near.point - x_new.point)
            if d < radius:
                neighbors.append(x_near)
        return neighbors

    def _collision_free(self, node1, node2, delta_q):
        """
        Is the path from node1 to node2 collision free?

        @param node1 The first node.
        @param node2 The second node.
        @param delta_q The max single segment path-length.
        @return True if the path is collision free, false otherwise.
        """
        path = self._steer_non_holonomic(node1.point, node2.point, delta_q)
        for p in path:
            if not self._valid(np.asarray(p)):
                return False
        return True

    def _obstacle_free(self, path):
        """
        Is the path obstacle free?

        @param path The path.
        @return True of the path is obstacle free, false otherwise.
        """
        for p in path:
            if not self._valid(np.asarray(p)):
                return False
        return True

    # End helper functions for cost, get_neighbors, collision_free, and obstacle_free
    ##################################################################################

    def rrt(self, start_point, goal_point, k, delta_q):
        """
        Run RRT given the start and (optional) goal node for k iterations. Is not goal node is
        given just search the space, if a goal node is given head generally in that direction.

        @param start_point A start point within state_bounds to grow the RRT from.
        @param goal_point An (optional) goal point within state_bounds to target with the RRT.
        @param k The number of points to sample.
        @param delta_q  The maximum distance allowed between vertices.
        @returns A list of RRT graph nodes.
        """

        q_start = Node(start_point)
        node_list = [q_start]

        for _ in range(k):
            q_rand = (
                self.get_random_valid_vertex()
            )  # q_rand is a randomly sampled point

            # Bias the search towards the goal
            if goal_point is not None:
                if np.random.random_sample() > 0.5:
                    q_rand = goal_point

            q_near = self._get_nearest_vertex(
                node_list, q_rand
            )  # q_near is the nearest vertex in G to q_rand
            path = self._steer_non_holonomic(
                q_near.point, q_rand, delta_q
            )  # path is the path from q_near to q_rand

            if not self._obstacle_free(path):
                continue

            q_new = Node(path[-1], q_near)
            q_new.path_from_parent = path
            node_list.append(q_new)

            # Check for goals
            if (goal_point is not None) and (
                np.linalg.norm(q_new.point - goal_point) < 1e-5
            ):
                break

        return node_list

    def rrt_star(self, start_point, goal_point, k, delta_q):
        """
        Run RRT* given the start and (optional) goal node for k iterations. Is not goal node is
        given just search the space, if a goal node is given head generally in that direction.

        @param start_point: Point within state_bounds to grow the RRT from.
        @param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None).
        @param k: Number of points to sample.
        @param delta_q: Maximum distance allowed between vertices.
        @returns List of RRT* graph nodes.
        """

        q_start = Node(start_point)
        node_list = [q_start]

        for _ in range(k):
            x_rand = (
                self.get_random_valid_vertex()
            )  # x_rand is a randomly sampled point

            # Bias the search towards the goal.
            if goal_point is not None:
                if np.random.random_sample() > 0.5:
                    x_rand = goal_point

            x_nearest = self._get_nearest_vertex(
                node_list, x_rand
            )  # x_nearest is the nearest vertex in G to x_rand
            path = self._steer_non_holonomic(
                x_nearest.point, x_rand, delta_q
            )  # path is the path from x_nearest to x_rand
            x_new = np.asarray(path[-1])

            # Check if the path is obstacle free
            if not self._obstacle_free(path):
                continue

            x_new = Node(x_new)
            node_list.append(x_new)

            # loop over neighbors of x_new to find closest within some radius
            x_min = x_nearest
            c_min = self._cost(x_nearest, q_start) + np.linalg.norm(
                x_nearest.point - x_new.point
            )
            radius = 0.2
            neighbors = self._get_neighbors(node_list, node_list[-1], radius)

            for x_near in neighbors:
                # if is collision free
                if not self._collision_free(x_near, node_list[-1], delta_q):
                    continue
                cost = self._cost(x_near, q_start) + np.linalg.norm(
                    x_near.point - x_new.point
                )
                if cost < c_min:
                    x_min = x_near
                    c_min = cost

            node_list[-1].parent = x_min
            node_list[-1].path_from_parent = np.asarray(
                self._steer_non_holonomic(x_min.point, x_new.point, 1)
            )

            for x_near in neighbors:
                # if is collision free
                if not self._collision_free(x_near, node_list[-1], delta_q):
                    continue
                cost = self._cost(node_list[-1], q_start) + np.linalg.norm(
                    x_nearest.point - x_new.point
                )
                if cost < self._cost(x_near, q_start):
                    x_near.parent = node_list[-1]
                    x_near.path_from_parent = np.asarray(
                        self._steer_non_holonomic(x_new.point, x_near.point, 1)
                    )

            # Check for goals
            if (goal_point is not None) and (
                np.linalg.norm(x_new.point - goal_point) < 1e-5
            ):
                break

        return node_list

    def get_path(self, node_list, goal_point):
        """
        Return the path from goal to start as array of [x,y] points.
        """
        path_points = []

        # Find the goal node
        goal_node = None
        for node in node_list:
            if (
                goal_point is not None
                and np.linalg.norm(node.point - np.array(goal_point)) <= 1e-5
            ):
                goal_node = node

        # Trace back the path
        if goal_node is not None:
            cur_node = goal_node
            while cur_node is not None:
                if cur_node.parent is not None:
                    node_path = np.array(cur_node.path_from_parent)
                    path_points.append(cur_node.point)
                    cur_node = cur_node.parent
                else:
                    break
        return np.asarray(path_points)


def plan_rrt_webots(
    start, goal, obstacles, bounds, visualize_route=False, filename="./plot.png"
):
    iterations = 5000
    planner = RRT(bounds, "holonomic")

    for ob in obstacles:
        planner.add_obstacle(ob)

    nodes = planner.rrt_star(start, goal, iterations, np.linalg.norm(bounds / 10.0))
    waypoints = planner.get_path(nodes, goal)
    waypoints = np.flip(waypoints)
    if visualize_route:
        visualize(bounds, planner.get_obstacles_dict(), nodes, goal, filename)

    return waypoints


##################################################
# stuff for testing
def get_nd_obstacle(state_bounds):
    """
    Create an obstacle within the state bounds.
    """
    center_vector = []
    for d in range(state_bounds.shape[0]):
        center_vector.append(
            state_bounds[d][0]
            + random.random() * (state_bounds[d][1] - state_bounds[d][0])
        )
    radius = random.random() * 0.6
    return [np.array(center_vector), radius]


def setup_random_2d_world():
    """
    Create a world. State bounds + randomly located set of obstacles.
    """
    state_bounds = np.array([[0, 10], [0, 10]])
    obs = {}  # [[x,y], radius] circular obstacles
    for n in range(30):
        obs[str(n)] = get_nd_obstacle(state_bounds)
    return state_bounds, obs


def setup_fixed_test_2d_world():
    """
    Create a world. State bounds + hard coded obstacles.
    """
    state_bounds = np.array([[0, 1], [0, 1]])
    obs = [
        Obstacle(Obstacle.circle, [0.5, 0.5], [0.2], "0"),
        Obstacle(Obstacle.circle, [0.1, 0.7], [0.1], "1"),
        Obstacle(Obstacle.circle, [0.7, 0.2], [0.1], "2"),
    ]
    return state_bounds, obs


def plot_circle(ax, x, y, radius, color="-k", label="test"):
    """
    Plot a circle at some (x,y) and radius.
    """
    deg = np.linspace(0, 360, 50)

    xl = [x + radius * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + radius * math.sin(np.deg2rad(d)) for d in deg]
    ax.plot(xl, yl, color)
    ax.annotate(text=label, xy=(x, y), ha="center")


def plot_rect(ax, x, y, width, height, angle=0, color="blue", label="test"):
    """
    Plot a circle at some (x,y) and radius.
    """
    rect = plt.Rectangle((x, y), width, height, color=color, angle=angle)
    ax.add_patch(rect)
    ax.annotate(text=label, xy=(x + width / 2, y + height / 2), ha="center")


def visualize(state_bounds, obstacles, nodes_list, goal_point=None, filename=None):
    """
    @param state_bounds Array of min/max for each dimension
    @param obstacles Locations and radii of spheroid obstacles
    @param nodes_list List of vertex locations
    @param goal_point The goal point
    @param filename The filename to save the plot to
    """

    fig, ax = plt.subplots(1)
    ax.set_xlim(state_bounds[0, 0], state_bounds[0, 1])
    ax.set_ylim(state_bounds[1, 0], state_bounds[1, 1])
    ax.axes.set_aspect("equal")
    for obs_id, obs in obstacles.items():
        buffer = 0.0
        if obs.buffer is not None:
            buffer = obs.buffer
        if obs.type is Obstacle.circle:
            plot_circle(ax, obs.center[0], obs.center[1], obs.axis[0], label=obs_id)
        elif obs.type is Obstacle.rectangle:
            plot_rect(
                ax,
                obs.center[0] - obs.axis[0] / 2,
                obs.center[1] - obs.axis[1] / 2,
                obs.axis[0],
                obs.axis[1],
                angle=obs.angle,
                label=obs_id,
            )

    node = None
    goal_node = None
    for node in nodes_list:
        if node.parent is not None:
            node_path = np.array(node.path_from_parent)
            ax.plot(node_path[:, 0], node_path[:, 1], "-b")
        if (
            goal_point is not None
            and np.linalg.norm(node.point - np.array(goal_point)) <= 1e-5
        ):
            goal_node = node
            ax.plot(node.point[0], node.point[1], "k^")
        else:
            ax.plot(node.point[0], node.point[1], "ro")

    ax.plot(nodes_list[0].point[0], nodes_list[0].point[1], "ko")

    if goal_node is not None:
        cur_node = goal_node
        while cur_node is not None:
            if cur_node.parent is not None:
                node_path = np.array(cur_node.path_from_parent)
                ax.plot(node_path[:, 0], node_path[:, 1], "--y")
                cur_node = cur_node.parent
            else:
                break

    if goal_point is not None:
        ax.plot(node.point[0], node.point[1], "gx")

    if filename is not None:
        fig.savefig(filename)
    else:
        plt.show()
