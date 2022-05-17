import math
from rrt import RRT

"""
https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/RRTStar
http://ilab.usc.edu/publications/doc/Itti_Baldi06nips.pdf
https://link-springer-com.colorado.idm.oclc.org/article/10.1007/s11004-017-9709-7
https://en.wikipedia.org/wiki/Probabilistic_forecasting
https://stats.stackexchange.com/questions/296285/find-cdf-from-an-estimated-pdf-estimated-by-kde
https://en.wikipedia.org/wiki/Kernel_density_estimation
"""

"""
RRT path planning code. Adapted from:
https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/RRTStar
"""


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=30.0,
                 path_resolution=1.0,
                 goal_sample_rate=20,
                 max_iter=300,
                 connect_circle_dist=50.0,
                 search_until_max_iter=False,
                 play_area=None):
        """
        :param start                    The starting state
        :param goal                     The goal state
        :param obstacle_list            List of all known obstacles
        :param rand_area                Area which nodes can be sampled from
        :param expand_dis               Distance to expand nodes
        :param path_resolution          Resolution of the path
        :param goal_sample_rate         Rate at which the planner samples the goal
        :param max_iter                 Max number of RRT iterations
        :param connect_circle_dist      TODO
        :param search_until_max_iter    True if the planner should run until max iter, false to return first found path
        :param play_area                The play area bounds in [minx, maxx, miny, maxy]
        """
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter, play_area=play_area)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter

    def planning(self, animation=True):
        """
        Calculate a new self-confidence for each goal

        :param animation:   True if planning should be animated, false otherwise
        :return:            A RRT* Path to the goal
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            # print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd,
                                  self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + math.hypot(new_node.x - near_node.x, new_node.y - near_node.y)

            if self.check_collision(new_node, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(
                    new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if animation:
                self.draw_graph(rnd)

            if (not self.search_until_max_iter) and new_node:
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list near_inds and set such a node as the parent of
        new_node.

        :param new_node     randomly generated node with a path from its neared point
        :param near_inds    Indices of indices of the nodes what are near to new_node
        :return             None, or a copy of the new_node
        """
        if not near_inds:
            return None

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        """
        Search for the best goal node in the tree

        :return: The index of the best goal node, or None if not found
        """
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball

        :param new_node     New randomly generated node without collisions between its neighbors
        :return:            List with the indices of the nodes inside the ball of radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
        For each node in near_inds, this will check if it is cheaper to arrive to them from new_node.
        In such a case, this will re-assign the parent of the nodes in near_inds to new_node.

        :param new_node     Node randomly added which can be joined to the tree
        :param near_inds    A list of indices of the self.new_node which contains nodes within a circle of a given radius.
        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        """
        Calculate the cost from from_node to to_node

        :param from_node    Some from node
        :param to_node      Some to node
        :return             The cost
        """
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        """
        Propagate cost to the leaves of the tree

        :param parent_node  The parent node
        """
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


def get_path(current_state, current_goal, obstacles, play_area):
    """
    Get a path from the current_state to the current_goal avoiding obstacles and within the play_area.

    :param current_state:   The current state
    :param current_goal:    The current goal
    :param obstacles:       List of circular obstacles [x,y,r]
    :param play_area:       The size of the play area [minx, maxx, miny, maxy]
    :return:                The path to the goal
    """
    rrt_star = RRTStar(
        start=current_state,
        goal=current_goal,
        rand_area=[min(play_area), max(play_area)],
        obstacle_list=obstacles,
        expand_dis=1,
        max_iter=1000,
        play_area=play_area,
        search_until_max_iter=True)
    path = rrt_star.planning(animation=False)

    if path is None:
        print("Cannot find path")
        return []
    else:
        print("found path!!")
        path.reverse()
    return path


def rollout(num_rollouts, current_state, current_goal, obstacles, play_area):
    """
    Perform rollouts from the current_state to the current_goal avoiding obstacles and within the play_area.

    :param num_rollouts:    The number of rollouts to execute
    :param current_state:   The current state
    :param current_goal:    The current goal
    :param obstacles:       List of circular obstacles [x,y,r]
    :param play_area:       The size of the play area [minx, maxx, miny, maxy]
    :return:                A list of rollout state trajectories
    """
    _rollouts = []
    for i in range(num_rollouts):
        _rollout = get_path(current_state, current_goal, obstacles, play_area)
        _rollouts.append(_rollout)

    # from concurrent import futures
    # import threading
    # with futures.ThreadPoolExecutor(max_workers=5) as executor:
    #    future = {executor.submit(get_next_states, current_state, current_goal, obstacles, play_area)
    #              for _ in range(num_rollouts)}
    #    for fut in futures.as_completed(future):
    #        print("The outcome is {}".format(fut.result()))
    #        _rollouts.append(fut.result())
    return _rollouts
