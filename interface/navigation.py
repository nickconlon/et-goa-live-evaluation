#!/usr/bin/env python
# Import necessary libraries
import math  # Math library
import time
import traceback

import numpy as np
import rospy  # Ros library
from typing import List, Any
from numpy.typing import NDArray
import rrt
from et_goa import et_goa
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist  # Twist messages
from matplotlib import patches as patches
from matplotlib import pyplot as plt
from ros_conversion import extract_msg

from world_model import WorldModel


class WaypointFollower:
    def __init__(self, world_model: WorldModel) -> None:
        self.x_dest = None
        self.y_dest = None
        self.pose_cmd_vel = None
        self.rot_cmd_vel = None
        self.dist_err = 1
        self.K1 = 2
        self.K2 = 4
        self.xx = 0
        self.yy = 0
        self.last_time = 0
        self.counter = 0
        self.start_time = time.time()
        self.et_object = et_goa(world_model)
        self.do_et = False
        self.MQ = -1
        self.predx = -1
        self.predy = -1
        ###

        self.sleep_rate = rospy.Rate(10)

        # Initialize odometry subscriber
        self.grab_odom = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.calculate_heading
        )

        # Initialize command velocity publisher
        self.pub_vel = rospy.Publisher(
            "/jackal_velocity_controller/cmd_vel", Twist, queue_size=10
        )

    def calculate_heading(self, data: Any) -> None:
        """
        PD control for waypoint following
        """
        self.last_time = time.time()
        pose, angle = extract_msg(data)
        (x_pose, y_pose, z_pose) = pose[0], pose[1], pose[2]
        (y_rot, x_rot, z_rot) = angle[0], angle[1], angle[2]
        self.xx = x_pose
        self.yy = y_pose
        self.counter += 1
        if self.do_et:
            if self.counter % 100 == 0:
                try:
                    print("trying to ET-GOA")
                    self.MQ, self.predx, self.predy = self.et_object.get_si(
                        self.xx, self.yy, t=self.last_time - self.start_time
                    )
                except Exception:
                    traceback.print_exc()
        # Calculate x and y errors
        x_err = self.x_dest - x_pose
        y_err = self.y_dest - y_pose

        # Calculate angular error
        angle_err = math.atan2(y_err, x_err)

        if angle_err > math.pi:
            angle_err = angle_err - (2 * math.pi)

        angle_err = angle_err - z_rot

        if angle_err > math.pi:
            angle_err = angle_err - (2 * math.pi)
        elif angle_err < -math.pi:
            angle_err = angle_err + (2 * math.pi)

        # Calculate distance error
        self.dist_err = math.sqrt((x_err**2) + (y_err**2))

        # Calculate command velocities
        max_speed = 0.25
        if abs(angle_err) > 0.05:  # Only rotate
            if angle_err >= 0:
                self.rot_cmd_vel = np.minimum(angle_err * self.K1, max_speed)
            else:
                self.rot_cmd_vel = np.maximum(angle_err * self.K1, -max_speed)
            self.pose_cmd_vel = 0
        else:  # Rotate and move
            # self.rot_cmd_vel = np.minimum(angle_err * self.K1, max_speed)
            if angle_err >= 0:
                self.rot_cmd_vel = np.minimum(angle_err * self.K1, max_speed)
            else:
                self.rot_cmd_vel = np.maximum(angle_err * self.K1, -max_speed)
            self.pose_cmd_vel = np.minimum(self.dist_err * self.K2, max_speed)

    def reset(self) -> None:
        """
        Reset the parameters
        """
        self.x_dest = None
        self.y_dest = None
        self.pose_cmd_vel = None
        self.rot_cmd_vel = None
        self.dist_err = 1

    def go_to_waypoint(self, _x: float, _y: float) -> None:
        """
        Navigate to a waypoint at (_x, _y)
        """
        t_timeout = 1
        self.x_dest = _x
        self.y_dest = _y

        self.do_et = True
        # Go to waypoint
        while self.dist_err > 0.1:
            if abs(self.last_time - time.time()) < t_timeout:
                vel = Twist()
                vel.linear.x = self.pose_cmd_vel
                vel.angular.z = self.rot_cmd_vel
                self.pub_vel.publish(vel)
            self.sleep_rate.sleep()


def plan(
    goal_pos: NDArray[float],
    robot_pos: NDArray[float],
    obstacle_pos: List[rrt.Obstacle],
    ret: str = "all",
) -> NDArray[float]:
    """
    The RRT planner is weird and takes everything in (y, x)..
    """
    # RRT goal = [y, x]
    rrt_goal = np.flip(goal_pos)

    # RRT point [y, x]
    starting_point = np.array([robot_pos[1], robot_pos[0]])

    # [ymin ymax], [xmin, xmax]
    bounds = np.array([[1, 10], [1, 5]])

    waypoints = rrt.plan_rrt_webots(starting_point, rrt_goal, obstacle_pos, bounds)
    if len(waypoints) == 0:
        _ret_waypoint = []
    elif ret == "next":
        _ret_waypoint = waypoints[0]
    else:
        _ret_waypoint = waypoints

    print("starting at: " + str(starting_point))
    print("going to: " + str(goal_pos))
    print("next waypoint " + str(_ret_waypoint))
    return np.array(_ret_waypoint)


def plot_map(position, goal, waypoints, obstacles, predx=None, predy=None):
    fig, ax = plt.subplots(figsize=(3, 3))
    if len(waypoints) > 0:
        # plt.scatter([position[0]], [position[1]], color='black')
        ax.plot(waypoints[:, 0], waypoints[:, 1], color="black")
        ax.scatter(waypoints[:, 0], waypoints[:, 1], color="black")
        ax.plot(
            [position[0], waypoints[0, 0]],
            [position[1], waypoints[0, 1]],
            color="black",
        )
    if goal is not None:
        plt.scatter(*goal, color="green")
    if predx is not None:
        ax.scatter([predx], [predy], color="orange")
    for o, loc in obstacles.items():
        obs = patches.Circle(loc, radius=0.25, color="black", fill=False)
        ax.add_patch(obs)
    ax.scatter(*position, color="blue", label="robot pose")

    ax.set_ylim([0, 10.1])
    ax.set_xlim([0, 10.1])
    ax.set_yticks([0, 2, 4, 6, 8, 10])
    ax.set_xticks([0, 2, 4, 6, 8, 10])
    ax.set_ylabel("y")
    ax.set_xlabel("x")
    rect = patches.Rectangle(
        (1, 1), 4, 9, linewidth=1, edgecolor="black", facecolor="none"
    )
    ax.add_patch(rect)
    plt.tight_layout()
    fig.canvas.draw()
    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    img = img.reshape(*reversed(fig.canvas.get_width_height()), 3)
    # plt.imshow(img)
    # plt.show()
    # plt.savefig('basemap.png')
    plt.close("all")
    return img
