import numpy as np
import base64
import cv2
import matplotlib.pyplot as plt
import zmq
import math

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

import outcome_assessment
from outcome_assessment import get_self_confidence
from zmq_publisher import ZmqPublisher
from zmq_subscriber import ZmqSubscriber
import rrt_planner as planner

plt.switch_backend('Agg')


class GazeboCommsClient:
    def __init__(self):
        self.move_base_srv = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_srv.wait_for_server()
        self.xy = np.zeros(2)
        self.rpy = np.zeros(3)
        self.gazebo_state = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback)
        self.odom_state = rospy.Subscriber("/jackal_velocity_controller/odom", Odometry, self.odom_callback)

    def send_waypoint(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, np.deg2rad(theta))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base_srv.send_goal(goal)
        rospy.sleep(2)
        #wait = self.move_base_srv.wait_for_result()
        #if not wait:
        #   rospy.logerr("Action server not available!")
        #   rospy.signal_shutdown("Action server not available!")
        #else:
        #   return self.move_base_srv.get_result()
        return 1

    def get_state(self):
        return self.xy

    def odom_callback(self, msg):
        xyz = msg.pose.pose.position
        self.xy = np.array([xyz.x, xyz.y])

    def gazebo_callback(self, msg):
        model_idx = msg.name.index('jackal')
        lin = msg.pose[model_idx].position
        ang = msg.pose[model_idx].orientation
        rpy_list = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
        #self.rpy = np.array(rpy_list)
        #self.xy = np.array([lin.x, lin.y])


class UiCommsClient:
    def __init__(self):
        self.confidence_publisher = ZmqPublisher("*", "5556")
        self.map_publisher = ZmqPublisher("*", "5557")
        self.goal_subscriber = ZmqSubscriber("localhost", "5558", "goal")

    def send_confidence(self, confidence):
        self.confidence_publisher.publish(confidence)

    def canvas2rgb_array(self, canvas):
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

    def send_map(self, paths, obstacles, state, goal, play_area):
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
        plt_array = self.canvas2rgb_array(fig.canvas)
        string = base64.b64encode(cv2.imencode('.png', plt_array)[1]).decode()
        self.map_publisher.publish(string)

    def check_new_goal(self, current_goal, block=False):
        """
        Check for any new goals

        :param goal_sub:        The subscriber we receive goals on
        :param current_goal:    The current goal
        :param block:           True if should block on receive, false otherwise
        :return:                A new goal index if received, else the old goal index
        """
        print("Checking for new goal")
        try:
            if block:
                _new_goal = self.goal_subscriber.receive()
            else:
                _new_goal = self.goal_subscriber.receive(flags=zmq.NOBLOCK)
            _new_goal = int(_new_goal.split()[1])
            return _new_goal
        except:
            pass
        return current_goal


def control_loop():
    gazebo_interface = GazeboCommsClient()
    ui_interface = UiCommsClient()

    # Wait for everything to connect
    rospy.sleep(2)

    # Obstacles in the environment - can be updated in realtime
    obstacle_list = [(0, 2, 0.5), (0.5, 3, 0.8), (3, 7, 0.5), (0, 8, 0.3), (-2, 5, 1)]

    # The play area [minx, maxx, miny, maxy]
    play_area = [-5, 5, 0, 10]

    # State - can be updated in realtime
    jackal_state = gazebo_interface.get_state()
    print("initial state ", jackal_state)

    # List of goals - can be updated in realtime
    goals = [[0, 0], [-4, 7], [-1, 8], [2, 8]]

    # Wait for the first goal from the UI
    print("Waiting for first goal")
    goal_idx = ui_interface.check_new_goal(0, block=True)

    r = 0
    for i in range(15):
        """
        Check for any state updates
        """
        jackal_state = gazebo_interface.get_state()

        """
        Check for any goal updates
        """
        goal_idx = ui_interface.check_new_goal(goal_idx)

        """
        Do robot GOA calculations and publish to the UI
        TODO implement this
        TODO event triggering
        """
        topic = "sc"
        scs = get_self_confidence(jackal_state, goals, obstacle_list, play_area)
        datastring = f"{topic} {scs[0]} {scs[1]} {scs[2]}"
        print("Publishing new self-confidence: ", datastring)
        ui_interface.send_confidence(datastring)

        """
        Do a planning rollout, create a new image of the plan, publish the image to the UI
        """
        print("state ", jackal_state)
        obstacles_to_plan_with = []
        for o in obstacle_list:
            if not outcome_assessment.checkInCircle(o[0], o[1], o[2], jackal_state[0], jackal_state[1]):
                obstacles_to_plan_with.append(o)
        waypoints = planner.rollout(1, jackal_state, goals[goal_idx], obstacles_to_plan_with, play_area)[0]
        ui_interface.send_map([waypoints], obstacle_list, jackal_state, goals[goal_idx], play_area)
        print("Publishing new map")

        """
        TODO Publish the next waypoint to ROS
        """
        if len(waypoints) >= 3:
            next_waypoint = waypoints[1]
            next_next_waypoint = waypoints[2]
            next_heading = 90 + math.atan2(next_waypoint[1] - next_next_waypoint[1],
                                           next_waypoint[0] - next_next_waypoint[0])
            # TODO this heading isn't right
            r = gazebo_interface.send_waypoint(next_waypoint[0], next_waypoint[1], next_heading)
            print("Next waypoint: ({:.2f}, {:.2f}), heading: {:.2f}".format(*next_waypoint, next_heading))
        else:
            break

        """
        For now - magically teleport to the next waypoint
        TODO get updated state from ROS
        TODO get updated obstacles from ROS
        """
        #jackal_state = np.array(next_waypoint)
        rospy.sleep(2)
    return r


if __name__ == "__main__":
    try:
        rospy.init_node('movebase_client_py')
        result = control_loop()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
