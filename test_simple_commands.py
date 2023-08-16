import numpy as np
import base64
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import zmq
import math

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

import outcome_assessment
from outcome_assessment import get_self_confidence
from zmq_publisher import ZmqPublisher
from zmq_subscriber import ZmqSubscriber
import rrt_planner as planner
import simulator

plt.switch_backend('Agg')


class RobotCommsClient:
    #SIM_TYPE = "ASPEN"
    SIM_TYPE = "GAZEBO"
    ASPEN = "ASPEN"
    GAZEBO = "GAZEBO"
    GAZEBO_STATE_TOPIC = '/jackal_velocity_controller/odom'
    CMD_VEL_TOPIC = '/jackal_velocity_controller/cmd_vel'
    GAZEBO_FRAME_ID = 'odom'
    ASPEN_STATE_TOPIC = '/tars/vicon_pose'
    ASPEN_FRAME_ID = 'odom'#'aspen_odom'

    def __init__(self):
        if RobotCommsClient.SIM_TYPE == RobotCommsClient.ASPEN:
            self.state_topic = RobotCommsClient.ASPEN_STATE_TOPIC
            self.frame_id = RobotCommsClient.ASPEN_FRAME_ID
            self.state_msg = PoseWithCovarianceStamped
        else:
            self.state_topic = RobotCommsClient.GAZEBO_STATE_TOPIC
            self.frame_id = RobotCommsClient.GAZEBO_FRAME_ID
            self.state_msg = Odometry
        print("Starting client for ", RobotCommsClient.SIM_TYPE)
        #self.move_base_srv = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.move_base_srv.wait_for_server()
        print("MOVE BASE connected")
        self.xy = np.zeros(2)
        self.rpy = np.zeros(3)
        #self.gazebo_state = rospy.Subscriber("/gazebo/model_states", ModelStates, self._gazebo_callback)
        #self.odom_state = rospy.Subscriber("/jackal_velocity_controller/odom", Odometry, self._odom_callback)
        #self.odom_state = rospy.Subscriber("/tars/odometry/filtered/aspen", Odometry, self._odom_callback)
        self.odom_state = rospy.Subscriber(self.state_topic, self.state_msg, self._state_callback)
        self.cmd_vel_pub = rospy.Publisher(self.CMD_VEL_TOPIC, Twist, queue_size=10)

    def send_waypoint(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = RobotCommsClient.ASPEN_FRAME_ID
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, np.deg2rad(theta))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base_srv.send_goal(goal)
        rospy.sleep(5)
        #wait = self.move_base_srv.wait_for_result()
        #if not wait:
        #   rospy.logerr("Action server not available!")
        #   rospy.signal_shutdown("Action server not available!")
        #else:
        #   return self.move_base_srv.get_result()
        return 1

    def send_waypoint_and_wait(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = RobotCommsClient.ASPEN_FRAME_ID
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, np.deg2rad(theta))
        print('target ', q)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base_srv.send_goal(goal)
        rospy.sleep(5)
        wait = self.move_base_srv.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            #rospy.logger("at waypoint")
            print("at waypoint")
            return self.move_base_srv.get_result()
        return 1

    def send_cmd_vel(self, vel):
        msg = Twist()
        msg.linear.x = vel
        print(msg.linear)
        self.cmd_vel_pub.publish(msg)

    def get_state(self):
        return self.xy

    def _state_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.xy = np.array([x, y])

    def _gazebo_callback(self, msg):
        model_idx = msg.name.index('jackal')
        lin = msg.pose[model_idx].position
        ang = msg.pose[model_idx].orientation
        rpy_list = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
        self.rpy = np.array(rpy_list)
        self.xy = np.array([lin.x, lin.y])


class UiCommsClient:
    def __init__(self):
        self.confidence_publisher = ZmqPublisher("*", "5556")
        self.map_publisher = ZmqPublisher("*", "5557")
        self.goal_subscriber = ZmqSubscriber("localhost", "5558", "goal")

    def send_confidence(self, confidence):
        self.confidence_publisher.publish(confidence)

    @staticmethod
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

    def send_map(self, paths, obstacles, state, goal, goals, play_area):
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
            p = patches.Circle((x, y), radius=size, facecolor='black')
            ax.add_artist(p)

        for (x, y) in goals:
            p = patches.Circle((x, y), radius=0.5, facecolor='orange', ec='black')
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
        plt_array = self.canvas2rgb_array(fig.canvas)
        string = base64.b64encode(cv2.imencode('.png', plt_array)[1]).decode()
        self.map_publisher.publish(string)

    def check_new_goal(self, current_goal, block=False):
        """
        Check for any new goals

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
    gazebo_interface = RobotCommsClient()

    # Wait for everything to connect
    rospy.sleep(2)


    for i in range(20):
        jackal_state = gazebo_interface.get_state()
        if abs(jackal_state[0] - 0) < 0.1:
            break

        gazebo_interface.send_cmd_vel(-1)
        print("Jackal state: ({:2f}, {:2f})".format(jackal_state[0], jackal_state[1]))
        rospy.sleep(0.25)
    return 0


if __name__ == "__main__":
    try:
        rospy.init_node('movebase_client_py')
        result = control_loop()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
