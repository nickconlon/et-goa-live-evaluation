#!/usr/bin/env python
# Import necessary libraries
import time
import traceback

import rospy  # Ros library
import math  # Math library
import sys
import numpy as np
from et_goa import et_goa
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist  # Twist messages
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates

def extract_msg(data):
    """
    Extract the same fields from different messages
    """
    if type(data) == ModelStates:
        model_idx = data.name.index('jackal')
        lin = data.pose[model_idx].position
        ang = data.pose[model_idx].orientation
        pose = (lin.x, lin.y, lin.z)
        angle = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
        return pose, angle
    elif type(data) == PoseStamped:
        x_pose = data.pose.position.x  # Current x-position of the Jackal
        y_pose = data.pose.position.y  # Current y-position of the Jackal
        z_pose = data.pose.position.z  # Current z-position of the Jackal
        pose = (x_pose, y_pose, z_pose)
        orientation_q = [data.pose.orientation.x, data.pose.orientation.y,
                         data.pose.orientation.z, data.pose.orientation.w]
        # Convert quaternion to euler angles
        angle = euler_from_quaternion(orientation_q)
        return pose, angle


class WaypointFollower:
    def __init__(self):
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
        ###
        self.et_object = et_goa()
        pred_paths = ['/data/webots/rollout{}_state.npy'.format(x) for x in np.arange(0, 10)]
        self.et_object.set_pred_paths(pred_paths)
        self.et_object.preprocess()
        self.et_object.sample_rate = 20
        goa = self.et_object.get_goa_times(60, 0)
        print('GOA', goa)
        self.do_et = False
        self.MQ = -1
        self.predx = -1
        self.predy = -1
        ###

        # Initialize ros node
        #rospy.init_node('go_to_waypoint', anonymous=True)
        # Set sleep rate
        self.sleep_rate = rospy.Rate(10)

        # Initialize odometry subscriber
        self.grab_odom = rospy.Subscriber('/tars/vrpn_client_node/cohrint_tars/pose', PoseStamped, self.calculate_heading)
        #self.grab_odom = rospy.Subscriber('/gazebo/model_states', ModelStates, self.calculate_heading)

        # Initialize command velocity publisher
        #self.pub_vel = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.pub_vel = rospy.Publisher('/tars/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)

    def calculate_heading(self, data):
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
                    self.MQ, self.predx, self.predy = self.et_object.get_si(self.xx, self.yy, 0)
                except Exception as e:
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
        self.dist_err = math.sqrt((x_err ** 2) + (y_err ** 2))

        # Calculate command velocities
        max_speed = 0.25
        if abs(angle_err) > 0.05:  # Only rotate
            if angle_err >=0:
                self.rot_cmd_vel = np.minimum(angle_err * self.K1, max_speed)
            else:
                self.rot_cmd_vel = np.maximum(angle_err * self.K1, -max_speed)
            self.pose_cmd_vel = 0
        else:  # Rotate and move
            #self.rot_cmd_vel = np.minimum(angle_err * self.K1, max_speed)
            if angle_err >=0:
                self.rot_cmd_vel = np.minimum(angle_err * self.K1, max_speed)
            else:
                self.rot_cmd_vel = np.maximum(angle_err * self.K1, -max_speed)
            self.pose_cmd_vel = np.minimum(self.dist_err * self.K2, max_speed)


    def reset(self):
        """
        Reset the parameters
        """
        self.x_dest = None
        self.y_dest = None
        self.pose_cmd_vel = None
        self.rot_cmd_vel = None
        self.dist_err = 1
    def go_to_waypoint(self, _x, _y):
        """
        Navigate to a waypoint at (_x, _y)
        """
        t_timeout = 1
        self.x_dest = _x
        self.y_dest = _y

        self.do_et = True
        # Go to waypoint
        while self.dist_err > 0.1:
            # TODO test me - stop on loss of connection
            if abs(self.last_time-time.time()) < t_timeout:
                vel = Twist()
                vel.linear.x = self.pose_cmd_vel
                vel.angular.z = self.rot_cmd_vel
                self.pub_vel.publish(vel)
            self.sleep_rate.sleep()


if __name__ == '__main__':
    try:
        wp = WaypointFollower()
        x_dest = float(sys.argv[1])
        y_dest = float(sys.argv[2])
        wp.go_to_waypoint(x_dest, y_dest)
        wp.dist_err = 1
        wp.go_to_waypoint(3, 6)
    except rospy.ROSInterruptException:
        pass
