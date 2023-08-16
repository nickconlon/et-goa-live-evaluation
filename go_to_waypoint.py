#!/usr/bin/env python
# Import necessary libraries
import rospy  # Ros library
import math  # Math library
import sys
import numpy as np
from et_goa import et_goa
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
    elif type(data) == Odometry:
        x_pose = data.pose.pose.position.x  # Current x-position of the Jackal
        y_pose = data.pose.pose.position.y  # Current y-position of the Jackal
        z_pose = data.pose.pose.position.z  # Current z-position of the Jackal
        pose = (x_pose, y_pose, z_pose)
        orientation_q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                         data.pose.pose.orientation.z, data.pose.pose.orientation.w]
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
        self.MQ = 0
        # Initialize ros node
        rospy.init_node('go_to_waypoint', anonymous=True)
        # Set sleep rate
        self.sleep_rate = rospy.Rate(10)

        # Initialize odometry subscriber
        # grab_odom = rospy.Subscriber('/odometry/filtered/aspen', Odometry, calculate_heading)
        self.grab_odom = rospy.Subscriber('/gazebo/model_states', ModelStates, self.calculate_heading)

        # Initialize command velocity publisher
        self.pub_vel = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)

    def calculate_heading(self, data):
        """
        PD control for waypoint following
        """
        pose, angle = extract_msg(data)
        (x_pose, y_pose, z_pose) = pose[0], pose[1], pose[2]
        (y_rot, x_rot, z_rot) = angle[0], angle[1], angle[2]

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
        if abs(angle_err) > 0.05:  # Only rotate
            self.rot_cmd_vel = np.minimum(angle_err * self.K1, 0.5)
            self.pose_cmd_vel = 0
        else:  # Rotate and move
            self.rot_cmd_vel = np.minimum(angle_err * self.K1, 0.5)
            self.pose_cmd_vel = np.minimum(self.dist_err * self.K2, 0.5)

        if self.pose_cmd_vel > 1.0:
            self.pose_cmd_vel = 1.0

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
        self.x_dest = _x
        self.y_dest = _y
        ###
        self.et_object = et_goa()
        pred_paths = ['/data/webots/rollout{}_state.npy'.format(x) for x in np.arange(0, 10)]
        self.et_object.set_pred_paths(pred_paths)
        self.et_object.preprocess()
        goa = self.et_object.get_goa_times(self.goa_threshold, 0)
        ###

        # Go to waypoint
        while self.dist_err > 0.1:
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
        #wp.reset()
    except rospy.ROSInterruptException:
        pass
