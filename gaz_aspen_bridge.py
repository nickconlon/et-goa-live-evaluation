import rospy
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates


rospy.init_node('bridgers', anonymous=True)

pose_pub = rospy.Publisher('/tars/vrpn_client_node/cohrint_tars/pose', PoseStamped, queue_size=10)

def pose_callback(msg):
    #print('pose callback')
    def extract_msg(data):
        """
        Extract the same fields from different messages
        """
        if type(data) == ModelStates:
            model_idx = data.name.index('jackal')
            lin = data.pose[model_idx].position
            ang = data.pose[model_idx].orientation
            return lin, ang

    p, a = extract_msg(msg)
    msg = PoseStamped()
    msg.pose.position = p
    msg.pose.orientation = a
    pose_pub.publish(msg)

pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, pose_callback)

cmd_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
def cmd_callback(msg):
    cmd_pub.publish(msg)

cmd_sub = rospy.Subscriber('/tars/jackal_velocity_controller/cmd_vel', Twist, cmd_callback)



rospy.spin()
