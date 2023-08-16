import json
import traceback
import sys
import time
import numpy as np
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5 import QtCore, QtGui
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import comms as comms
from et_goa import et_goa

import ros_ui
import rrt

import rospy
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates
import go_to_waypoint

class WaypointThread(QtCore.QThread):
    #sub = comms.ZmqSubscriber("localhost", "5558")
    mq_emit = QtCore.pyqtSignal(object)
    task_time_emit = QtCore.pyqtSignal(int)
    done_emit = QtCore.pyqtSignal()

    waypoints = []
    should_run = True
    #xx = 2.5
    #yy = 4

    def run(self):
        t1 = time.time()
        gtw = go_to_waypoint.WaypointFollower()
        print('starting video thread')
        _xs = self.waypoints[:,0]
        _ys = self.waypoints[:,1]
        for _x, _y in zip(_xs, _ys):
            if not self.should_run:
                break
            print('going to wp {}, {}'.format(_x, _y))
            gtw.dist_err = 1.0
            gtw.x_dest = _x
            gtw.y_dest = _y

            # Go to waypoint
            while gtw.dist_err > 0.1 and self.should_run:
                gtw.do_et = True
                self.mq_emit.emit([gtw.MQ, gtw.predx, gtw.predy])
                if gtw.pose_cmd_vel is not None:
                    vel = Twist()
                    vel.linear.x = gtw.pose_cmd_vel
                    vel.angular.z = gtw.rot_cmd_vel
                    gtw.pub_vel.publish(vel)
                gtw.sleep_rate.sleep()
            gtw.do_et = False
        gtw.grab_odom.unregister()
        gtw.grab_odom = None
        t2 = time.time()
        rospy.sleep(0.25)
        self.task_time_emit.emit(int(abs(t2-t1)))
        print('task time', abs(t2-t1))
        self.done_emit.emit()
        print('exiting waypoint thread')
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
    else:
        x_pose = data.pose.position.x  # Current x-position of the Jackal
        y_pose = data.pose.position.y  # Current y-position of the Jackal
        z_pose = data.pose.position.z  # Current z-position of the Jackal
        pose = (x_pose, y_pose, z_pose)
        orientation_q = [data.pose.orientation.x, data.pose.orientation.y,
                         data.pose.orientation.z, data.pose.orientation.w]
        # Convert quaternion to euler angles
        angle = euler_from_quaternion(orientation_q)
    return pose, angle

def assessment_to_label(raw_assessment):
    if raw_assessment < 0.25:
        return 'very bad', 'red'
    elif raw_assessment < 0.4:
        return 'bad', '#C34A2C'
    elif raw_assessment < 0.6:
        return 'fair', 'yellow'
    elif raw_assessment < 0.75:
        return 'good', '#6CC417'
    else:
        return 'very good', 'green'

class VideoThread(QtCore.QThread):
    #sub = comms.ZmqSubscriber("localhost", "5558")
    image_emit = QtCore.pyqtSignal(object)
    #ar = aruco_read.ArUco()
    should_run = True

    def run(self):
        print('starting video thread')
        while self.should_run:
            try:
                pass
                #img, det_id = self.ar.detect()
                #if det_id is not None:
                #    self.image_emit.emit(det_id)
            except Exception as e:
                traceback.print_exc()
        #self.ar.close()
        print('exiting video thread')


class ControlThread(QtCore.QThread):
    sub = comms.ZmqSubscriber("localhost", "5558")

    should_run = True

    def run(self):
        print('starting control thread')
        while self.should_run:
            try:
                try:
                    stat = self.sub.receive(flags=comms.NOBLOCK)
                    stat = json.loads(stat)
                    pose = stat["position"]
                    orient = stat["orientation"]
                    update = [*pose, 0, 0, orient[-1]]
                    self.state_emit.emit(update)
                    self.mq_emit.emit(pose)
                except:
                    pass
            except Exception as e:
                traceback.print_exc()
        # TODO close the socket stuff
        print('exiting control thread')


class StateUpdateThread(QtCore.QThread):
    sub = comms.ZmqSubscriber("localhost", "5558")

    state_emit = QtCore.pyqtSignal(object)
    mq_emit = QtCore.pyqtSignal(object)

    should_run = True

    def run(self):
        print('starting state thread')
        while self.should_run:
            try:
                try:
                    stat = self.sub.receive(flags=comms.NOBLOCK)
                    stat = json.loads(stat)
                    pose = stat["position"]
                    orient = stat["orientation"]
                    update = [*pose, 0, 0, orient[-1]]
                    self.state_emit.emit(update)
                    self.mq_emit.emit(pose)
                except:
                    pass
                # print("state control stuff")
                # self.progress.emit(np.random.normal(size=6))
            except Exception as e:
                traceback.print_exc()
        print('exiting state thread')


class RolloutThread(QtCore.QThread):
    finished = QtCore.pyqtSignal()
    pose = None
    orientation = None
    goal = None
    waypoints = []

    def run(self):
        print('starting rollout thread')
        try:
            comms.do_rollout(self.pose, self.orientation, self.goal, 0, [], self.waypoints)
            self.finished.emit()
        except Exception as e:
            traceback.print_exc()
        print('exiting rollout thread')


def plan(goal_pos, robot_pos, obstacle_pos, ret='all'):
    """
    The RRT planner is weird and takes everything in (y, x)..
    """
    # RRT goal = [y, x]
    rrt_goal = np.flip(goal_pos)

    # RRT point [y, x]
    starting_point = np.array([robot_pos[1], robot_pos[0]])

    # [ymin ymax], [xmin, xmax]
    bounds = np.array([[1, 10], [1, 5]])

    waypoints = rrt.plan_rrt_webots(starting_point, rrt_goal, obstacle_pos, bounds, visualize_route=False)
    if len(waypoints) == 0:
        _ret_waypoint = []
    elif ret == 'next':
        _ret_waypoint = waypoints[0]
    else:
        _ret_waypoint = waypoints

    print("starting at: " + str(starting_point))
    print("going to: " + str(goal_pos))
    print("next waypoint " + str(_ret_waypoint))
    return _ret_waypoint


class myMainWindow(QMainWindow, ros_ui.Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        rospy.init_node('interface', anonymous=True)
        self.setupUi(self)
        self.generate_plan_button.clicked.connect(self.generate_plan_callback)
        self.update_goal_button.clicked.connect(self.update_goal_callback)
        self.run_goa.clicked.connect(self.run_assessment_callback)
        self.update_goa.clicked.connect(self.update_goa_callback)
        self.stop_robot.clicked.connect(self.stop_robot_callback)
        self.start_robot.clicked.connect(self.start_robot_callback)

        self.wp_thread = None

        # parameters and storage for runtime stuff
        self.goa_threshold = 61
        self.goal = [3, 6]
        self.pose = [1.5, 1.5, 0.0001]
        self.orientation = [0, 0, 1, 0]
        self.et_counter = 0
        self.pose_counter = 0
        self.predx = 0
        self.predy = 0
        self.et_object = et_goa()
        self.waypoints = [] #np.array([[3, 1],[2.5, 4],[3, 6]])

        self.update_goa_value.setText(str(self.goa_threshold))
        self.update_map_callback()

        #self.grab_odom = rospy.Subscriber('/gazebo/model_states', ModelStates,
        #                                  self.pose_callback)
        self.grab_odom = rospy.Subscriber('/tars/vrpn_client_node/cohrint_tars/pose', PoseStamped, self.pose_callback)

        self.start_time = time.time()
        self.x = 0
        self.y = 0
        pred_paths = ['/data/webots/rollout{}_state.npy'.format(x) for x in np.arange(0, 10)]
        self.et_object.set_pred_paths(pred_paths)
        self.et_object.preprocess()

        self.solver_quality_callback()
        self.experience_callback()

    def stop_robot_callback(self):
        if self.wp_thread is not None:
            self.wp_thread.should_run = False

    def robot_graceful_stop(self):
        if self.wp_thread is not None:
            self.wp_thread.quit()
            self.wp_thread.wait()
            self.wp_thread = None
            self.waypoints = []

    def start_robot_callback(self):
        self.wp_thread = WaypointThread()
        self.wp_thread.xx = self.goal[0]
        self.wp_thread.yy = self.goal[1]
        self.wp_thread.waypoints = self.waypoints
        self.wp_thread.mq_emit.connect(self.model_quality_callback)
        self.wp_thread.done_emit.connect(self.robot_graceful_stop)
        self.wp_thread.task_time_emit.connect(self.task_time_callback)
        self.wp_thread.start()

    def task_time_callback(self, msg):
        try:
            self.actual_outcome_display.setText(str(msg))
        except Exception as e:
            traceback.print_exc()

    def waypoint_callback(self, msg):
        try:
            wp_str = ','.join([str(x) for x in msg])
            print('Waypoint str', wp_str)
            self.next_waypoint_display.setText(wp_str)
        except Exception as e:
            traceback.print_exc()

    @staticmethod
    def convert_msg(msg):
        x, y, z, rx, ry, rz = msg  # np.random.normal(size=6)
        return x, y, z, rx, ry, rz, time.time()

    def pose_callback(self, msg):
        self.pose_counter += 1
        if self.pose_counter % 100 == 0:
            try:
                pose, angle = extract_msg(msg)
                x, y, z = pose
                rx, ry, rz = angle
                t = time.time()-self.start_time
                pos_str = "({:.2f}, {:.2f}, {:.2f})".format(x, y, z)
                orient_str = "({:.2f}, {:.2f}, {:.2f}, {:.2f})".format(rx, ry, rz, 1)
                #print('{} pose '.format(self.pose_counter), pos_str)
                time_str = "{:.2f}".format(t)
                self.position_display.setText(pos_str)
                self.orientation_display.setText(orient_str)
                self.time_display.setText(time_str)
                self.pose = np.array([x, y, z])
                self.orientation = np.array([0, 0, 1, rz])
                #if self.et_counter % self.et_object.sample_rate == 0:
                self.update_map_callback()
                app.processEvents()
            except Exception as e:
                traceback.print_exc()


    def run_assessment_callback(self):
        try:
            if len(self.waypoints) > 0:
                print('doing rollout')
                self.rollout = RolloutThread()
                self.rollout.pose = self.pose
                self.rollout.orientation = self.orientation
                self.rollout.goal = self.goal
                self.rollout.waypoints = self.waypoints
                self.rollout.finished.connect(self.update_goa_callback)
                self.rollout.start()
                #self.update_goa_callback()
        except Exception as e:
            traceback.print_exc()

    def update_goa_callback(self):
        try:
            self.thread = None
            self.rollout = None
            if len(self.waypoints) > 0:
                self.goa_threshold = float(self.update_goa_value.text())
                pred_paths = ['/data/webots/rollout{}_state.npy'.format(x) for x in np.arange(0, 10)]
                self.et_object.set_pred_paths(pred_paths)
                self.et_object.preprocess()
                goa = self.et_object.get_goa_times(self.goa_threshold, 0)
                label, color = assessment_to_label(goa)
                self.goa_display.setText(label)
                self.goa_display.setStyleSheet('background-color: {}'.format(color))
        except Exception as e:
            traceback.print_exc()

    def update_goal_callback(self):
        try:
            x = float(self.goal_x_value.text())
            y = float(self.goal_y_value.text())
            self.goal = [x, y]
            self.update_map_callback()
        except:
            print('exception while updating goal')

    def generate_plan_callback(self):
        try:
            self.waypoints = plan(self.goal, self.pose, [rrt.Obstacle(rrt.Obstacle.circle, [2.5, 2.5], [0.5], '1')])
            print(self.waypoints)
            self.update_map_callback()
        except Exception as e:
            traceback.print_exc()

    def update_map_callback(self):
        try:
            #print('updating the map!')
            img = plot_map(self.pose[0:2], self.goal, self.waypoints, self.predx, self.predy)
            img = img.astype(np.uint8)
            height, width, channel = img.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(img.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            QtGui.QPixmap.fromImage(qImg)
            self.label.setPixmap(QtGui.QPixmap.fromImage(qImg))
            # plt.imshow(img)
            # plt.show()
            # self.label.setPixmap(QtGui.QPixmap("basemap.png"))
        except Exception as e:
            traceback.print_exc()

    def model_quality_callback(self, msg):
        if self.et_counter % self.et_object.sample_rate == 0:
            si, self.predx, self.predy = msg
            if si >= 0 and self.wp_thread is not None:
                label, color = assessment_to_label(si)
                self.et_display.setText(label)
                self.et_display.setStyleSheet('background-color: {}'.format(color))
        self.et_counter += 1

    def solver_quality_callback(self):
        label, color = assessment_to_label(1)
        self.sq_display.setText('Not implemented')
        self.sq_display.setStyleSheet('background-color: {}'.format('blue'))
    def experience_callback(self):
        label, color = assessment_to_label(0.5)
        self.exp_display.setText('Not implemented')
        self.exp_display.setStyleSheet('background-color: {}'.format('blue'))


def plot_map(position, goal, waypoints, predx=None, predy=None):
    fig, ax = plt.subplots(figsize=(3, 3))
    if len(waypoints) > 0:
        # plt.scatter([position[0]], [position[1]], color='black')
        ax.plot(waypoints[:, 0], waypoints[:, 1], color='black')
        ax.scatter(waypoints[:, 0], waypoints[:, 1], color='black')
    if goal is not None:
        plt.scatter(*goal, color='green')
    if predx is not None:
        ax.scatter([predx], [predy], color='orange')
    ax.scatter(*position, color='blue', label='robot pose')
    ax.set_ylim([0, 10.1])
    ax.set_xlim([0, 10.1])
    ax.set_yticks([0, 2, 4, 6, 8, 10])
    ax.set_xticks([0, 2, 4, 6, 8, 10])
    ax.set_ylabel('y')
    ax.set_xlabel('x')
    rect = patches.Rectangle((1, 1), 4, 9, linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(rect)
    plt.tight_layout()
    fig.canvas.draw()
    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    img = img.reshape(*reversed(fig.canvas.get_width_height()), 3)
    # plt.imshow(img)
    # plt.show()
    # plt.savefig('basemap.png')
    plt.close('all')
    return img


try:
    app = QApplication(sys.argv)
    MainWindow = myMainWindow()
    MainWindow.show()
    app.exec_()
except Exception as e:
    print(e)
    traceback.print_exc()
