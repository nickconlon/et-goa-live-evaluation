import copy
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import pandas as pd
import ros_ui
import rospy
import rrt
from et_goa import et_goa
from famsec import assessment_to_label
from gazebo_msgs.msg import ModelStates
from navigation import plan, plot_map
from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow
from ros_conversion import extract_msg
from std_msgs.msg import String
from ui_threads import WaypointThread, WorldModelRolloutThread
from world_model import State, WebotsWorldModel


class ControlState:
    STOPPED = "Stopped"
    AUTONOMY = "Autonomy"
    HUMAN = "Human"


class myMainWindow(QMainWindow, ros_ui.Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        rospy.init_node("interface", anonymous=True)

        # Set up the UI components
        self.setupUi(self)
        self.generate_plan_button.clicked.connect(self.generate_plan_callback)
        self.update_goal_button.clicked.connect(self.update_goal_callback)
        self.run_goa.clicked.connect(self.run_assessment_callback)
        self.update_goa.clicked.connect(self.update_goa_callback)
        self.stop_robot_button.clicked.connect(self.stop_robot_callback)
        self.start_robot_button.clicked.connect(self.start_robot_callback)
        self.go_home_button.clicked.connect(self.go_home_callback)
        self.famsec_toggle_button.clicked.connect(self.toggle_famsec_callback)
        self.et_goa_toggle_button.clicked.connect(self.toggle_triggering_callback)
        self.write_data_button.clicked.connect(self.write_data_callback)
        self.toggle_record_button.clicked.connect(self.toggle_record_callback)
        self.teleop_robot_button.clicked.connect(self.teleop_callback)
        self.clear_obstacles_button.clicked.connect(self.clear_obstacles_callback)
        self.test_add_obstacle_button.clicked.connect(self.test_add_obstacle_callback)
        self.test_trigger_goa_button.clicked.connect(self.test_trigger_goa_callback)
        self.end_sim.clicked.connect(self.hide_buttons_callback)

        self.wp_thread = None
        self.rollout_thread = None

        # parameters and storage for runtime stuff
        self.state = State(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([0.0, 0.0, 0.0, 0.0]),
            time=0.0,
            goal=np.array([0, 0]),
            known_obstacles={},
            waypoints=np.empty(0),
        )

        self.wm_rollout_data_path = Path(__file__).parent.parent / "rollouts"
        self.world_model = WebotsWorldModel(self.wm_rollout_data_path)

        self.goa_threshold = 60
        self.et_counter = 0
        self.pose_counter = 0
        self.predx = -1
        self.predy = -1
        self.et_object = et_goa(self.world_model)

        # Some metrics we'll record
        self.model_quality_over_time = []
        self.outcome_assessment_over_time = []
        self.orientation_over_time = []
        self.time_over_time = []
        self.pose_over_time = []
        self.should_record = False

        # 0 => off, 1 => on
        self.FaMSeC_state = 0
        self.ET_GOA_state = 0

        self.control_state = ControlState.STOPPED
        self.outcome_assessment = -1
        self.model_quality_assessment = -1
        self.current_time = -1
        self.current_waypoint = 0
        self.scenario_data = []

        self.delta = 0.05  # TODO triggering threshold

        self.update_goa_value.setText(str(self.goa_threshold))
        self.update_map_callback()

        self.grab_odom = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.pose_callback
        )
        self.image_signal = rospy.Subscriber(
            "/goa/image_signal", String, self.image_signal_callback
        )

        self.start_time = time.time()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_timer_callback)
        self.timer.setInterval(10)
        self.timer.start()

    def hide_buttons_callback(self):
        self.exp_display.hide()
        self.sq_display.hide()
        self.exp_label.hide()
        self.sq_label.hide()
        self.actual_outcome_display.hide()
        self.actual_oa_label.hide()

    def clear_obstacles_callback(self):
        self.state.known_obstacles = {}

    def teleop_callback(self):
        if self.control_state == ControlState.HUMAN:
            self.control_state = ControlState.STOPPED
            self.teleop_robot_button.setStyleSheet(
                "background-color: {}".format("light gray")
            )
        else:
            self.control_state = ControlState.HUMAN
            self.teleop_robot_button.setStyleSheet(
                "background-color: {}".format("green")
            )

    def toggle_record_callback(self):
        self.record(not self.should_record)

    def record(self, state):
        if state:
            self.toggle_record_button.setStyleSheet(
                "background-color: {}".format("green")
            )
        else:
            self.toggle_record_button.setStyleSheet(
                "background-color: {}".format("light gray")
            )
        self.should_record = state

    def test_add_obstacle_callback(self):
        obs = "BOX1"
        pos = self.state.position
        ori = self.state.orientation[-1]
        loc = [pos[0] + 2 * np.cos(ori), pos[1] + 2 * np.sin(ori)]
        self.state.known_obstacles[obs] = np.array([float(loc[0]), float(loc[1])])
        print(self.state.known_obstacles)

    def test_trigger_goa_callback(self):
        self.stop_robot_callback()
        rospy.sleep(1)
        self.run_assessment_callback()

    def image_signal_callback(self, msg):
        ids = msg
        print("received ids", ids)
        self.test_add_obstacle_callback()
        if self.wp_thread is not None:
            self.wp_thread.should_drive = False

    def write_data_callback(self):
        self.record(False)
        rospy.sleep(0.01)  # let the last recording occur
        data = self.scenario_data
        columns = ["t", "Xm", "Xo", "px", "py", "pz", "state"]

        filename = "data.csv"
        pd.DataFrame(data, columns=columns).to_csv(filename, index=False)
        self.scenario_data = []
        pass

    def go_home_callback(self):
        # Set the goal a little past home so it's easier to
        # turn the robot around.
        self.state.goal = np.array([2.0, 1.5])
        self.generate_plan_callback()
        self.state.goal = np.array([2.0, 2.0])

    def toggle_famsec_callback(self):
        self.FaMSeC_state = not self.FaMSeC_state
        if self.FaMSeC_state:
            self.solver_quality_callback()
            self.experience_callback()
            self.outcome_assessment_callback()
            self.famsec_toggle_button.setStyleSheet(
                "background-color: {}".format("green")
            )
        else:
            self.sq_display.setStyleSheet("background-color: {}".format("light gray"))
            self.exp_display.setStyleSheet("background-color: {}".format("light gray"))
            self.goa_display.setStyleSheet("background-color: {}".format("light gray"))
            self.et_display.setStyleSheet("background-color: {}".format("light gray"))
            self.goa_display.setText("")
            self.sq_display.setText("")
            self.exp_display.setText("")
            self.et_display.setText("")
            self.famsec_toggle_button.setStyleSheet(
                "background-color: {}".format("light gray")
            )

    def toggle_triggering_callback(self):
        self.ET_GOA_state = not self.ET_GOA_state
        if self.ET_GOA_state:
            self.et_goa_toggle_button.setStyleSheet(
                "background-color: {}".format("green")
            )
        else:
            self.et_goa_toggle_button.setStyleSheet(
                "background-color: {}".format("light gray")
            )

    def stop_robot_callback(self):
        if self.wp_thread is not None:
            self.wp_thread.should_run = False

    def robot_graceful_stop(self):
        if self.wp_thread is not None:
            self.wp_thread.quit()
            self.wp_thread.wait()
            self.wp_thread = None
        self.start_robot_button.setStyleSheet(
            "background-color: {}".format("light gray")
        )
        self.control_state = ControlState.STOPPED

    def start_robot_callback(self):
        self.start_robot_button.setStyleSheet("background-color: {}".format("green"))
        self.control_state = ControlState.AUTONOMY

        self.wp_thread = WaypointThread()
        self.wp_thread.world_model = self.world_model
        self.wp_thread.waypoints = self.state.waypoints  # from the path solver
        self.wp_thread.mq_emit.connect(self.model_quality_callback)
        self.wp_thread.done_emit.connect(self.robot_graceful_stop)
        self.wp_thread.task_time_emit.connect(self.task_time_callback)
        self.wp_thread.waypoint_emit.connect(self.waypoint_callback)
        self.wp_thread.start()

    def task_time_callback(self, msg):
        try:
            self.actual_outcome_display.setText(str(msg))
        except Exception:
            traceback.print_exc()

    def record_timestep(self, t, Xm, Xo, px, py, pz, state):
        self.scenario_data.append([t, Xm, Xo, px, py, pz, state])

    def update_timer_callback(self):
        try:
            t = time.time() - self.start_time
            pos_str = "({:.1f}, {:.1f}, {:.1f})".format(*self.state.position)
            orient_str = "{:.1f} degrees".format(np.rad2deg(self.state.orientation[-1]))
            time_str = "{:.2f} seconds".format(t)
            self.position_display.setText(pos_str)
            self.orientation_display.setText(orient_str)
            self.time_display.setText(time_str)
            self.update_map_callback()
            if self.should_record:
                self.record_timestep(
                    t,
                    self.model_quality_assessment,
                    self.outcome_assessment,
                    *self.state.position,
                    self.control_state,
                )

            if len(self.state.waypoints) > 0:
                wp_x, wp_y = self.state.waypoints[self.current_waypoint]
            else:
                wp_x, wp_y = 0, 0
            wp_str = "({:.1f}, {:.1f})".format(wp_x, wp_y)
            self.next_waypoint_display.setText(wp_str)
        except Exception:
            traceback.print_exc()

    def waypoint_callback(self, msg):
        try:
            if len(self.state.waypoints) > 0:
                self.state.waypoints = self.state.waypoints[
                    1:
                ]  # just ignore the first one for viz.
        except Exception:
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
                self.current_time = time.time() - self.start_time
                self.state.position = np.array([x, y, z])
                self.state.orientation = np.array([0, 0, 1, rz])
                app.processEvents()
            except Exception:
                traceback.print_exc()

    def run_assessment_callback(self):
        print("Simulating {} waypoints".format(len(self.state.waypoints)))
        try:
            if len(self.state.waypoints) > 0 and self.FaMSeC_state:
                self.run_goa.setStyleSheet("background-color: {}".format("green"))
                print("doing rollout")
                current_state = copy.deepcopy(self.state)
                current_path = np.array(self.state.waypoints)
                self.rollout_thread = WorldModelRolloutThread(
                    current_state, current_path, self.world_model.generate_samples
                )
                self.rollout_thread.finished.connect(self.update_goa_callback)
                self.rollout_thread.start()
        except Exception:
            traceback.print_exc()

    def update_goa_callback(self):
        try:
            if len(self.state.waypoints) > 0 and self.FaMSeC_state:
                self.goa_threshold = float(self.update_goa_value.text())
                goa = self.et_object.get_goa_times(self.goa_threshold)
                print("GOA ", goa)
                label, color = assessment_to_label(goa)
                self.outcome_assessment = goa
                self.goa_display.setText(label)
                self.goa_display.setStyleSheet("background-color: {}".format(color))
                self.et_display.setText("")
                self.et_display.setStyleSheet(
                    "background-color: {}".format("light gray")
                )
                self.run_goa.setStyleSheet("background-color: {}".format("light grey"))
        except Exception:
            traceback.print_exc()

    def update_goal_callback(self):
        try:
            x = float(self.goal_x_value.text())
            y = float(self.goal_y_value.text())
            self.state.goal = np.array([x, y])
            self.update_map_callback()
        except:
            print("exception while updating goal")

    def generate_plan_callback(self):
        try:
            self.generate_plan_button.setStyleSheet(
                "background-color: {}".format("green")
            )
            self.current_waypoint = 0
            obs = [
                rrt.Obstacle(rrt.Obstacle.circle, (v[1], v[0]), [0.5], "1")
                for k, v in self.state.known_obstacles.items()
            ]
            print(obs)
            self.state.waypoints = plan(self.state.goal, self.state.position, obs)
            print(self.state.waypoints)
            self.update_map_callback()
            self.generate_plan_button.setStyleSheet(
                "background-color: {}".format("light grey")
            )
        except Exception:
            traceback.print_exc()

    def update_map_callback(self):
        try:
            # print('updating the map!')
            img = plot_map(
                self.state.position[0:2],
                self.state.goal,
                self.state.waypoints,
                self.state.known_obstacles,
                self.predx,
                self.predy,
            )

            img = img.astype(np.uint8)
            height, width, channel = img.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(
                img.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888
            )
            QtGui.QPixmap.fromImage(qImg)
            self.label.setPixmap(QtGui.QPixmap.fromImage(qImg))
            # plt.imshow(img)
            # plt.show()
            # self.label.setPixmap(QtGui.QPixmap("basemap.png"))
        except Exception:
            traceback.print_exc()

    def model_quality_callback(self, msg):
        if self.et_counter % self.et_object.sample_rate == 0 and self.FaMSeC_state:
            si, self.predx, self.predy = msg
            if si >= 0 and self.wp_thread is not None:
                self.model_quality_assessment = si
                label, color = assessment_to_label(si)
                self.et_display.setText(label)
                self.et_display.setStyleSheet("background-color: {}".format(color))

                # Save off the machine self-confidence assessments
                self.model_quality_over_time.append(self.model_quality_assessment)
                self.outcome_assessment_over_time.append(self.outcome_assessment)
                self.time_over_time.append(abs(time.time() - self.start_time))
                self.pose_over_time.append([p for p in self.state.position])
                self.orientation_over_time.append([p for p in self.state.orientation])

                if self.ET_GOA_state and si < self.delta:
                    print("TRIGGERING ET-GOA")
                    self.stop_robot_callback()
                    # TODO correct the waypoints here
                    self.run_assessment_callback()

        self.et_counter += 1

    def outcome_assessment_callback(self):
        label = "N/A"
        self.goa_display.setText(label)
        self.goa_display.setStyleSheet("background-color: {}".format("light gray"))
        self.et_display.setText(label)
        self.et_display.setStyleSheet("background-color: {}".format("light gray"))

    def solver_quality_callback(self):
        if self.FaMSeC_state:
            label, color = assessment_to_label(1)
            self.sq_display.setText("Not implemented")
            self.sq_display.setStyleSheet("background-color: {}".format("blue"))

    def experience_callback(self):
        if self.FaMSeC_state:
            label, color = assessment_to_label(0.5)
            self.exp_display.setText("Not implemented")
            self.exp_display.setStyleSheet("background-color: {}".format("blue"))


try:
    app = QApplication(sys.argv)
    MainWindow = myMainWindow()
    MainWindow.show()
    app.exec_()
except Exception as e:
    print(e)
    traceback.print_exc()
