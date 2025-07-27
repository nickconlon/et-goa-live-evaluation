import json
import time
import traceback
from typing import Callable

import comms
import rospy
from geometry_msgs.msg import Twist  # Twist messages
from navigation import WaypointFollower
from numpy.typing import NDArray
from PyQt5 import QtCore
from world_model import State


class WaypointThread(QtCore.QThread):
    mq_emit = QtCore.pyqtSignal(object)
    task_time_emit = QtCore.pyqtSignal(int)
    done_emit = QtCore.pyqtSignal()
    waypoint_emit = QtCore.pyqtSignal(int)

    waypoints = []
    should_run = True
    should_drive = True
    world_model = None

    def run(self):
        t1 = time.time()
        gtw = WaypointFollower(self.world_model)
        print("starting waypoint thread")
        _xs = self.waypoints[:, 0]
        _ys = self.waypoints[:, 1]
        waypoint_counter = 0
        for _x, _y in zip(_xs, _ys):
            if not self.should_run:
                break
            print("going to wp {}, {}".format(_x, _y))
            gtw.dist_err = 1.0
            gtw.x_dest = _x
            gtw.y_dest = _y
            # Go to waypoint
            while gtw.dist_err > 0.1 and self.should_run:
                gtw.do_et = True
                self.mq_emit.emit([gtw.MQ, gtw.predx, gtw.predy])
                if gtw.pose_cmd_vel is not None and self.should_drive:
                    vel = Twist()
                    vel.linear.x = gtw.pose_cmd_vel
                    vel.angular.z = gtw.rot_cmd_vel
                    gtw.pub_vel.publish(vel)
                gtw.sleep_rate.sleep()
            gtw.do_et = False
            if gtw.dist_err <= 0.1:
                self.waypoint_emit.emit(waypoint_counter)
                waypoint_counter += 1
        gtw.grab_odom.unregister()
        gtw.grab_odom = None
        t2 = time.time()
        rospy.sleep(0.25)
        self.task_time_emit.emit(int(abs(t2 - t1)))
        print("task time", abs(t2 - t1))
        self.done_emit.emit()
        print("exiting waypoint thread")


class ControlThread(QtCore.QThread):
    sub = comms.ZmqSubscriber("localhost", "5558")

    should_run = True

    def run(self) -> None:
        print("starting control thread")
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
            except Exception:
                traceback.print_exc()
        # TODO close the socket stuff
        print("exiting control thread")


class StateUpdateThread(QtCore.QThread):
    sub = comms.ZmqSubscriber("localhost", "5558")

    state_emit = QtCore.pyqtSignal(object)
    mq_emit = QtCore.pyqtSignal(object)

    should_run = True

    def run(self) -> None:
        print("starting state thread")
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
            except Exception:
                traceback.print_exc()
        print("exiting state thread")


class WorldModelRolloutThread(QtCore.QThread):
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        state: State,
        path: NDArray,
        wm_rollout_func: Callable[[State, NDArray[float]], None],
    ) -> None:
        super(WorldModelRolloutThread, self).__init__()
        self.known_obstacles = None
        self.path = path
        self.state = state
        self.wm_rollout_func = wm_rollout_func

    def run(self) -> None:
        print("starting rollout thread")
        try:
            self.wm_rollout_func(self.state, self.path)
            self.finished.emit()
        except Exception:
            traceback.print_exc()
        print("exiting rollout thread")
