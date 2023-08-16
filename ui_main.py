import base64
import numpy as np
import traceback
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtGui import QPixmap, QImage
import PyQt5.QtCore as QtCore
import goa_ui
import sys
import cv2

import zmq_subscriber as subscriber
import zmq_publisher as publisher


class Generic_ZeroMQ_Listener(QtCore.QObject):
    message = QtCore.pyqtSignal(str)

    def __init__(self, ip, port, topic):
        QtCore.QObject.__init__(self)
        self.sub = subscriber.ZmqSubscriber(ip, port, topic)
        self.running = True

    def loop(self):
        while self.running:
            string = self.sub.receive()
            self.message.emit(string)


class myMainWindow(QMainWindow, goa_ui.Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)

        """
        Setup the area buttons
        """
        self.area1_button.clicked.connect(self.button1_click)
        self.area2_button.clicked.connect(self.button2_click)
        self.area3_button.clicked.connect(self.button3_click)
        self.go_home_button.clicked.connect(self.go_home_click)

        """
        Setup the confidence listener thread
        """
        self.thread = QtCore.QThread()
        self.zeromq_self_confidence_listener = Generic_ZeroMQ_Listener("localhost", "5556", "sc")
        self.zeromq_self_confidence_listener.moveToThread(self.thread)
        self.thread.started.connect(self.zeromq_self_confidence_listener.loop)
        self.zeromq_self_confidence_listener.message.connect(self.update_self_confidence)
        QtCore.QTimer.singleShot(0, self.thread.start)

        """
        Setup the map listener thread
        """
        self.mapThread = QtCore.QThread()
        self.zeromq_self_map_listener = Generic_ZeroMQ_Listener("localhost", "5557", "")
        self.zeromq_self_map_listener.moveToThread(self.mapThread)
        self.mapThread.started.connect(self.zeromq_self_map_listener.loop)
        self.zeromq_self_map_listener.message.connect(self.map_update)
        QtCore.QTimer.singleShot(0, self.mapThread.start)

        """
        Setup the goal publisher
        """
        self.goal_publisher = publisher.ZmqPublisher("*", "5558")

        self.stop_robot_button.clicked.connect(self.stop_robot_click)

    def button1_click(self):
        try:
            self.area1_data_label.setStyleSheet("background-color: green")
            self.area2_data_label.setStyleSheet("background-color: none")
            self.area3_data_label.setStyleSheet("background-color: none")
            self.send_new_goal("1")
        except Exception as e:
            print(e)
            traceback.print_exc()

    def button2_click(self):
        try:
            self.area1_data_label.setStyleSheet("background-color: none")
            self.area2_data_label.setStyleSheet("background-color: green")
            self.area3_data_label.setStyleSheet("background-color: none")
            self.send_new_goal("2")
        except Exception as e:
            print(e)
            traceback.print_exc()

    def button3_click(self):
        try:
            self.area1_data_label.setStyleSheet("background-color: none")
            self.area2_data_label.setStyleSheet("background-color: none")
            self.area3_data_label.setStyleSheet("background-color: green")
            self.send_new_goal("3")
        except Exception as e:
            print(e)
            traceback.print_exc()

    def go_home_click(self):
        try:
            self.area1_data_label.setStyleSheet("background-color: none")
            self.area2_data_label.setStyleSheet("background-color: none")
            self.area3_data_label.setStyleSheet("background-color: none")
            self.send_new_goal("0")
        except Exception as e:
            print(e)
            traceback.print_exc()

    def stop_robot_click(self):
        try:
            self.send_new_goal("-1")
            print("Stopping Robot")
        except Exception as e:
            print(e)
            traceback.print_exc()

    def send_new_goal(self, goal_index):
        try:
            self.goal_publisher.publish("goal {}".format(goal_index))
            print("Published goal ", goal_index)
        except Exception as e:
            print(e)

    def map_update(self, msg):
        try:
            jpg_original = base64.b64decode(msg)
            jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
            img = cv2.imdecode(jpg_as_np, flags=1)
            image = QImage(img, img.shape[1], img.shape[0], img.shape[1] * 3, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(image)
            self.map_area.setPixmap(pixmap)
            self.map_area.setScaledContents(True)
        except Exception as e:
            print(e)

    def update_self_confidence(self, msg):
        try:
            topic, sc1, sc2, sc3 = msg.split()
            confidence_array = [sc1, sc2, sc3]
            confidence_array = [float(x) for x in confidence_array]
            self.area1_self_confidence_value.setText("{:.2f}".format(confidence_array[0]))
            self.area2_self_confidence_value.setText("{:.2f}".format(confidence_array[1]))
            self.area3_self_confidence_value.setText("{:.2f}".format(confidence_array[2]))

            value_array = [1,2,3]
            self.area1_value.setText("{:.2f}".format(value_array[0]))
            self.area2_value.setText("{:.2f}".format(value_array[1]))
            self.area3_value.setText("{:.2f}".format(value_array[2]))
        except Exception as e:
            print(e)


app = QApplication(sys.argv)
MainWindow = myMainWindow()
MainWindow.show()
app.exec_()
