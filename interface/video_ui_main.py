import json
import time
import traceback
import sys

import cv2
import cv2 as cv
from cv2 import aruco
import numpy as np
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5 import QtCore, QtGui
import video_ui

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class VideoThread(QtCore.QThread):
    image_emit = QtCore.pyqtSignal(object)
    bridge = CvBridge()
    should_run = True
    pub = rospy.Publisher('/image_topic', Image, queue_size=10)

    def run(self):
        cap = cv.VideoCapture(0)
        while self.should_run:
            ret, frame = cap.read()
            if not ret:
                continue
            else:
                frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='passthrough')
                self.pub.publish(img_msg)
            time.sleep(0.01)
        cap.release()

class myMainWindow(QMainWindow, video_ui.Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        rospy.init_node('video_interface', anonymous=True)
        self.setupUi(self)
        self.video_label.setPixmap(QtGui.QPixmap("./nature.jpg"))

        self.grab_image = rospy.Subscriber('/goa/image_topic', Image, self.receive_image_callback)
        self.signal_pub = rospy.Publisher('/goa/image_signal', String, queue_size=10)

        self.bridge = CvBridge()
        self.dict_aruco = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
        self.parameters = cv.aruco.DetectorParameters_create()
        self.id_for_signal = 1

    def receive_image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.update_image_callback(cv_img)


    def update_image_callback(self, img):
        img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dict_aruco, parameters=self.parameters)
        img_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)

        img = img_markers.astype(np.uint8)
        height, width, channel = img.shape
        bytesPerLine = 3 * width
        qImg = QtGui.QImage(img.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
        QtGui.QPixmap.fromImage(qImg)
        self.video_label.setPixmap(QtGui.QPixmap.fromImage(qImg))
        if ids is not None:
            self.signal_pub.publish('ID_SIGNAL')


try:
    app = QApplication(sys.argv)
    MainWindow = myMainWindow()
    MainWindow.show()
    app.exec_()
except Exception as e:
    print(e)
    traceback.print_exc()
