'''
import cv2 as cv
import traceback
import numpy as np
import os
import time


def make_markers():
    # Save location
    dir_mark = r'C:\DATA\aruco'

    # Parameter
    num_mark = 20 #Number of markers
    size_mark = 500 #Size of markers

    ### --- marker images are generated and saved --- ###
    # Call marker type
    dict_aruco = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)

    for count in range(num_mark) :

        id_mark = count
        img_mark = cv.aruco.drawMarker(dict_aruco, id_mark, size_mark)

        if count < 10 :
            img_name_mark = 'mark_id_0' + str(count) + '.jpg'
        else :
            img_name_mark = 'mark_id_' + str(count) + '.jpg'
        path_mark = os.path.join(dir_mark, img_name_mark)

        cv.imwrite(path_mark, img_mark)


class MarkSearch:
    dict_aruco = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters_create()

    def __init__(self, cameraID):
        self.cap = cv.VideoCapture(cameraID)

    def get_markID(self):
        """
        Obtain marker id list from still image
        """
        ret, frame = self.cap.read()
        gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)

        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, self.dict_aruco, parameters=self.parameters)

        list_ids = np.ravel(ids)

        return list_ids


def read_aruco():
    dict_aruco = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters_create()

    ### --- parameter --- ###
    cameraID = 0
    cam0_mark_search = MarkSearch(cameraID)

    try:
        while True:
            print(' ----- get_markID ----- ')
            print(cam0_mark_search.get_markID())
            time.sleep(0.5)
    except KeyboardInterrupt:
        cam0_mark_search.cap.release()


class ArUco:
    dict_aruco = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters_create()

    def __init__(self, device=0):
        self.cap = cv.VideoCapture(device)

    def detect(self):
        ret, frame = self.cap.read()
        if ret:
            gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, self.dict_aruco, parameters=self.parameters)
            frame_markers = cv.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            return frame_markers, ids
        return None, None

    def close(self):
        self.cap.release()


def show_aruco():
    dict_aruco = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters_create()

    cap = cv.VideoCapture(0)

    try:
        while True:
            ret, frame = cap.read()
            gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)

            corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, dict_aruco, parameters=parameters)

            frame_markers = cv.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            cv.imshow('frame', frame_markers)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        cv.destroyWindow('frame')
        cap.release()
    except KeyboardInterrupt:
        cv.destroyWindow('frame')
        cap.release()


def read_from_camera():
    print('starting')
    try:
        cam = cv.VideoCapture(0)
        while True:
            result, image = cam.read()
            print(result)
            cv.imshow('nothing', image)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        cv.destroyWindow('nothing')
    except Exception as e:
        traceback.print_exc()
    print('ending')


if __name__ == '__main__':
    ar = ArUco(0)
    while True:
        frame_markers, id = ar.detect()
        cv.imshow('frame', frame_markers)
        print(id)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    cv.destroyWindow('frame')
    ar.close()
'''