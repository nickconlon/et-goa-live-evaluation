import zmq
import numpy as np
import time
import base64
import cv2


class ZmqPublisher:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://{}:{}".format(self.ip, self.port))

    def publish(self, data):
        self.socket.send_string(str(data))


if __name__ == "__main__":
    pub = ZmqPublisher("*", "5556")
    mappub = ZmqPublisher("*", "5557")
    img_idx = 0
    while True:
        topic = "sc"
        sc1 = np.random.rand()
        sc2 = np.random.rand()
        sc3 = np.random.rand()
        datastring = f"{topic} {sc1} {sc2} {sc3}"
        print("Publishing new self-confidence: ", datastring)
        pub.publish(datastring)

        img = cv2.imread('imgs/img{}.png'.format(img_idx))
        string = base64.b64encode(cv2.imencode('.png', img)[1]).decode()
        print("Publishing new map")
        mappub.publish(string)
        img_idx = (img_idx+1) % 10
        time.sleep(1)





