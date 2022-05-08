import sys
import zmq
import base64
import cv2
import numpy as np

class ZmqSubscriber:
    def __init__(self, ip, port, topic=""):
        self.ip = ip
        self.port = port
        self.topic = topic
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://{}:{}".format(self.ip, self.port))
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)

    def receive(self, flags=0):
        data = self.socket.recv_string(flags=flags)
        return data


if __name__ == "__main__":
    sub = ZmqSubscriber("localhost", "5557")

    string = sub.receive()
    jpg_original = base64.b64decode(string)
    jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
    img = cv2.imdecode(jpg_as_np, flags=1)
    cv2.imshow("image", img)
    cv2.waitKey(0)

    '''
    sub = ZmqSubscriber("localhost", "5556", topic="10001")
    zip_filter = sys.argv[1] if len(sys.argv) > 1 else "10001"
    # Process 5 updates
    total_temp = 0
    for update_nbr in range(5):
        string = sub.receive()
        zipcode, temperature, relhumidity = string.split()
        total_temp += int(temperature)

        print((f"Average temperature for zipcode "
               f"'{zipcode}' was {total_temp / (update_nbr + 1)} F"))
    '''