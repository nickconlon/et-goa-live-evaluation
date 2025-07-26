import zmq

NOBLOCK = zmq.NOBLOCK


class ZmqPublisher:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://{}:{}".format(self.ip, self.port))

    def publish(self, data):
        self.socket.send_string(str(data))


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
