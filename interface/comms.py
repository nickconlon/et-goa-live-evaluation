import zmq
import numpy as np
import yaml
from subprocess import Popen
import time

def find_nearest(array, value):
    idx = (np.abs(array-value)).argmin()
    return idx


def min_diff_pos_sorted(sorted_array, target):
    idx = np.searchsorted(sorted_array, target)
    idx1 = max(0, idx-1)
    return np.abs(np.array(sorted_array[idx1:idx+1])-target).argmin() + idx1


def settings_reader(settings_file):
    with open(settings_file, 'r') as file:
        settings = yaml.safe_load(file)
    return settings


class StateObject:
    def __init__(self):
        self.pos = []
        self.ori = []
        self.obs = []
        self.robot_time = 0
        self.wall_time = 0
        self.goal = []
        self.waypoint = 0

    def get_state_dict(self):
        obj = {
            'position': self.pos,
            'orientation': self.ori,
            'known_obs': self.obs,
            't_robot': self.robot_time,
            't_wall': self.wall_time,
            'goal': self.goal,
            'waypoint': self.waypoint
        }
        return obj

    def get_state_array(self):
        return [
            *self.pos,
            *self.ori,
            self.obs,
            self.robot_time,
            self.wall_time
        ]

    def set_state(self, position, orientation, sensed_obs, t_robot, t_wall, goal, waypoint):
        self.pos = [float(x) for x in position]
        self.ori = [float(x) for x in orientation]
        self.obs = sensed_obs
        self.robot_time = t_robot
        self.wall_time = t_wall
        self.goal = [float(x) for x in goal]
        self.waypoint = waypoint

    def set_state_from_object(self, obj):
        self.pos = obj['position']
        self.ori = obj['orientation']
        self.obs = obj['known_obs']
        self.robot_time = obj['t_robot']
        self.wall_time = obj['t_wall']
        self.goal = obj['goal']
        self.waypoint = obj['waypoint']


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


def do_rollout(position, orientation, goal, waypoint_counter, known_obstacles, waypoints=None):
    d = {
        'start_position': [float(x) for x in position],
        'start_orientation': [float(x) for x in orientation],
        'goal_position': [float(x) for x in goal],
        'waypoint_index': waypoint_counter,
        'num_iterations': 10,
        'publish': False,
        'prefix': 'rollout',
        'known_obs': known_obstacles,
    }
    if waypoints is not None:
        d['wp_x'] = [float(x) for x in waypoints[:, 0]]
        d['wp_y'] = [float(x) for x in waypoints[:, 1]]

    print(yaml.dump(d))
    fname = '/home/cohrint-skynet/Documents/et-goa/controllers/rollout_controller/'
    with open(fname + 'settings.yaml', 'w') as f:
        yaml.dump(d, f, default_flow_style=None, sort_keys=False)

    p = Popen('/home/cohrint-skynet/Documents/et-goa/controllers/do_rollouts.sh')
    stdout, stderr = p.communicate()


if __name__ == "__main__":
    sub = ZmqSubscriber('192.168.0.12', 5559)
    while True:
        try:
            data = sub.receive(flags=NOBLOCK)
            print('received: ', data)
        except Exception as e:
            pass
        time.sleep(0.1)
