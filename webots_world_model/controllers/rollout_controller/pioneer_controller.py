import numpy as np


def turn_right():
    return np.array([1, -1, 1, -1]) * 0.5  # *1.5  # added multiplier


def turn_left():
    return np.array([-1, 1, -1, 1]) * 0.5  # *1.5  # added multiplier


def forward():
    return np.array([2, 2, 2, 2]) * 0.99  # *2  # added multiplier


def backward():
    return np.array([-2, -2, -2, -2])


def stop():
    return np.array([0, 0, 0, 0])
