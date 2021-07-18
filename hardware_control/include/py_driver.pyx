#!python
#cython: language_level=3

class PyCarRobot:
    def __init__(self):
        self._motor_speed = None
        self._steering_angle = None

    def read_speed(self):
        "Reading data from encoder"

    def set_speed(self, speed):
        return f"This is new speed {speed}"

    def set_angle(self, angle):
        return f"This is new speed {angle}"

    def get_speed(self):
        self._motor_speed


cdef public object instantiatePyCarRobot():
    return PyCarRobot()

cdef public char* set_speed(object p, float val):
    return bytes(p.set_speed(val), encoding = 'utf-8')

cdef public char* set_angle(object p, float val):
    return bytes(p.set_angle(val), encoding = 'utf-8')