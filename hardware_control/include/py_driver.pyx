#!python
#cython: language_level=3
from libcpp cimport bool

class DummyServoKit:
    class Servo:
        def __init__(self):
            self.angle = 0

    class ContinuousServo:
        def __init__(self):
            self.throttle = 0

    def __init__(self, *args, **kwargs):
        self.servo = [self.Servo()] * 16
        self.continuous_servo = [self.ContinuousServo()] * 16

try:
    from adafruit_servokit import ServoKit

except (NotImplementedError, ImportError):
    print("Using dummy version of servokit")
    ServoKit = DummyServoKit

class PyCarRobot:
    def __init__(self, motor_channel_ind=0, steer_channel_ind=1):
        self._kit = ServoKit(channels=16)
        self._motor_channel_ind = motor_channel_ind
        self._steer_channel_ind = steer_channel_ind


    def read_speed(self):
        "Reading data from encoder"
        pass

    def set_throttle(self, throttle):
        try:
            self._kit.continuous_servo[self._motor_channel_ind] = throttle
        except:
            return False

        return True

    def set_angle(self, angle):
        """
        Assumptions:
        0 degrees - wheels are straight
        +max degrees - wheels are fully turned right
        -max degrees - wheels are fully turned left
        """
        try:
            self._kit.servo[self._steer_channel_ind] = angle
        except:
            return False
        return True


cdef public object instantiatePyCarRobot():
    return PyCarRobot()

cdef public bool set_throttle(object p, float val):
    return p.set_throttle(val)

cdef public bool set_angle(object p, float val):
    return p.set_angle(val)