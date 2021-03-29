from __future__ import print_function
import time

try:
    import Jetson.GPIO as GPIO
except ImportError:
    import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)


class Motor(object):
    def __init__(self, pwm_pin, pwm_min, pwm_max, pwm_init_duty_cycle=0,
                 pwm_frequency=50, pwm_neutral=None):
        self.pwm_pin = pwm_pin
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.pwm_init_duty_cycle = pwm_init_duty_cycle
        self.pwm_frequency = pwm_frequency
        self.pwm = None

        if pwm_neutral is None:
            self.pwm_neutral = (pwm_min + pwm_max) / 2

        self.setup()

    def setup(self):
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_frequency)
        self.pwm.start(self._clip_value(self.pwm_init_duty_cycle))

    def _clip_value(self, n):
        return max(min(self.pwm_max, n), self.pwm_min)

    def _standardised_to_inner_range(self, value):
        """Converts values from -1 to 1 into inner
        PWM range.
        """
        standard_min = -1
        standard_max = 1

        inner = (value - standard_min)\
            * (self.pwm_max - self.pwm_min)\
            / (standard_max - standard_min)\
            + self.pwm_min

        return inner

    def set_pwm_duty_cycle(self, pwm_duty_cycle):
        pass


class SteeringMotor(Motor):

    def set_pwm_duty_cycle(self, pwm_duty_cycle):
        dc = self._standardised_to_inner_range(pwm_duty_cycle)
        self.pwm.ChangeDutyCycle(dc)
        time.sleep(0.1)
        self.pwm.ChangeDutyCycle(0)


class DriveMotor(Motor):

    def set_pwm_duty_cycle(self, pwm_duty_cycle):
        pwm_duty_cycle *= -1
        dc = self._standardised_to_inner_range(pwm_duty_cycle)
        self.pwm.ChangeDutyCycle(dc)

    def break_(self):
        self.pwm.ChangeDutyCycle(self.pwm_neutral)
        self.pwm.ChangeDutyCycle(0)


class TamiyaVehicle:
    def __init__(self, steering_motor, drive_motor):
        self.steering_motor = steering_motor
        self.drive_motor = drive_motor

    def move(self, steering_val, speed_val):
        self.steering_motor.set_pwm_duty_cycle(steering_val)
        self.drive_motor.set_pwm_duty_cycle(speed_val)

    def stop(self):
        self.drive_motor.break_()
