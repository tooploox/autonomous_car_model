from __future__ import print_function

import time

try:
    import Jetson.GPIO as GPIO
except ImportError:
    import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)


class HCSR04:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.setup_pins()

    def get_measurement(self):
        """
        Needs refactoring, ideally should support interrupts.
        """

        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, False)

        timeout_start = time.time()

        StartTime = time.time()
        StopTime = time.time()

        while GPIO.input(self.echo_pin) == 0:
            StartTime = time.time()
            if StartTime - timeout_start > 0.5:
                break

        while GPIO.input(self.echo_pin) == 1:
            StopTime = time.time()
            if StartTime - timeout_start > 0.5:
                break

        TimeElapsed = StopTime - StartTime
        return (TimeElapsed * 34300) / 2

    def setup_pins(self):
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
