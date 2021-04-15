#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

from motors_driver import SteeringMotor, DriveMotor, TamiyaVehicle


steering = SteeringMotor(pwm_pin=33, pwm_min=6.5,
                         pwm_max=10.5,
                         pwm_init_duty_cycle=8.45,
                         pwm_neutral=8.45)
drive = DriveMotor(pwm_pin=32, pwm_min=5.5, pwm_max=8.5,
                   pwm_init_duty_cycle=7,
                   pwm_neutral=7)

robot = TamiyaVehicle(steering_motor=steering, drive_motor=drive)


def callback_receive(msg):
    global robot
    robot.move(msg.angular.z, msg.linear.x)


if __name__ == "__main__":
    rospy.init_node('cmd_listener')

    sub = rospy.Subscriber('/cmd_vel', Twist, callback_receive, queue_size=1)
    rospy.spin()
