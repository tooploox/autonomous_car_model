#!/usr/bin/env python

import json

import rospy
from std_msgs.msg import String

from imu import mpu6050

if __name__ == "__main__":
    rospy.init_node('acc_gyro_streamer')
    pub = rospy.Publisher('/acc_gyro', String, queue_size=10)

    rate = rospy.Rate(20)

    # /This assumes that it's the first I2C device
    mpu = mpu6050(0x68)

    while not rospy.is_shutdown():
        readings = mpu.get_all_data()

        msg = String()
        msg.data = json.dumps(readings)
        pub.publish(msg)

        rate.sleep()
