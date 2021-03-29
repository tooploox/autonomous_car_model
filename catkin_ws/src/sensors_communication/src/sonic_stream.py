#!/usr/bin/env python

import json

import rospy
from std_msgs.msg import String

from sonic_sensor import HCSR04


if __name__ == "__main__":
    rospy.init_node('sonic_data_streamer')
    pub = rospy.Publisher('/sonic_data', String, queue_size=10)

    rate = rospy.Rate(20)

    sensors = rospy.get_param('/sonic_sensors')
    initialized_sensors = {}
    for sensor, params in sensors.items():
        initialized_sensors[sensor] = HCSR04(**params)

    while not rospy.is_shutdown():
        readings = {}
        for sensor_name, initialized_sensor in initialized_sensors.items():
            readings[sensor_name] = initialized_sensor.get_measurement()

        msg = String()
        msg.data = json.dumps(readings)
        pub.publish(msg)

        rate.sleep()
