#!/usr/bin/env python

from __future__ import print_function

import rospy
from std_msgs.msg import String

import base64
import cv2


def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=29,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


if __name__ == "__main__":

    vc = cv2.VideoCapture(gstreamer_pipeline(flip_method=2),
                          cv2.CAP_GSTREAMER)

    rospy.init_node('camera_streamer')
    pub = rospy.Publisher('/camera_stream', String, queue_size=2)

    rate = rospy.Rate(29)

    decoded = ""
    while not rospy.is_shutdown():
        try:
            ret, frame = vc.read()
            _, buffer = cv2.imencode('.jpg', frame)
            decoded = base64.b64encode(buffer).decode('utf-8')
        except KeyboardInterrupt:
            break
        except cv2.error as e:
            rospy.loginfo(e)

        msg = String()
        msg.data = decoded
        pub.publish(msg)

        rate.sleep()

    vc.release()
    rospy.loginfo("stopped")

