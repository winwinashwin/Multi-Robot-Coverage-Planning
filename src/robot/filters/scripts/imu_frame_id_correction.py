#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu


class FrameIdCorrectionNode:
    def __init__(self, prefix):
        self._prefix = prefix
        self._sub = rospy.Subscriber("imu_in", Imu, self.callback)
        self._pub = rospy.Publisher("imu_out", Imu, queue_size=10)

    def callback(self, msg: Imu):
        msg.header.frame_id = f"{self._prefix}/{msg.header.frame_id}"
        self._pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("imu_frame_id_correction", anonymous=True)
    node = FrameIdCorrectionNode(rospy.get_param("tf_prefix"))
    rospy.spin()
