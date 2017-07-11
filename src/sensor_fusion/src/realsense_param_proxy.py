#!/usr/bin/env python2.7

# R2 Perception - Hanson Robotics Unified Perception System, v1.0
# by Desmond Germans

import os
import rospy
import time
import dynamic_reconfigure.client
from r2_perception.msg import RealSenseParam


class RealSenseParamProxy(object):


    def __init__(self):

        #self.dynparam = dynamic_reconfigure.client.Client("vision_pipeline",timeout=30,config_callback=self.HandleConfig)

        self.param_pub = rospy.Publisher("realsense_param",RealSenseParam,queue_size=5)


    def HandleConfig(self,data):

        msg = RealSenseParam()
        msg.ts = rospy.get_rostime()
        msg.fovy = data.fovy
        msg.aspect = data.aspect
        msg.camera_rate = data.camera_rate
        msg.face_detect_rate = data.face_detect_rate
        msg.hand_detect_rate = data.hand_detect_rate
        msg.saliency_detect_rate = data.saliency_detect_rate
        self.param_pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('realsense_param_proxy')
    node = RealSenseParamProxy()
    rospy.spin()
