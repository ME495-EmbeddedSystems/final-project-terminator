#!/usr/bin/env python

import unittest
import rospy
from std_msgs.msg import String
from time import sleep
import rostest
import math
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import cv2
import actionlib
import tf
import numpy as np
from random import uniform
from darknet_ros_msgs.msg import BoundingBoxes
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, Point

from std_msgs.msg import String




class TalkerTestCase(unittest.TestCase):

    talker_ok = False

    def callback(self, data):
        self.talker_ok = True

    def test_if_publishes(self):

        rospy.init_node('myTestingNode')

        rospy.Subscriber('/custom_calibration', CameraInfo, self.callback)

        counter = 0

        # while rospy.is_shutdown():
        while (counter < 5):
    
            print("entered")
            counter = counter + 1
            sleep(1)


        self.assertTrue(self.talker_ok)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('terminator', 'talker_Test_Case', TalkerTestCase )
	

