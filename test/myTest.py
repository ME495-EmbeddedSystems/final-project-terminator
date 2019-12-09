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

    # Booleans that record if we have any messages published
    customCalibration = False
    imageRaw = False
    yolo_detection = False

    def call_custom_calibration(self, data):
        self.customCalibration = True
    
    def call_image_raw(self, data):
        self.imageRaw = True
    
    def call_yolo_detection(self, data):
        self.yolo_detection = True

    def test_if_publishes(self):

        rospy.init_node('myTestingNode')
 
        rospy.Subscriber('/custom_calibration', CameraInfo, self.call_custom_calibration )
      
        rospy.Subscriber('/image_raw', Image, self.call_image_raw  )
        
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.call_yolo_detection)

        # Let this node spin for a few seconds to allow 
        # the messages some time to be published
        counter = 0
        while (counter < 5):
    
            counter = counter + 1
            sleep(1)
            

        self.assertTrue(self.customCalibration)
        self.assertTrue(self.imageRaw)
        self.assertTrue(self.yolo_detection)



if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('terminator', 'talker_Test_Case', TalkerTestCase )
	

