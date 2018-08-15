#!/usr/bin/env python

"""
Gaussian Mixture Model for human interest prediction
@Vivek Chauhan (viveksdchauhan@icloud.com)
"""

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker, MarkerArray
import math, time, matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import numpy as np, seaborn as sns
import rospy, csv
from cv_bridge import CvBridge, CvBridgeError
import cv2
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from std_msgs.msg import Header, ColorRGBA, Int8
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PointStamped, Quaternion, Pose, Point, Vector3, TransformStamped

class PredictionEstimator(object):

    def __init__(self):
        self.tf_listener_msg = rospy.Subscriber("/tf", TFMessage, callback_tf_msg, queue_size=1)
    	self.lefteye_position = rospy.Subscriber("/new/subject/lefteye/position", PointStamped, callback_left_msg, queue_size=1)
    	self.righteye_position = rospy.Subscriber("/new/subject/righteye/position", PointStamped, callback_right_msg, queue_size=1)
    	self.nose_position = rospy.Subscriber("/new/subject/nose/position", PointStamped, callback_nose_msg, queue_size=1)
    	self.darknet_msg = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_yolo_msg, queue_size=1)

    def callback_tf_msg(self, data):
        pass

    def callback_left_msg(self, data):
        pass

    def callback_right_msg(self, data):
        pass

    def callback_nose_msg(self, data):
        pass

    def callback_yolo_msg(self, data):
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('predict probabilty')
        predict_estimator = PredictionEstimator()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("See ya")
    except rospy.ROSException as e:
        if str(e) == "publish() to a closed topic":
            print("See ya")
        else:
            raise e
    except KeyboardInterrupt:
        print("Shutting down")
