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

    def callback_tf_msg(self., data):
        if self.data.transforms[0].child_frame_id == "world_gazetwoeyes":
            self.rotation_x = self.data.transforms[0].transform.rotation.x
    		self.rotation_y = self.data.transforms[0].transform.rotation.y
    		self.rotation_z = self.data.transforms[0].transform.rotation.z
    		self.rotation_w = self.data.transforms[0].transform.rotation.w

    	if self.data.transforms[0].child_frame_id == "/head_pose_estimated_new":
            self.h_rotation_x = self.data.transforms[0].transform.rotation.x
    		self.h_rotation_y = self.data.transforms[0].transform.rotation.y
    		self.h_rotation_z = self.data.transforms[0].transform.rotation.z
    		self.h_rotation_w = self.data.transforms[0].transform.rotation.w

    def callback_left_msg(self, data):
        self.x_pos = (self.data.point.x ) / (180 * np.pi)
    	self.y_pos = (self.data.point.y ) / (180 * np.pi)
    	self.z_pos = (self.data.point.z ) / (180 * np.pi)
    	# print("left eye position")
    	# print x_pos, y_pos, z_pos
    	self.left_eye_marker_publisher = rospy.Publisher('left_eye', Marker, queue_size=5)
    	self.left_eye_marker = Marker(type=Marker.SPHERE,id=0,lifetime=rospy.Duration(1.5),pose=Pose(Point(x_pos, y_pos, z_pos), Quaternion(w=1, x=0, y=0, z=0)),scale=Vector3(0.2, 0.2, 0.2),header=Header(frame_id='kinect2_rgb_optical_frame'),color=ColorRGBA(0.0, 1.0, 0.0, 0.8),text="whatsup")
    	self.left_eye_line = rospy.Publisher('left_eye_line', Marker, queue_size=5)
    	self.left_eye_line_marker = Marker()
    	self.left_eye_line_marker.header.frame_id = "kinect2_link"
    	self.left_eye_line_marker.type = self.left_eye_line_marker.LINE_STRIP
    	self.left_eye_line_marker.action = self.left_eye_line_marker.ADD

    	# marker scale
    	self.left_eye_line_marker.scale.x = 0.03
    	self.left_eye_line_marker.scale.y = 0.03
    	self.left_eye_line_marker.scale.z = 0.03

    	# marker color
    	self.left_eye_line_marker.color.a = 1.0
    	self.left_eye_line_marker.color.r = 0.0
    	self.left_eye_line_marker.color.g = 1.0
    	self.left_eye_line_marker.color.b = 1.0

    	# marker orientaiton
    	self.left_eye_line_marker.pose.orientation.x = self.rotation_x
    	self.left_eye_line_marker.pose.orientation.y = self.rotation_y
    	self.left_eye_line_marker.pose.orientation.z = self.rotation_z
    	self.left_eye_line_marker.pose.orientation.w = self.rotation_w

    	# marker positionrotation_x
    	# left_eye_line_marker.pose.position.x = x_pos
    	# left_eye_line_marker.pose.position.y = y_pos
    	# left_eye_line_marker.pose.position.z = z_pos

    	# marker line pointsn the scripts direc
    	self.left_eye_line_marker.points = []
    	# first point
    	self.first_line_point = Point()
    	self.first_line_point.x = self.x_pos
    	self.first_line_point.y = self.y_pos - 0.4
    	self.first_line_point.z = self.z_pos
    	# second point

    	# first_line_point.x = x_pos
    	# first_line_point.y = y_pos - 0.8
    	# first_line_point.z = z_pos
    	self.left_eye_line_marker.points.append(self.first_line_point)
    	# second point
    	self.second_line_point = Point()
    	self.second_line_point.x = self.x_pos - 1.1
    	self.second_line_point.y = self.y_pos - 0.5
    	self.second_line_point.z = self.z_pos
    	# second_line_point.x = x_pos - 1
    	# second_line_point.y = y_pos - 1.1
    	# second_line_point.z = z_pos
    	self.left_eye_line_marker.points.append(self.second_line_point)

    	self.left_eye_line.publish(self.left_eye_line_marker)

    def callback_right_msg(self, data):
        self.x_pos = (self.data.point.x ) / (180 * np.pi)
    	self.y_pos = (self.data.point.y ) / (180 * np.pi)
    	self.z_pos = (self.data.point.z ) / (180 * np.pi)

    	self.right_eye_marker_publisher = rospy.Publisher('right_eye', Marker, queue_size=5)
    	self.right_eye_marker = Marker(type=Marker.SPHERE,id=0,lifetime=rospy.Duration(1.5),pose=Pose(Point(self.x_pos, self.y_pos, self.z_pos), Quaternion(w=1, x=0, y=0, z=0)),scale=Vector3(0.2, 0.2, 0.2),header=Header(frame_id='kinect2_rgb_optical_frame'),color=ColorRGBA(0.0, 1.0, 0.0, 0.8),text="whatsup")
    	self.right_eye_line = rospy.Publisher('right_eye_line', Marker, queue_size=5)
    	self.right_eye_line_marker = Marker()
    	self.right_eye_line_marker.header.frame_id = "kinect2_link"
    	self.right_eye_line_marker.type = self.right_eye_line_marker.LINE_STRIP
    	self.right_eye_line_marker.action = self.right_eye_line_marker.ADD

    	# marker scale
    	self.right_eye_line_marker.scale.x = 0.03
    	self.right_eye_line_marker.scale.y = 0.03
    	self.right_eye_line_marker.scale.z = 0.03
    	# marker color
    	self.right_eye_line_marker.color.a = 1.0
    	self.right_eye_line_marker.color.r = 0.0
    	self.right_eye_line_marker.color.g = 1.0
    	self.right_eye_line_marker.color.b = 0.0

    	# marker orientaitonworld
    	self.right_eye_line_marker.pose.orientation.x = self.rotation_x
    	self.right_eye_line_marker.pose.orientation.y = self.rotation_y
    	self.right_eye_line_marker.pose.orientation.z = self.rotation_z
    	self.right_eye_line_marker.pose.orientation.w = self.rotation_w

    	# marker position
    	# right_eye_line_marker.pose.position.x = x_pos
    	# right_eye_line_marker.pose.position.y = y_pos
    	# right_eye_line_marker.pose.position.z = z_pos
    	# marker line points
    	self.right_eye_line_marker.points = []
    	# first point
    	self.first_line_point = Point()
    	self.first_line_point.x = self.x_pos
    	self.first_line_point.y = self.y_pos - 0.7
    	self.first_line_point.z = self.z_pos
    	self.right_eye_line_marker.points.append(self.first_line_point)
    	# second point
    	self.second_line_point = Point()
    	self.second_line_point.x = self.x_pos - 1.1
    	self.second_line_point.y = self.y_pos - 0.6
    	self.second_line_point.z = self.z_pos
    	self.right_eye_line_marker.points.append(self.second_line_point)

    	self.right_eye_line.publish(self.right_eye_line_marker)


    def callback_nose_msg(self, data):
        self.x_pos = self.data.point.x / (180 * np.pi)
    	self.y_pos = self.data.point.y / (180 * np.pi)
    	self.z_pos = self.data.point.z / (180 * np.pi)

    def callback_yolo_msg(self, data):
        self.bound = rospy.Publisher('bound', MarkerArray, queue_size=1)
    	# bound_array.header.frame_id = "camera_link"
    	# bound_array.type = bound_array.LINE_STRIP
    	# bound_array.action = bound_array.ADD
    	self.bound_array = MarkerArray()

    #CLEAR stupid markers
    	self.stupid_mark_array = MarkerArray()
    	self.stupid_marker = Marker()
    	self.stupid_marker.action = stupid_marker.DELETEALL
    	self.stupid_mark_array.markers.append(self.stupid_marker)
    	# bounding_line = rospy.Publisher('bound', Marker, queue_size=5)
    	self.bound.publish(self.stupid_mark_array)
    #CLEAR stupid markers

    	for x in range(len(self.data.bounding_boxes)):
    		# print data.bounding_boxes[x]
    		self.data_bounding = self.data.bounding_boxes[x].Class
    		# print data_bounding
    		self.xmin = self.data.bounding_boxes[x].xmin / (180 * np.pi)
    		self.xmax = self.data.bounding_boxes[x].xmax / (180 * np.pi)
    		self.ymin = self.data.bounding_boxes[x].ymin / (180 * np.pi)
    		self.ymax = self.data.bounding_boxes[x].ymax / (180 * np.pi)
    		# print data.bounding_boxes
    		# tvmonitor_boundingbox = []
    		# print xmin
    		# poinssss = np.append(poinssss, [xmin, xmax, ymin, ymax])
    		# print np.linspace(-6, 8, 200)
    		# poi = pointofinter(xmin, xmax, ymin, ymax)
    		# print -poi[1], poi[0]
    		# print ymin, poi[1], ymax
    		# if (xmin) < poi[0] < (xmax) and (ymin) < -poi[1] < (ymax):
    		# 	# print data.bounding_boxes[x].Class
    		# 	pass
    		# else:
    		# 	pass
    		if count == 0:
    			if time.time() < timeout:
    				if self.data.bounding_boxes[x].Class == "tvmonitor":
    					self.tvmonitor_datapoints = np.vstack((self.tvmonitor_datapoints, np.array([self.xmin, self.xmax, self.ymin, self.ymax])))
    					print np.array([self.xmin, self.xmax, self.ymin, self.ymax])
    					# print tvmonitor_datapoints
    					# print tvmonitor_datapoints.size
    					# tv_monitor_variance = np.std(tvmonitor_datapoints)
    					# tvmonitor_mean = np.mean(tvmonitor_datapoints)
    					# probabitydistri(tv_monitor_variance, tvmonitor_mean, poi[0])
    				elif self.data.bounding_boxes[x].Class == "laptop":
    					self.laptop_datapoints = np.vstack((self.laptop_datapoints, np.array([self.xmin, self.xmax, self.ymin, self.ymax])))
    					# print xmin, xmax, ymin ,ymax
    					# pointofinter(laptop_datapoints)
    					# print "laptop"
    					# laptop_variance = np.std(laptop_datapoints)
    				elif self.data.bounding_boxes[x].Class == "cell phone":
    					self.cellphone_datapoints = np.vstack((self.cellphone_datapoints, np.array([self.xmin, self.xmax, self.ymin, self.ymax])))
    					# pointofinter(cellphone_datapoints)
    					# print "cell phone"
    					# cellphone_variance = np.std(cellphone_datapoints)
    				elif self.data.bounding_boxes[x].Class == "cup":
    					self.cup_datapoints = np.vstack((self.cup_datapoints, np.array([self.xmin, self.xmax, self.ymin, self.ymax])))
    					# cup_variance = np.std(cup_datapoints)
    				else:
    					pass
    			else:
    				count +=1
    				self.my_data = np.concatenate((self.tvmonitor_datapoints, self.laptop_datapoints))
    				self.my_data1 = np.concatenate((self.cup_datapoints, self.cellphone_datapoints))
    				self.my_data = np.concatenate((self.my_data, self.my_data1))
    				print self.my_data
    				np.savetxt("foo.csv", self.my_data, delimiter=",")
    				with open("output.csv", "w") as f:
    					writer = csv.writer(f)
    					writer.writerow(self.my_data)
    					# writer.writerow(',')
    				print "DONE"
    				plt.hist(self.my_data, 80, normed=True)
    				plt.xlim(-10,20)
    		else:
    			pass
    		# 	tv_monitor_variance = np.std(tvmonitor_datapoints)
    		# 	tvmonitor_mean = np.mean(tvmonitor_datapoints)
    		# 	probabitydistri(tvmonitor_mean, tv_monitor_variance, rotation_y)


    				# print tv_monitor_variance

    		self.bounding_line_marker = Marker()
    		self.bounding_line_marker.header.frame_id = "kinect2_link"
    		self.bounding_line_marker.id = x
    		self.bounding_line_marker.type = self.bounding_line_marker.LINE_STRIP
    		self.bounding_line_marker.action = self.bounding_line_marker.ADD
    		# my_data.append([xmin, xmax, ymin, ymax])
    		# print ymin
    		# marker scaleworld
    		self.bounding_line_marker.scale.x = 0.03
    		self.bounding_line_marker.scale.y = 0.03
    		self.bounding_line_marker.scale.z = 0.03

    		# marker color
    		self.bounding_line_marker.color.a = 1.0
    		self.bounding_line_marker.color.r = 1.0
    		self.bounding_line_marker.color.g = 1.0
    		self.bounding_line_marker.color.b = 0.0

    		# marker orientaiton
    		self.bounding_line_marker.pose.orientation.x = 0.0
    		self.bounding_line_marker.pose.orientation.y = 0.0
    		self.bounding_line_marker.pose.orientation.z = 0.0
    		self.bounding_line_marker.pose.orientation.w = 1.0

    		self.bounding_line_marker.points = []


    		# first point
    		self.first_line_point = Point()
    		self.first_line_point.x = 1.0
    		self.first_line_point.y = -self.xmin
    		self.first_line_point.z = self.ymin
    		self.bounding_line_marker.points.append(self.first_line_point)
    		# second point
    		self.second_line_point = Point()
    		self.second_line_point.x = 1.0
    		self.second_line_point.y = -self.xmax
    		self.second_line_point.z = self.ymin
    		self.bounding_line_marker.points.append(self.second_line_point)

    		self.third_line_point = Point()
    		self.third_line_point.x = 1.0
    		self.third_line_point.y = -self.xmax
    		self.third_line_point.z = self.ymax
    		self.bounding_line_marker.points.append(self.third_line_point)

    		self.forth_line_point = Point()
    		self.forth_line_point.x = 1.0 #1.0
    		self.forth_line_point.y = -self.xmin
    		self.forth_line_point.z = self.ymax
    		self.bounding_line_marker.points.append(self.forth_line_point)

    		self.fifth_line_point = Point()
    		self.fifth_line_point.x = 1.0 #1.0
    		self.fifth_line_point.y = -self.xmin
    		self.fifth_line_point.z = self.ymin
    		self.bounding_line_marker.points.append(self.fifth_line_point)

    		self.bound_array.markers.append(self.bounding_line_marker)
    		# bounding_line.publish(bounding_line_marker)

    	self.bound.publish(self.bound_array)

    def probabitydistri(self, sigma, mu, x):
    	self.u = (self.x - self.mu) / abs(self.sigma)
    	self.y = (1 / (math.sqrt(2 * math.pi) * abs(self.sigma))) * math.exp(-self.u * self.u / 2)
    	self.y = stats.norm(self.y)
    	sns.distplot(self.tvmonitor_datapoints, bins=20, kde=False, norm_hist=True)
    	plt.plot(self.rotation_y, self.y, label='single')
    	plt.legend()

    def pointofinter(self, xmin, xmax, ymin, ymax):
    	# if data
    	self.p1 = np.array([1/2, self.xmin/2, self.ymin/2])
    	self.p2 = np.array([1, self.xmax, self.ymax])
    	self.p3 = np.array([1, self.xmax, self.ymin])

    	# p1 = np.append(p1, tvmonitor_datapoints[np.random.choice(tvmonitor_datapoints.shape[0], 2, replace=False)])
    	# p2 = np.append(p2, tvmonitor_datapoints[np.random.choice(tvmonitor_datapoints.shape[0], 2, replace=False)])
    	# p3 = np.append(p3, tvmonitor_datapoints[np.random.choice(tvmonitor_datapoints.shape[0], 2, replace=False)])


    	#
    	self.v1 = self.p3 - self.p1
    	self.v2 = self.p2 - self.p1
    	# # print v1, v2
    	self.cp = np.cross(self.v1, self.v2)
    	# # print cp
    	self.a,self.b,self.c = self.cp
    	# print a,b,c
    	self.d = np.dot(self.cp, self.p3)
    	# print a ,b ,c, d
    	# print a,b,c,d
    	self.t = self.d - (self.a * self.x_pos ) + (self.b * self.y_pos ) + (self.c * self.z_pos ) / ((self.a * self.h_rotation_x) + (self.b * self.h_rotation_y) + (self.c * self.h_rotation_z))
    	# print (a * rotation_x) + (b * rotation_y) + (c * rotation_z)
    	self.poi_x = (self.a * self.x_pos * 6) + (self.a * self.t * self.h_rotation_x)
    	self.poi_y = (self.b * self.y_pos * 6) + (self.b * self.t * self.h_rotation_y)
    	self.poi_z = (self.c * self.z_pos * 6) + (self.c * self.t * self.h_rotation_z)
    	self.poi = np.array([self.poi_x, self.poi_y, self.poi_z])
    	# print poi
    	self.poi_marker_publisher = rospy.Publisher('poi_marker', Marker, queue_size=5)
    	self.poi_marker = Marker(type=Marker.SPHERE,id=0,lifetime=rospy.Duration(1.5),pose=Pose(Point(self.poi_x, self.poi_y, self.poi_z), Quaternion(w=1, x=0, y=0, z=0)),scale=Vector3(0.2, 0.2, 0.2),header=Header(frame_id='kinect2_link'),color=ColorRGBA(0.0, 1.0, 0.0, 0.8),text="whatsup")
    	self.poi_marker_publisher.publish(self.poi_marker)
    	return self.poi


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
