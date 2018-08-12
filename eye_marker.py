#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker, MarkerArray
import math, time, matplotlib.pyplot as plt
from scipy.stats import norm
import numpy as np, seaborn as sns
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from std_msgs.msg import Header, ColorRGBA, Int8
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PointStamped, Quaternion, Pose, Point, Vector3, TransformStamped

rotation_x = None
rotation_y = None
rotation_z = None
rotation_w = None
trans_x = None
trans_y = None
trans_z = None
eq_tv = None
x_pos= None
y_pos= None
z_pos= None
tvmonitor_datapoints = np.array([])
laptop_datapoints = np.array([])
cup_datapoints = np.array([])
cellphone_datapoints = np.array([])
tv_monitor_variance = 0
laptop_variance = 0
cup_variance = 0
cellphone_variance = 0
timeout = time.time() + 60*.5 #2 min time
tvmonitor_mean = None
def callback_tf(data):
	global rotation_x, rotation_y, rotation_z, rotation_w, trans_x, trans_y, trans_z
	if data.transforms[0].child_frame_id == "world_gazetwoeyes":

		rotation_x = data.transforms[0].transform.rotation.x
		rotation_y = data.transforms[0].transform.rotation.y
		rotation_z = data.transforms[0].transform.rotation.z
		rotation_w = data.transforms[0].transform.rotation.w


def callback_left(data):
	global x_pos, y_pos, z_pos
	x_pos = (data.point.x ) / (180 * np.pi)
	y_pos = (data.point.y ) / (180 * np.pi)
	z_pos = (data.point.z ) / (180 * np.pi)
	# print("left eye position")
	# print x_pos, y_pos, z_pos
	left_eye_marker_publisher = rospy.Publisher('left_eye', Marker, queue_size=5)
	left_eye_marker = Marker(type=Marker.SPHERE,id=0,lifetime=rospy.Duration(1.5),pose=Pose(Point(x_pos, y_pos, z_pos), Quaternion(w=1, x=0, y=0, z=0)),scale=Vector3(0.2, 0.2, 0.2),header=Header(frame_id='kinect2_rgb_optical_frame'),color=ColorRGBA(0.0, 1.0, 0.0, 0.8),text="whatsup")
	left_eye_line = rospy.Publisher('left_eye_line', Marker, queue_size=5)
	left_eye_line_marker = Marker()
	left_eye_line_marker.header.frame_id = "kinect2_link"
	left_eye_line_marker.type = left_eye_line_marker.LINE_STRIP
	left_eye_line_marker.action = left_eye_line_marker.ADD

	# marker scale
	left_eye_line_marker.scale.x = 0.03
	left_eye_line_marker.scale.y = 0.03
	left_eye_line_marker.scale.z = 0.03

	# marker color
	left_eye_line_marker.color.a = 1.0
	left_eye_line_marker.color.r = 0.0
	left_eye_line_marker.color.g = 1.0
	left_eye_line_marker.color.b = 1.0

	# marker orientaiton
	left_eye_line_marker.pose.orientation.x = rotation_x
	left_eye_line_marker.pose.orientation.y = rotation_y
	left_eye_line_marker.pose.orientation.z = rotation_z
	left_eye_line_marker.pose.orientation.w = rotation_w

	# marker positionrotation_x
	# left_eye_line_marker.pose.position.x = x_pos
	# left_eye_line_marker.pose.position.y = y_pos
	# left_eye_line_marker.pose.position.z = z_pos

	# marker line pointsn the scripts direc
	left_eye_line_marker.points = []
	# first point
	first_line_point = Point()
	first_line_point.x = x_pos
	first_line_point.y = y_pos - 0.4
	first_line_point.z = z_pos
	# second point

	# first_line_point.x = x_pos
	# first_line_point.y = y_pos - 0.8
	# first_line_point.z = z_pos
	left_eye_line_marker.points.append(first_line_point)
	# second point
	second_line_point = Point()
	second_line_point.x = x_pos - 1.1
	second_line_point.y = y_pos - 0.5
	second_line_point.z = z_pos
	# second_line_point.x = x_pos - 1
	# second_line_point.y = y_pos - 1.1
	# second_line_point.z = z_pos
	left_eye_line_marker.points.append(second_line_point)

	left_eye_line.publish(left_eye_line_marker)




def callback_right(data):
	x_pos = (data.point.x ) / (180 * np.pi)
	y_pos = (data.point.y ) / (180 * np.pi)
	z_pos = (data.point.z ) / (180 * np.pi)

	right_eye_marker_publisher = rospy.Publisher('right_eye', Marker, queue_size=5)
	right_eye_marker = Marker(type=Marker.SPHERE,id=0,lifetime=rospy.Duration(1.5),pose=Pose(Point(x_pos, y_pos, z_pos), Quaternion(w=1, x=0, y=0, z=0)),scale=Vector3(0.2, 0.2, 0.2),header=Header(frame_id='kinect2_rgb_optical_frame'),color=ColorRGBA(0.0, 1.0, 0.0, 0.8),text="whatsup")
	right_eye_line = rospy.Publisher('right_eye_line', Marker, queue_size=5)
	right_eye_line_marker = Marker()
	right_eye_line_marker.header.frame_id = "kinect2_link"
	right_eye_line_marker.type = right_eye_line_marker.LINE_STRIP
	right_eye_line_marker.action = right_eye_line_marker.ADD

	# marker scale
	right_eye_line_marker.scale.x = 0.03
	right_eye_line_marker.scale.y = 0.03
	right_eye_line_marker.scale.z = 0.03
	# marker color
	right_eye_line_marker.color.a = 1.0
	right_eye_line_marker.color.r = 0.0
	right_eye_line_marker.color.g = 1.0
	right_eye_line_marker.color.b = 0.0

	# marker orientaitonworld
	right_eye_line_marker.pose.orientation.x = rotation_x
	right_eye_line_marker.pose.orientation.y = rotation_y
	right_eye_line_marker.pose.orientation.z = rotation_z
	right_eye_line_marker.pose.orientation.w = rotation_w

	# marker position
	# right_eye_line_marker.pose.position.x = x_pos
	# right_eye_line_marker.pose.position.y = y_pos
	# right_eye_line_marker.pose.position.z = z_pos
	# marker line points
	right_eye_line_marker.points = []
	# first point
	first_line_point = Point()
	first_line_point.x = x_pos
	first_line_point.y = y_pos - 0.7
	first_line_point.z = z_pos
	right_eye_line_marker.points.append(first_line_point)
	# second point
	second_line_point = Point()
	second_line_point.x = x_pos - 1.1
	second_line_point.y = y_pos - 0.6
	second_line_point.z = z_pos
	right_eye_line_marker.points.append(second_line_point)

	right_eye_line.publish(right_eye_line_marker)

	# right_eye_marker_publisher.publish(right_eye_marker)
	# print x_pos, y_pos, z_pos
	# print("right eye position")
	# print x_pos, y_pos, z_pos

	# def show_text_in_rviz(marker_publisher, text):

def yolo(data):

	global tvmonitor_datapoints, tvmonitor_mean, laptop_datapoints, cup_datapoints, cellphone_datapoints, laptop_variance, cup_variance, cellphone_variance, timeout, tv_monitor_variance
	bound = rospy.Publisher('bound', MarkerArray, queue_size=1)
	# bound_array.header.frame_id = "camera_link"
	# bound_array.type = bound_array.LINE_STRIP
	# bound_array.action = bound_array.ADD
	bound_array = MarkerArray()

#CLEAR stupid markers
	stupid_mark_array = MarkerArray()
	stupid_marker = Marker()
	stupid_marker.action = stupid_marker.DELETEALL
	stupid_mark_array.markers.append(stupid_marker)
	# bounding_line = rospy.Publisher('bound', Marker, queue_size=5)
	bound.publish(stupid_mark_array)
#CLEAR stupid markers

	for x in range(len(data.bounding_boxes)):
		# print data.bounding_boxes[x]

		xmin = data.bounding_boxes[x].xmin / (180 * np.pi)
		xmax = data.bounding_boxes[x].xmax / (180 * np.pi)
		ymin = data.bounding_boxes[x].ymin / (180 * np.pi)
		ymax = data.bounding_boxes[x].ymax / (180 * np.pi)

		# tvmonitor_boundingbox = []
		# print xmin
		if time.time() < timeout:
			if data.bounding_boxes[x].Class == "tvmonitor":
				tvmonitor_datapoints = np.append(tvmonitor_datapoints, [xmin, xmax, ymin, ymax])
			# p1 = np.array([1, -xmin, ymin])
			# p2 = np.array([1, -xmax, ymin])
			# p3 = np.array([1, -xmax, ymax])

			# print('the equation is {0}x +{1}y + {2}z = {3}'.format(a,b,c,d))
		#
		#
		# 	elif data.bounding_boxes[x].Class == "laptop":
		# 		laptop_datapoints = np.append(laptop_datapoints, [xmin, xmax, ymin, ymax])
		# 		laptop_variance = np.std(laptop_datapoints)
		# 	elif data.bounding_boxes[x].Class == "cell phone":
		# 		cellphone_datapoints = np.append(cellphone_datapoints, [xmin, xmax, ymin, ymax])
		# 		cellphone_variance = np.std(cellphone_datapoints)
		# 	elif data.bounding_boxes[x].Class == "cup":
		# 		cup_datapoints = np.append(cup_datapoints, [xmin, xmax, ymin, ymax])
		# 		cup_variance = np.std(cup_datapoints)
		# 	else:
		# 		pass
		else:
			pointofinter(tvmonitor_datapoints)
		# 	tv_monitor_variance = np.std(tvmonitor_datapoints)
		# 	tvmonitor_mean = np.mean(tvmonitor_datapoints)
		# 	probabitydistri(tvmonitor_mean, tv_monitor_variance, rotation_y)


				# print tv_monitor_variance

		bounding_line_marker = Marker()
		bounding_line_marker.header.frame_id = "kinect2_link"
		bounding_line_marker.id = x
		bounding_line_marker.type = bounding_line_marker.LINE_STRIP
		bounding_line_marker.action = bounding_line_marker.ADD
		# my_data.append([xmin, xmax, ymin, ymax])
		# print ymin
		# marker scaleworld
		bounding_line_marker.scale.x = 0.03
		bounding_line_marker.scale.y = 0.03
		bounding_line_marker.scale.z = 0.03

		# marker color
		bounding_line_marker.color.a = 1.0
		bounding_line_marker.color.r = 1.0
		bounding_line_marker.color.g = 1.0
		bounding_line_marker.color.b = 0.0

		# marker orientaiton
		bounding_line_marker.pose.orientation.x = 0.0
		bounding_line_marker.pose.orientation.y = 0.0
		bounding_line_marker.pose.orientation.z = 0.0
		bounding_line_marker.pose.orientation.w = 1.0

		bounding_line_marker.points = []


		# first point
		first_line_point = Point()
		first_line_point.x = 1.0
		first_line_point.y = -xmin
		first_line_point.z = ymin
		bounding_line_marker.points.append(first_line_point)
		# second point
		second_line_point = Point()
		second_line_point.x = 1.0
		second_line_point.y = -xmax
		second_line_point.z = ymin
		bounding_line_marker.points.append(second_line_point)

		third_line_point = Point()
		third_line_point.x = 1.0
		third_line_point.y = -xmax
		third_line_point.z = ymax
		bounding_line_marker.points.append(third_line_point)

		forth_line_point = Point()
		forth_line_point.x = 1.0 #1.0
		forth_line_point.y = -xmin
		forth_line_point.z = ymax
		bounding_line_marker.points.append(forth_line_point)

		fifth_line_point = Point()
		fifth_line_point.x = 1.0 #1.0
		fifth_line_point.y = -xmin
		fifth_line_point.z = ymin
		bounding_line_marker.points.append(fifth_line_point)

		bound_array.markers.append(bounding_line_marker)
		# bounding_line.publish(bounding_line_marker)

	bound.publish(bound_array)

def probabitydistri(sigma, mu, x):
	u = (x - mu) / abs(sigma)
	y = (1 / (math.sqrt(2 * math.pi) * abs(sigma))) * math.exp(-u * u / 2)
	y = norm(y)
	sns.distplot(tvmonitor_datapoints, bins=20, kde=False, norm_hist=True)
	plt.plot(rotation_y, y, label='single')
	plt.legend()

def pointofinter(tvmonitor_datapoints):

	p1 = np.array(tvmonitor_datapoints[np.random.choice(tvmonitor_datapoints.shape[0], 3, replace=False)])
	p2 = np.array(tvmonitor_datapoints[np.random.choice(tvmonitor_datapoints.shape[0], 3, replace=False)])
	p3 = np.array(tvmonitor_datapoints[np.random.choice(tvmonitor_datapoints.shape[0], 3, replace=False)])
	#
	v1 = p3 - p1
	v2 = p2 - p1
	# # print v1, v2
	cp = np.cross(v1, v2)
	# # print cp
	a,b,c = cp
	d = np.dot(cp, p3)
	# print a ,b ,c, d
	t = d - (a * x_pos) + (b * y_pos) + (c * z_pos) / (a * rotation_x) + (b * rotation_y) + (c * rotation_z)
	# # print (a * rotation_x) + (b * rotation_y) + (c * rotation_z)
	poi_x = (a * x_pos) + (a * t * rotation_x)
	poi_y = (b * y_pos) + (b * t * rotation_y)
	poi_z = (c * z_pos) + (c * t * rotation_z)
	poi = np.array([poi_x, poi_y, poi_z])


	poi_marker_publisher = rospy.Publisher('poi_marker', Marker, queue_size=5)
	poi_marker = Marker(type=Marker.SPHERE,id=0,lifetime=rospy.Duration(1.5),pose=Pose(Point(x_pos, y_pos, z_pos), Quaternion(w=1, x=0, y=0, z=0)),scale=Vector3(0.2, 0.2, 0.2),header=Header(frame_id='kinect2_link'),color=ColorRGBA(0.0, 1.0, 0.0, 0.8),text="whatsup")
	poi_marker_publisher.publish(poi_marker)

if __name__ == '__main__':
	# ROS node initialization
	rospy.init_node('listener', anonymous=True)
	# rospy.sleep(0.5)
	# show_text_in_rviz(marker_publisher, 'Hello world!')
	# Create Subscribers
	# rospy.Subscriber("/twoeyes/subject/lefteye/gazeimage", Image, callback_image, queue_size=1)
	rospy.Subscriber("/tf", TFMessage, callback_tf, queue_size=1)
	rospy.Subscriber("/new/subject/lefteye/position", PointStamped, callback_left, queue_size=1)
	rospy.Subscriber("/new/subject/righteye/position", PointStamped, callback_right, queue_size=1)

	rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, yolo, queue_size=1)
	# publishShape()
	# print marker_sub[0]
	# Create Publishers
	# pcl_objects_pub = rospy.Publisher("/pcl_objects", pc2.PointCloud2, queue_size=1)
	# pcl_table_pub = rospy.Publisher("/pcl_table", pc2.PointCloud2, queue_size=1)
	# pcl_cluster_pub = rospy.Publisher("/pcl_cluster", pc2.PointCloud2, queue_size=1)

	# Initialize color_list
	# get_color_list.color_list = []

	# Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()
