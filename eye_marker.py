#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
import rospy
from std_msgs.msg import Header, ColorRGBA, Int8
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PointStamped, Quaternion, Pose, Point, Vector3

def callback_left(data):
	x_pos = (data.point.x ) / (180 * np.pi)
	y_pos = (data.point.y ) / (180 * np.pi)
	z_pos = (data.point.z ) / (180 * np.pi)
	# print("left eye position")
	# print x_pos, y_pos, z_pos
	left_eye_marker_publisher = rospy.Publisher('left_eye', Marker, queue_size=5)
	left_eye_marker = Marker(type=Marker.SPHERE,id=0,lifetime=rospy.Duration(1.5),pose=Pose(Point(x_pos, y_pos, z_pos), Quaternion(w=1, x=0, y=0, z=0)),scale=Vector3(0.2, 0.2, 0.2),header=Header(frame_id='kinect2_rgb_optical_frame'),color=ColorRGBA(0.0, 1.0, 0.0, 0.8),text="whatsup")
	left_eye_line = rospy.Publisher('left_eye_line', Marker, queue_size=5)
	left_eye_line_marker = Marker()
	left_eye_line_marker.header.frame_id = "world"
	left_eye_line_marker.type = left_eye_line_marker.LINE_STRIP
	left_eye_line_marker.action = left_eye_line_marker.ADD

	# marker scale
	left_eye_line_marker.scale.x = 0.03
	left_eye_line_marker.scale.y = 0.03
	left_eye_line_marker.scale.z = 0.03

	# marker color
	left_eye_line_marker.color.a = 1.0
	left_eye_line_marker.color.r = 1.0
	left_eye_line_marker.color.g = 1.0
	left_eye_line_marker.color.b = 0.0

	# marker orientaiton
	left_eye_line_marker.pose.orientation.x = 0.0
	left_eye_line_marker.pose.orientation.y = 0.0
	left_eye_line_marker.pose.orientation.z = 0.0
	left_eye_line_marker.pose.orientation.w = 1.0

	# marker position
	# left_eye_line_marker.pose.position.x = x_pos
	# left_eye_line_marker.pose.position.y = y_pos
	# left_eye_line_marker.pose.position.z = z_pos

	# marker line pointsn the scripts direc
	left_eye_line_marker.points = []
	# first point
	first_line_point = Point()
	first_line_point.x = x_pos
	first_line_point.y = y_pos
	first_line_point.z = z_pos
	left_eye_line_marker.points.append(first_line_point)
	# second point
	second_line_point = Point()
	second_line_point.x = x_pos + 1.0
	second_line_point.y = y_pos + 1.0
	second_line_point.z = 0.0
	left_eye_line_marker.points.append(second_line_point)

	left_eye_line.publish(left_eye_line_marker)

	# left_eye_marker_publisher.publish(left_eye_marker)

	# left_eye_marker_publisher.publish(left_eye_marker)
	# rate = rospy.Rate(5)
	# triplePoints = [x_pos, y_pos, z_pos]
 #     #transform from x,y points to x,y,z points
 #    # for x,y,z in data.point:
	#    #  p = Point()
	#    #  p.x = x
	#    #  p.y = y
	#    #  p.z = zn the scripts direc
	#    #  triplePoints.append(p)

	# iterations = 0
 # 	while not rospy.is_shutdown() and iterations <= 10:
 # 		pub = rospy.Publisher("/my_frame", Marker, queue_size = 100)
 # 		marker = Marker()
 # 		marker.header.frame_id = "/kinect2_nonrotated_link"

 # 		marker.type = marker.POINTS
 # 		marker.action = marker.ADD
 # 		marker.pose.orientation.w = 1

 # 		marker.points = triplePoints;
 # 		t = rospy.Duration()
 # 		marker.lifetime = t
 # 		marker.scale.x = 0.4
 # 		marker.scale.y = 0.4
 # 		marker.scale.z = 0.4
 # 		marker.color.a = 1.0
 # 		marker.color.r = 1.0

 # 		pub.publish(marker)
 # 		iterations += 1
 # 		rate.sleep()

def callback_right(data):
	x_pos = (data.point.x ) / (180 * np.pi)
	y_pos = (data.point.y ) / (180 * np.pi)
	z_pos = (data.point.z ) / (180 * np.pi)

	right_eye_marker_publisher = rospy.Publisher('right_eye', Marker, queue_size=5)
	right_eye_marker = Marker(type=Marker.SPHERE,id=0,lifetime=rospy.Duration(1.5),pose=Pose(Point(x_pos, y_pos, z_pos), Quaternion(w=1, x=0, y=0, z=0)),scale=Vector3(0.2, 0.2, 0.2),header=Header(frame_id='kinect2_rgb_optical_frame'),color=ColorRGBA(0.0, 1.0, 0.0, 0.8),text="whatsup")
	right_eye_line = rospy.Publisher('right_eye_line', Marker, queue_size=5)
	right_eye_line_marker = Marker()
	right_eye_line_marker.header.frame_id = "world"
	right_eye_line_marker.type = right_eye_line_marker.LINE_STRIP
	right_eye_line_marker.action = right_eye_line_marker.ADD

	# marker scale
	right_eye_line_marker.scale.x = 0.03
	right_eye_line_marker.scale.y = 0.03
	right_eye_line_marker.scale.z = 0.03

	# marker color
	right_eye_line_marker.color.a = 1.0
	right_eye_line_marker.color.r = 1.0
	right_eye_line_marker.color.g = 1.0
	right_eye_line_marker.color.b = 0.0

	# marker orientaiton
	right_eye_line_marker.pose.orientation.x = 0.0
	right_eye_line_marker.pose.orientation.y = 0.0
	right_eye_line_marker.pose.orientation.z = 0.0
	right_eye_line_marker.pose.orientation.w = 1.0

	# marker position
	# right_eye_line_marker.pose.position.x = x_pos
	# right_eye_line_marker.pose.position.y = y_pos
	# right_eye_line_marker.pose.position.z = z_pos
	# marker line points
	right_eye_line_marker.points = []
	# first point
	first_line_point = Point()
	first_line_point.x = x_pos
	first_line_point.y = y_pos
	first_line_point.z = z_pos
	right_eye_line_marker.points.append(first_line_point)
	# second point
	second_line_point = Point()
	second_line_point.x = x_pos + 1.0
	second_line_point.y = y_pos + 1.0
	second_line_point.z = 0.0
	right_eye_line_marker.points.append(second_line_point)

	right_eye_line.publish(right_eye_line_marker)

	# right_eye_marker_publisher.publish(right_eye_marker)
	# print x_pos, y_pos, z_pos
	# print("right eye position")
	# print x_pos, y_pos, z_pos

	# def show_text_in_rviz(marker_publisher, text):

def yolo(data):
	# print data.bounding_boxes
	# print ''
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

		bounding_line_marker = Marker()
		bounding_line_marker.header.frame_id = "camera_link"
		bounding_line_marker.id = x
		bounding_line_marker.type = bounding_line_marker.LINE_STRIP
		bounding_line_marker.action = bounding_line_marker.ADD
		# my_data.append([xmin, xmax, ymin, ymax])
		# print ymin
		# marker scale
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
		first_line_point.x = 0.0
		first_line_point.y = -xmin
		first_line_point.z = ymin
		bounding_line_marker.points.append(first_line_point)
		# second point
		second_line_point = Point()
		second_line_point.x = 0.0
		second_line_point.y = -xmax
		second_line_point.z = ymin
		bounding_line_marker.points.append(second_line_point)

		third_line_point = Point()
		third_line_point.x = 0.0
		third_line_point.y = -xmax
		third_line_point.z = ymax
		bounding_line_marker.points.append(third_line_point)

		forth_line_point = Point()
		forth_line_point.x = 0.0 #1.0
		forth_line_point.y = -xmin
		forth_line_point.z = ymax
		bounding_line_marker.points.append(forth_line_point)

		fifth_line_point = Point()
		fifth_line_point.x = 0.0 #1.0
		fifth_line_point.y = -xmin
		fifth_line_point.z = ymin
		bounding_line_marker.points.append(fifth_line_point)

		bound_array.markers.append(bounding_line_marker)
		# bounding_line.publish(bounding_line_marker)

	bound.publish(bound_array)

if __name__ == '__main__':

# ROS node initialization
	rospy.init_node('listener', anonymous=True)
	# rospy.sleep(0.5)
	# show_text_in_rviz(marker_publisher, 'Hello world!')
# Create Subscribers
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
