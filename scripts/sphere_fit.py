#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math

from robot_vision_lectures.msg import XYZarray, SphereParams
from cv_bridge import CvBridge


points_received = False
points = np.array([])
sphere_params = SphereParams()

# topic /xyz_cropped_ball
def point_callback(array):
	# Allow reassignment of global variables
	global points_received
	global points
	# Save the given data
	points = array
	points_received = True


def set_sphere_params(data):
	global sphere_params
	# Get A and B arrays
	A, B = [], []
	for point in data.points:
		A.append([point.x*2, point.y*2, point.z*2, 1])
		B.append(point.x**2 + point.y**2 + point.z**2)
	# Calculate P
	P = np.linalg.lstsq(A, B, rcond=None)[0]
	# Use P to set the attributes of the sphere_params
	sphere_params.xc = P[0]
	sphere_params.yc = P[1]
	sphere_params.zc = P[2]
	sphere_params.radius = math.sqrt(P[0]**2 + P[1]**2 + P[2]**2 + P[3])

if __name__ == "__main__":
	# initialize ros node
	rospy.init_node('robotics_lab5', anonymous=True)
	# declare pub and sub
	point_sub = rospy.Subscriber('xyz_cropped_ball', XYZarray, point_callback)
	point_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)
	
	# set the frequency to 10 ms
	rate = rospy.Rate(10)
	
	# main loop for publishing
	while not rospy.is_shutdown():
		# make sure we've received data to work on
		if points_received:
			# Calculate params needed
			set_sphere_params(points)
			# Publish the results
			point_pub.publish(sphere_params)
		rate.sleep()


