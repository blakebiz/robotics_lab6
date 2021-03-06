#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math

from robot_vision_lectures.msg import XYZarray, SphereParams
from cv_bridge import CvBridge
from std_msgs.msg import Bool


points_received = False
points = np.array([])
sphere_params = SphereParams()

# set defaults for filtering
filter_in = [0, 0, 0, 0]
filter_out = [0, 0, .2, .02]
filter_gain = .05
filters = [filter_in, filter_out, filter_gain]
pause_toggle = False


# topic /pause_toggle
def pause_callback(data):
	global pause_toggle
	pause_toggle = data
	if data:
		filter_in = [0, 0, 0, 0]
		filter_out = [0, 0, .2, .02]
		filter_gain = .05
		filters = [filter_in, filter_out, filter_gain]


# topic /xyz_cropped_ball
def point_callback(array):
	# Allow reassignment of global variables
	global points_received
	global points
	# Save the given data
	points = array
	points_received = True


def low_pass_filter(sphere, filter_in, filter_out, filter_gain):
	for i, attr in enumerate(('xc', 'yc', 'zc', 'radius')):
		# filter param
		filter_in[i] = getattr(sphere, attr)
		filter_out[i] = filter_gain * filter_in[i] + (1 - filter_gain) * filter_out[i]
		# set current attribute to value obtained
		setattr(sphere, attr, filter_out[i])
	
	return sphere, [filter_in, filter_out, filter_gain]


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
	point_sub = rospy.Subscriber('/xyz_cropped_ball', XYZarray, point_callback)
	pause_sub = rospy.Subscriber('/pause_toggle', Bool, pause_callback)
	point_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)
	
	# set the frequency to 10 ms
	rate = rospy.Rate(10)
	
	
	
	# main loop for publishing
	while not rospy.is_shutdown():
		# make sure we've received data to work on
		if points_received and not pause_toggle:
			# Calculate params needed
			set_sphere_params(points)
			# apply low-pass filter
			sphere_params, filters = low_pass_filter(sphere_params, *filters)
			# Publish the results
			point_pub.publish(sphere_params)
		rate.sleep()


