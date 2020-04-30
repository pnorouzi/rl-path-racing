#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

import os
import csv
import pdb
import numpy as np

# dirname = os.path.dirname(os.path.abspath(__file__)) + '/../waypoints/Multi-Paths/'


# waypoints = np.zeros((9,1000,2))

# # filename = 'multiwp' + str(1) + '.csv'
# # path  = dirname + filename
# # print(waypoints.shape)
# for i in range(1,9):
# 	filename = 'multiwp' + str(i) + '.csv'
# 	path  = dirname + filename
# 	waypoints[i,:,:] = np.loadtxt(path, ndmin=2,delimiter=',')
	# print(i)

# waypoints = np.loadtxt(path, ndmin=2,delimiter=',')
dirname = os.path.dirname(os.path.abspath(__file__)) + '/../waypoints/Multi-Paths2/'

paths = list(range(2,18))

waypoints = np.zeros((len(paths),1000,2))

count= 0
for i in paths:
    filename = 'multiwp' + str(i) + '.csv'
    path  = dirname + filename
    waypoints[count,:,:] = np.loadtxt(path, ndmin=2,delimiter=',')
    count+= 1


topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size="1")

rospy.init_node('view_node')
counter = 0

while not rospy.is_shutdown():
        markerArray = MarkerArray()
        for i in range(waypoints.shape[0]):
        	# print(i)
        	for j in range(waypoints.shape[1]):
        		if j % 2 == 0:
        			point = waypoints[i,j,:]
        			counter += 1
    			x = float(point[0])
    			y = float(point[1])
    			marker = Marker()
		        # print(x,y)
		        marker.header.frame_id = "/map"
		        marker.type = marker.SPHERE
		        marker.action = marker.ADD
		        marker.scale.x = 0.11
		        marker.scale.y = 0.11
		        marker.scale.z = 0.11
		        marker.color.a = 1.0
		        marker.color.r = 1.0
		        marker.color.g = 0.7
		        marker.color.b = 0.8
		        marker.pose.orientation.w = 1.0
		        marker.pose.position.x = x
		        marker.pose.position.y = y
		        marker.pose.position.z = 0
		        marker.id = counter
		        markerArray.markers.append(marker)

        # print('published')
        publisher.publish(markerArray)
        counter = 0

	rospy.sleep(0.3)
