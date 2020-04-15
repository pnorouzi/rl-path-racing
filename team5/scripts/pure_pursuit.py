#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry



import os
import numpy as np
import tf
import math
import sys

path = os.path.dirname(os.path.abspath(__file__)) + '/../log/'
waypoint = np.loadtxt(path +  os.listdir(path)[0], ndmin=2,delimiter=',')
FORWARD = 0.8

class PurePursuit(object):
    """
    The" class that handles pure pursuit.
    """
    def __init__(self):
        self.waypoint  = waypoint[0]
        self.index = 0
        pose_topic = '/odom'


        self.pose_sub = rospy.Subscriber( pose_topic, Odometry, self.pose_callback, queue_size=1)

        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        
    

    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture
        quaternion = np.array([pose_msg.pose.pose.orientation.x, 
                           pose_msg.pose.pose.orientation.y, 
                           pose_msg.pose.pose.orientation.z, 
                           pose_msg.pose.pose.orientation.w])

        euler = tf.transformations.euler_from_quaternion(quaternion)
        position = [ pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y] 
       

        point_dist =  np.sqrt(np.sum(np.square(waypoint[:, 0:2]-position), axis=1))
        point_index = np.where(abs(point_dist-FORWARD)< 0.20)[0]
        #print(point_index)
        for index in point_index:
            l2_0 = [waypoint[index, 0]-position[0], waypoint[index,1]-position[1]]
            goalx_veh = math.cos(euler[2])*l2_0[0] + math.sin(euler[2])*l2_0[1]
            goaly_veh = -math.sin(euler[2])*l2_0[0] + math.cos(euler[2])*l2_0[1]

            if abs(math.atan(goalx_veh/goaly_veh)) <  0.75*np.pi/2 and goalx_veh>0 :
                 self.waypoint = waypoint[index] 
                 #print("point find ", index)
                 break

        
        l2_0 = [self.waypoint[0]-position[0], self.waypoint[1]-position[1]]
        goalx_veh = math.cos(euler[2])*l2_0[0] + math.sin(euler[2])*l2_0[1]
        goaly_veh = -math.sin(euler[2])*l2_0[0] + math.cos(euler[2])*l2_0[1]  

        # TODO: calculate curvature/steering angle
        L = math.sqrt((self.waypoint[0]-position[0])**2 +  (self.waypoint[1]-position[1])**2 )

        arc = 2*goaly_veh/(L**2)
        angle = 0.3*arc
        angle = np.clip(angle, -0.35, 0.35)
        velocity = self.select_velocity(angle)
 
        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
    

    def select_velocity(self, angle):
        if abs(angle) <= 5*math.pi/180:
            velocity  = 4
        elif abs(angle) <= 10*math.pi/180:
            velocity  = 3.5
        elif abs(angle) <= 15*math.pi/180:
            velocity = 3
        elif abs(angle) <= 20*math.pi/180:
            velocity = 2.5
        else:
            velocity = 2
        return velocity

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
