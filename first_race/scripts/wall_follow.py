#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import sys
import math
import numpy as np

#PID CONTROL PARAMS
kp = 1.115
kd = 0.08
ki = 0.0000015

#kp = 0.42
#kd = 0.01
#ki = 0.0005


L = 0.4

# L = 1

servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 1.8 # meters
DESIRED_DISTANCE_LEFT = 1.8
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters





class WallFollow(object):
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)


    def getRange(self, data, angle):

        index = (angle+90) * (len(data.ranges)/360)
        distance = data.ranges[index]

        return distance

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = -kp*error + ki*(integral+ error*servo_offset) + kd*(error-prev_error)/servo_offset
        
        prev_error = error

        integral = integral + error*servo_offset


        velocity = 3


        angle = np.clip(angle,-0.4,0.4)


        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity


        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        error = DESIRED_DISTANCE_LEFT - leftDist
        return error

    def followRight(self, data, rightDist):
        error = DESIRED_DISTANCE_RIGHT - rightDist
        return error

    def scan_callback(self, data):

        global servo_offset
        servo_offset = L*1.0/ VELOCITY
        a = self.getRange(data, 135)
        b = self.getRange(data, 180)

        theta = math.radians(45)
    
        angle1 = math.atan((a*math.cos(theta)-b)/a*math.sin(theta))
        Dist_t1 = b*math.cos(angle1) + L*math.sin(angle1)
        leftDist = Dist_t1
        error = self.followLeft(data, leftDist)

        self.pid_control(error, VELOCITY)





if __name__ == '__main__':
    rospy.init_node('WallFollow_node')
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()
