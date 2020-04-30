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

dirname = os.path.dirname(os.path.abspath(__file__)) + '/../waypoints/Multi-Paths2/'


paths = list(range(2,18))

waypoints = np.zeros((len(paths),1000,2))

print(len(paths))

count= 0
for i in paths:
    filename = 'multiwp' + str(i) + '.csv'
    path  = dirname + filename
    waypoints[count,:,:] = np.loadtxt(path, ndmin=2,delimiter=',')
    count+= 1

FORWARD = 1
safety_radius = 0.15
TTC_threshold= 0.65



class MultiLanePPWObject(object):

    def __init__(self):
        self.current_path = 6
        self.waypoint  = waypoints[6,0][np.newaxis,:]
        print(self.waypoint.shape)
        self.index = 0
        pose_topic = '/odom'
        lidarscan_topic = '/scan'

        self.path_waypoints = np.zeros((len(paths),2))


        first_scan = rospy.wait_for_message(lidarscan_topic,LaserScan,2.0)
        self.ranges = np.array(first_scan.ranges)
        self.angles = np.linspace(first_scan.angle_min, first_scan.angle_max, num=self.ranges.shape[0])

        first_pose = rospy.wait_for_message(pose_topic,Odometry,2.0)
        quaternion = np.array([first_pose.pose.pose.orientation.x, 
                           first_pose.pose.pose.orientation.y, 
                           first_pose.pose.pose.orientation.z, 
                           first_pose.pose.pose.orientation.w])

        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = [ first_pose.pose.pose.position.x, first_pose.pose.pose.position.y]
        self.speed = first_pose.twist.twist.linear.x

        self.TTC_threshold = TTC_threshold
        self.aval_paths = set(range(len(paths)))
        # print(self.aval_paths)
        


        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=10)

        rospy.Subscriber( pose_topic, Odometry, self.pose_callback, queue_size=10)

        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    def lidar_callback(self, lidar_msg):
        self.ranges = np.array(lidar_msg.ranges)
        self.angles = np.linspace(lidar_msg.angle_min, lidar_msg.angle_max, num=self.ranges.shape[0])
        # print(self.ranges.shape)


    def find_waypoints(self,waypoints_new):

        point_dist =  np.sqrt(np.sum(np.square(waypoints_new[:, :,0:2]-self.position), axis=2))
        
        point_index = np.where(np.abs(point_dist-FORWARD)< 0.2)

        car_points = self.global_to_car(waypoints_new[point_index])

        abs_tan_vals = np.abs(np.arctan(car_points[:,0]/car_points[:,1]))

        good_points = np.all([abs_tan_vals < np.pi/2, car_points[:,0]>0],axis = 0)

        waypoint_idx_paths = {k: v for v, k in enumerate(point_index[0][good_points])}
        for key in waypoint_idx_paths.keys():
            self.path_waypoints[key,:] = waypoints[key,point_index[1][good_points][waypoint_idx_paths[key]],:]
        self.aval_paths = set(point_index[0][good_points])
        return self.path_waypoints


    def global_to_car(self,points):

        points = points - self.position

        R_mat = np.array([[np.cos(self.euler[2]),np.sin(self.euler[2])],
                        [-np.sin(self.euler[2]),np.cos(self.euler[2])]])

        car_points = np.dot(points,R_mat.T)


        return car_points

    def find_TTC(self,waypoint):
        goalx_veh = waypoint[0]
        goaly_veh = waypoint[1]

        waypoint_angle_car = np.arctan(goaly_veh/goalx_veh)


        waypoint_angle_car_idx = int((waypoint_angle_car+np.pi)/(self.angles[1]-self.angles[0]))
        waypoint_angle_car_idx = np.minimum(waypoint_angle_car_idx,self.ranges.shape[0]-1)
        waypoint_angle_car_idx = np.maximum(waypoint_angle_car_idx,0)
        min_val = self.ranges[waypoint_angle_car_idx]

        angle_circle = np.arctan(safety_radius/min_val)

        if waypoint_angle_car<0:
            min_idx = int((waypoint_angle_car-angle_circle-self.angles[0])/(self.angles[1]-self.angles[0]))
            max_idx = int((waypoint_angle_car+angle_circle-self.angles[0])/(self.angles[1]-self.angles[0]))
            # print('min ', min_idx, ' ', max_idx)
        else:
            min_idx = int((waypoint_angle_car-angle_circle-self.angles[0])/(self.angles[1]-self.angles[0]))
            max_idx = int((waypoint_angle_car+angle_circle-self.angles[0])/(self.angles[1]-self.angles[0]))

        lower_idx_0 = np.maximum(min_idx,0)    ##Look into this because of wrapping
        upper_idx_0 = np.minimum(max_idx,self.ranges.shape[0]-1)  ##Look into this because of wrapping

        TTC_denom = (self.speed*np.cos(self.angles[lower_idx_0:upper_idx_0]))

        TTC_denom[TTC_denom<=0] = 0.00000001

        TTC_vals = self.ranges[lower_idx_0:upper_idx_0]/TTC_denom

        TTC_min = np.amin(TTC_vals)

        return TTC_min

    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture
        quaternion = np.array([pose_msg.pose.pose.orientation.x, 
                           pose_msg.pose.pose.orientation.y, 
                           pose_msg.pose.pose.orientation.z, 
                           pose_msg.pose.pose.orientation.w])

        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = [ pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
        self.speed = pose_msg.twist.twist.linear.x


        all_waypoints =self.find_waypoints(waypoints)


        self.waypoint = all_waypoints[self.current_path,:][np.newaxis,:]


        goal_veh= self.global_to_car(self.waypoint)

        current_TTC = self.find_TTC(goal_veh[0,:])

        if current_TTC <= self.TTC_threshold:
            print('Change Path!')
            Path_TTC_vals = {}
            goal_vals = {}
            for path_idx in self.aval_paths:
                # if path_idx != self.current_path:
                goal_vals[path_idx]= self.global_to_car(all_waypoints[path_idx,:][np.newaxis,:])
                Path_TTC_vals[path_idx] = self.find_TTC(goal_vals[path_idx][0,:])

            test = max(Path_TTC_vals, key=Path_TTC_vals.get)

            print('Chose: ', test)
            self.waypoint =all_waypoints[test,:][np.newaxis,:]
            goal_veh= goal_vals[test]
            self.current_path = test
            
        
        # TODO: calculate curvature/steering angle
        L = math.sqrt((self.waypoint[0,0]-self.position[0])**2 +  (self.waypoint[0,1]-self.position[1])**2 )

        arc = 2*goal_veh[0,1]/(L**2)
        angle = 0.33*arc
        angle = np.clip(angle, -0.4, 0.4)
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
            velocity  = 4.5
        elif abs(angle) <= 10*math.pi/180:
            velocity  = 4
        elif abs(angle) <= 15*math.pi/180:
            velocity = 3
        elif abs(angle) <= 20*math.pi/180:
            velocity = 2.5
        else:
            velocity = 2
        return velocity

if __name__ == '__main__':
    rospy.init_node('MultiLanePPWObject_node')
    pp = MultiLanePPWObject()
    rospy.spin()
