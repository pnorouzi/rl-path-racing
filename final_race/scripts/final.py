#!/usr/bin/env python

import os
import os.path as osp
import csv
import numpy as np
import torch
import torch.nn as nn
import gym
from gym import spaces
import tf

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import sys




folder_name = './Model'
FORWARD = 1.15
WHEELBASE= 0.3
SAFETY_RADIUS = 0.15
TTC_THRESHOLD = 0.65
MINTTCSPEED = 3.0



def mlp(sizes, activation, output_activation=nn.Identity):
    layers = []
    for j in range(len(sizes)-1):
        act = activation if j < len(sizes)-2 else output_activation
        layers += [nn.Linear(sizes[j], sizes[j+1]), act()]
    return nn.Sequential(*layers)


class MLPActionSelector(nn.Module):
    '''
    Soft parameterization of q value logits,
    pi_log = (1/Z)*(e^((v(x)/alpha) - min((v(x)/alpha)))
    If determinstic take max value as action,
    Else (stochastic),
    Sample from multinomial of the soft logits.
    '''
    def __init__(self, alpha, act_dim):
        super(MLPActionSelector,self).__init__()
        self.alpha = alpha
        self.act_dim = act_dim

        self.logsoftmax = nn.LogSoftmax(dim=1)


    def forward(self, q, action_mask, deterministic=False, with_logprob=True):
        #Divide by temperature term, alpha
        q_soft = q/self.alpha

        # Mask out actions not available
        mask = np.ones(self.act_dim, dtype=bool)  #16 paths + optimal path
        mask[list(action_mask)] = False
        try:
            q_soft[:, mask] = -float("Inf")
        except:
            q_soft = q_soft.unsqueeze(0)
            q_soft[:, mask] = -float("Inf")

        pi_log = self.logsoftmax(q_soft)

        if deterministic:
            mu = torch.argmax(pi_log)
            pi_action = mu      
        else:
            try:
                q_log_dist = torch.distributions.multinomial.Multinomial(1, logits=pi_log)
                action = q_log_dist.sample()
                pi_action = torch.argmax(action, dim=1, keepdim=True)

            except: #This case happens if no paths are available -> 0.5 vel and 0 steer, force it crash and learn
                pi_action = torch.argmax(pi_log, dim=1, keepdim=True)   
                pi_action = (torch.ones([pi_log.shape[0],1]) * 15).type(torch.long)

        if with_logprob:
            logp_pi = torch.gather(pi_log,1,pi_action)
        else:
            logp_pi = None
        
        return pi_action, logp_pi

class MLPQFunction(nn.Module):

    def __init__(self, obs_dim, act_dim, hidden_sizes, activation):
        super(MLPQFunction, self).__init__()
        self.vf_mlp = mlp([obs_dim] + list(hidden_sizes) + [act_dim], activation)

    # v(x)
    def values(self, obs):
        v_x = self.vf_mlp(obs)
        return v_x

    # q(x,a)
    def forward(self, obs, act):
        v_x = self.vf_mlp(obs)
        q = torch.gather(v_x, 1, act.type(torch.LongTensor))

        return q


class MLPActorCritic(nn.Module):

    def __init__(self, observation_space, action_space, alpha, hidden_sizes=(256,256,128),
                 activation=nn.ReLU):
        super(MLPActorCritic,self).__init__()

        obs_dim = observation_space.shape[0]
        act_dim = action_space.n

        # build policy and value functions
        self.pi = MLPActionSelector(alpha, act_dim)
        self.q1 = MLPQFunction(obs_dim, act_dim, hidden_sizes, activation)
        self.q2 = MLPQFunction(obs_dim, act_dim,  hidden_sizes, activation)

    def act(self, obs, action_mask, deterministic=False):
        with torch.no_grad():
            v1 = self.q1.values(obs)
            v2 = self.q2.values(obs)

            a, _ = self.pi(v1+v2, action_mask, deterministic, False)
            # Tensor to int
            return int(a)


def load_pytorch_policy(fpath, itr='', deterministic=False):
# def load_pytorch_policy(fpath, deterministic=False):
    """ Load a pytorch policy saved with Spinning Up Logger."""
    
    fname = osp.join(fpath,itr,'model.pt')
    # fname = osp.join(fpath, 'pyt_save', 'model.pt')
    print('\n\nLoading from %s.\n\n'%fname)

    action_space = spaces.Discrete(17) #16 paths + optimal
    limit = np.zeros((2,256))
    limit[0,:(117*2)] = 0  #subsampled Lidar
    limit[1,:(117*2)] = 10

    limit[0,(117*2):(117*2)+2] = -10  #Position of the other car wrt our car
    limit[1,(117*2):(117*2)+2] = 10

    limit[0,(117*2)+2] = -2*np.pi  #Orientation of the other car wrt our car
    limit[1,(117*2)+2] = 2*np.pi

    limit[0,(117*2)+3:(117*2)+5] = -10 #Velocity of the other car wrt our car
    limit[1,(117*2)+3:(117*2)+5] = 10

    limit[0,(117*2)+5:] = 0    #Distance to all paths + optimal path
    limit[1,(117*2)+5:] = 10

    observation_space = spaces.Box(limit[0], limit[1], dtype=np.float32)

    model = MLPActorCritic(observation_space,action_space, alpha = 0.2)

    model.load_state_dict(torch.load(fname))
    model.eval()

    # make function for producing an action given a single state
    def get_action(x, action_mask, deterministic=True):
        with torch.no_grad():
            x = torch.as_tensor(x, dtype=torch.float32)
            action = model.act(x, action_mask, deterministic)
        return action

    print('Policy Loaded Successfully!')
    return get_action



class SqnDriver(object):

    def __init__(self,policy,csv_path):

        self.waypoints = np.zeros((len(csv_path),1000,2))
        count = 0
        for path in csv_path:
            self.waypoints[count,:,:] = np.loadtxt(path, ndmin=2,delimiter=',')
            count += 1
        # self.current_path = 6
        self.waypoint  = self.waypoints[6,0][np.newaxis,:]
        self.path_waypoints = np.zeros((len(csv_path),4))
        self.aval_paths = set(range(len(csv_path)))
        self.observations = {'scans':[0,0,0],'poses_x':[0,0], 'poses_y':[0,0], 'poses_theta':[0,0], 'linear_vels_x': [0,0]}
        self.policy = policy

        # print(self.waypoint.shape)
        pose_topic = '/odom'
        lidarscan_topic = '/scan'
        opp_pose_topic = '/opp_odom'


        first_scan = rospy.wait_for_message(lidarscan_topic,LaserScan,2.0)
        self.observations['scans'][0] = first_scan.ranges
        self.observations['scans'][1] = first_scan.angle_min
        self.observations['scans'][2] = first_scan.angle_max
        ranges = np.array(first_scan.ranges)
        self.angles = np.linspace(first_scan.angle_min, first_scan.angle_max, num=ranges.shape[0])

        first_pose = rospy.wait_for_message(pose_topic,Odometry,2.0)
        quaternion = np.array([first_pose.pose.pose.orientation.x, 
                           first_pose.pose.pose.orientation.y, 
                           first_pose.pose.pose.orientation.z, 
                           first_pose.pose.pose.orientation.w])

        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = [ first_pose.pose.pose.position.x, first_pose.pose.pose.position.y]
        # self.speed = first_pose.twist.twist.linear.x


        self.observations['poses_x'][0] = first_pose.pose.pose.position.x
        self.observations['poses_y'][0] = first_pose.pose.pose.position.y
        self.observations['poses_theta'][0] = self.euler[2]
        self.observations['linear_vels_x'][0] = first_pose.twist.twist.linear.x
        

        first_opp_pose = rospy.wait_for_message(opp_pose_topic,Odometry,2.0)
        quaternion = np.array([first_opp_pose.pose.pose.orientation.x, 
                           first_opp_pose.pose.pose.orientation.y, 
                           first_opp_pose.pose.pose.orientation.z, 
                           first_opp_pose.pose.pose.orientation.w])

        self.opp_euler = tf.transformations.euler_from_quaternion(quaternion)
        self.opp_position = [ first_opp_pose.pose.pose.position.x, first_opp_pose.pose.pose.position.y]
        # self.opp_speed = first_opp_pose.twist.twist.linear.x

        self.observations['poses_x'][1] = first_opp_pose.pose.pose.position.x
        self.observations['poses_y'][1] = first_opp_pose.pose.pose.position.y
        self.observations['poses_theta'][1] = self.opp_euler[2]
        self.observations['linear_vels_x'][1] = first_opp_pose.twist.twist.linear.x



        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=10)
        rospy.Subscriber( pose_topic, Odometry, self.pose_callback, queue_size=10)
        rospy.Subscriber( opp_pose_topic, Odometry, self.opp_odom_callback, queue_size=10)

        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    def lidar_callback(self, lidar_msg):
        self.observations['scans'][0] = lidar_msg.ranges


    def find_waypoints(self,current_position, current_theta):

        point_dist =  np.sqrt(np.sum(np.square(self.waypoints[:, :,0:2]-current_position), axis=2))
        
        point_index = np.where(np.abs(point_dist-FORWARD)< 0.2)

        car_points = self.global_to_car(self.waypoints[point_index], current_position,current_theta)

        abs_tan_vals = np.abs(np.arctan(car_points[:,0]/car_points[:,1]))

        good_points = np.all([abs_tan_vals < np.pi/2, car_points[:,0]>0],axis = 0)

        waypoint_idx_paths = {k: v for v, k in enumerate(point_index[0][good_points])}
        # self.test = point_index[0][good_points]
        for key in waypoint_idx_paths.keys():
            self.path_waypoints[key,:2] = self.waypoints[key,point_index[1][good_points][waypoint_idx_paths[key]],:]
            self.path_waypoints[key,2:] = self.waypoints[key,point_index[1][good_points][waypoint_idx_paths[key]-1],:]

        self.aval_paths = set(point_index[0][good_points])
        # print(self.aval_paths)


    @staticmethod
    def global_to_car(points, current_position, current_theta):

        points = points - current_position

        R_mat = np.array([[np.cos(current_theta),np.sin(current_theta)],
                        [-np.sin(current_theta),np.cos(current_theta)]])

        car_points = np.dot(points,R_mat.T)


        return car_points


    def process_obs(self):

        '''
        Args: 
            obs: observation that we get out of each step
        Returns:
            obs_array: (numpy array shape (256,)) I also find waypoints and available paths here
        '''

        ## First lets subsample from the lidar: lets do

        num_subsample = 234

        obs_array = np.zeros((256,))

        obs_array[239:] = 10 ## If lane is not accessible, this is the default distance to it

        ranges = np.array(self.observations['scans'][0])
        angles = np.linspace(self.observations['scans'][1], self.observations['scans'][2], num=ranges.shape[0])

        min_idx = int(((-100)*(np.pi/180)-angles[0])/(angles[1]-angles[0]))
        max_idx = int(((100)*(np.pi/180)-angles[0])/(angles[1]-angles[0]))

        # lidar_idxs = np.random.randint(min_idx,max_idx,num_subsample)
        lidar_idxs = np.linspace(min_idx,max_idx,num=num_subsample).astype(int) #subsample lidar
        obs_array[:num_subsample] = ranges[lidar_idxs]


        ## lets get other cars orientation with respect us
        ## opp position global to our frame
        our_position = np.array([self.observations['poses_x'][0],self.observations['poses_y'][0]])
        opp_car_global = np.array([self.observations['poses_x'][1],self.observations['poses_y'][1]])

        opp_car_global = opp_car_global - our_position

        R_mat = np.array([[np.cos(self.observations['poses_theta'][0]),np.sin(self.observations['poses_theta'][0])],
            [-np.sin(self.observations['poses_theta'][0]),np.cos(self.observations['poses_theta'][0])]])

        pos_opp_our_frame  = np.dot(opp_car_global,R_mat.T)  ## shape (1,2) that gives their position wrt us
        theta_opp_our_frame = self.observations['poses_theta'][0] - self.observations['poses_theta'][1]  # one value that gives their theta wrt us
        obs_array[num_subsample:(num_subsample)+2] = pos_opp_our_frame
        obs_array[(num_subsample)+2:(num_subsample)+3] = theta_opp_our_frame

        ## Now their velocity with respect to us

        # first their velocity to global frame:
        vel_opp_frame = np.array([self.observations['linear_vels_x'][1],0])
        R_mat_opp = np.array([[np.cos(self.observations['poses_theta'][1]),np.sin(self.observations['poses_theta'][1])],
            [-np.sin(self.observations['poses_theta'][1]),np.cos(self.observations['poses_theta'][1])]])

        vel_opp_global  = np.dot(vel_opp_frame,R_mat_opp)

        # Opp velocity global to our local frame:

        vel_opp_our_frame  = np.dot(vel_opp_global,R_mat.T) ## shape (1,2) that gives their velocity wrt us

        obs_array[(num_subsample)+3:(num_subsample)+5] = vel_opp_our_frame


        ## Now we gotta find our distance to each lane including optimal lane

        self.find_waypoints(our_position, self.observations['poses_theta'][0]) ## finds the waypoints from all paths and also find available paths

        for path in self.aval_paths:
            point1 = self.path_waypoints[path,:2]
            point2 = self.path_waypoints[path,2:]

            denom = np.sqrt((point2[1]-point1[1])**2 + (point2[0]-point1[0])**2)
            if denom == 0:
                continue
            num = np.abs(((point2[1]-point1[1])*our_position[0]) - ((point2[0]-point1[0])*our_position[1]) + (point2[0]*point1[1]) - (point2[1]*point1[0]))

            obs_array[((num_subsample)+5) + path] = num/denom

        return obs_array

    def find_TTC(self,waypoint):
        goalx_veh = waypoint[0]
        goaly_veh = waypoint[1]

        waypoint_angle_car = np.arctan(goaly_veh/goalx_veh)


        waypoint_angle_car_idx = int((waypoint_angle_car+np.pi)/(self.angles[1]-self.angles[0]))
        waypoint_angle_car_idx = np.minimum(waypoint_angle_car_idx,self.ranges.shape[0]-1)
        waypoint_angle_car_idx = np.maximum(waypoint_angle_car_idx,0)
        min_val = self.ranges[waypoint_angle_car_idx]

        angle_circle = np.arctan(SAFETY_RADIUS/min_val)

        if waypoint_angle_car<0:
            min_idx = int((waypoint_angle_car-angle_circle-self.angles[0])/(self.angles[1]-self.angles[0]))
            max_idx = int((waypoint_angle_car+angle_circle-self.angles[0])/(self.angles[1]-self.angles[0]))
            # print('min ', min_idx, ' ', max_idx)
        else:
            min_idx = int((waypoint_angle_car-angle_circle-self.angles[0])/(self.angles[1]-self.angles[0]))
            max_idx = int((waypoint_angle_car+angle_circle-self.angles[0])/(self.angles[1]-self.angles[0]))

        lower_idx_0 = np.maximum(min_idx,0)    ##Look into this because of wrapping
        upper_idx_0 = np.minimum(max_idx,self.ranges.shape[0]-1)  ##Look into this because of wrapping

        TTC_denom = (np.maximum(MINTTCSPEED, self.speed)*np.cos(self.angles[lower_idx_0:upper_idx_0]))

        TTC_denom[TTC_denom<=0] = 0.00000001

        TTC_vals = self.ranges[lower_idx_0:upper_idx_0]/TTC_denom

        TTC_min = np.amin(TTC_vals)

        return TTC_min


    def get_actuation(self, pose_theta, lookahead_point, position, TTC=True):

        goal_veh= self.global_to_car(lookahead_point, position, pose_theta)

        newaction = -1
        if TTC == True:
            ##TTC avoidance
            current_TTC = self.find_TTC(goal_veh)
            if current_TTC <= TTC_THRESHOLD:
                # print('TTC is take over')
                Path_TTC_vals = {}
                goal_vals = {}
                for path_idx in self.aval_paths:
                    goal_vals[path_idx]= self.global_to_car(self.path_waypoints[path_idx,:2],position, pose_theta)
                    Path_TTC_vals[path_idx] = self.find_TTC(goal_vals[path_idx])

                newaction = max(Path_TTC_vals, key=Path_TTC_vals.get)
                # print('Chose: ', newaction)
                lookahead_point =self.path_waypoints[newaction,:2]
                goal_veh= goal_vals[newaction]


        L = np.sqrt((lookahead_point[0]-position[0])**2 +  (lookahead_point[1]-position[1])**2 )

        if np.abs(L) < 1e-6:
            return self.safe_speed, 0.
        arc = 2*goal_veh[1]/(L**2)
        angle = 0.33*arc
        steering_angle = np.clip(angle, -0.4, 0.4)
        speed = self.select_velocity(steering_angle)

        return speed, steering_angle

    def plan(self, action):
        #Choose the path to follow

        pose_x = self.observations['poses_x'][0]
        pose_y = self.observations['poses_y'][0]
        pose_theta = self.observations['poses_theta'][0]
        position = np.array([pose_x, pose_y])
        self.ranges = np.array(list(self.observations['scans'][0]))

        if action in self.aval_paths:
            lookahead_point = self.path_waypoints[action,:2]
            speed, steering_angle = self.get_actuation(pose_theta, lookahead_point, position)
            return speed, steering_angle

        return 0.0, 0.0

    
    def opp_odom_callback(self,odom_msg):

        quaternion = np.array([odom_msg.pose.pose.orientation.x, 
                           odom_msg.pose.pose.orientation.y, 
                           odom_msg.pose.pose.orientation.z, 
                           odom_msg.pose.pose.orientation.w])

        self.opp_euler = tf.transformations.euler_from_quaternion(quaternion)
        self.opp_position = [ odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]
        # self.opp_speed = first_opp_pose.twist.twist.linear.x

        self.observations['poses_x'][1] = odom_msg.pose.pose.position.x
        self.observations['poses_y'][1] = odom_msg.pose.pose.position.y
        self.observations['poses_theta'][1] = self.opp_euler[2]
        self.observations['linear_vels_x'][1] = odom_msg.twist.twist.linear.x


    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture
        quaternion = np.array([pose_msg.pose.pose.orientation.x, 
                           pose_msg.pose.pose.orientation.y, 
                           pose_msg.pose.pose.orientation.z, 
                           pose_msg.pose.pose.orientation.w])

        self.euler = tf.transformations.euler_from_quaternion(quaternion)
        self.position = [ pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
        self.speed = pose_msg.twist.twist.linear.x

        self.observations['poses_x'][0] = pose_msg.pose.pose.position.x
        self.observations['poses_y'][0] = pose_msg.pose.pose.position.y
        self.observations['poses_theta'][0] = self.euler[2]
        self.observations['linear_vels_x'][0] = pose_msg.twist.twist.linear.x


        processed_obs = self.process_obs()

        decided_path = self.policy(processed_obs, self.aval_paths)

        # print('Decidesd rl path: ',decided_path)

        velocity, steering_angle = self.plan(decided_path)

        # print(steering_angle)

        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
    

    def select_velocity(self, angle):
        if abs(angle) <= 5*np.pi/180:
            velocity  = 4.5
        elif abs(angle) <= 10*np.pi/180:
            velocity  = 4.25
        elif abs(angle) <= 15*np.pi/180:
            velocity = 4
        elif abs(angle) <= 20*np.pi/180:
            velocity = 3.5
        else:
            velocity = 2.5
        return velocity

if __name__ == '__main__':
    dirname = os.path.dirname(os.path.abspath(__file__)) + '/../waypoints/Multi-Paths2/'
    # print(dirname)
    policy = load_pytorch_policy(os.path.dirname(os.path.abspath(__file__)) + '/Model')
    rospy.init_node('Team5_Driver_node')

    path_nums = list(range(2,18))

    ego_csv_paths = []
    for num in path_nums:
        ego_csv_paths.append(dirname + '/multiwp%d.csv'%(num))

    ego_csv_paths.append(dirname + '/multiwp-opt.csv') ## adding the opt path

    pp = SqnDriver(policy,ego_csv_paths)
    rospy.spin()
