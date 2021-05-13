#!/usr/bin/env python33

import rospy
from nav_msgs.msg import Odometry
import gym
import rospy
import roslaunch
import time
import numpy as np
from gym import utils, spaces
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from gym.utils import seeding
import gym
import rospy
import time
import numpy as np
import tf
import time
from gym import utils, spaces
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty as EmptyTopicMsg
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from gym.utils import seeding
import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import argparse
import actionlib
import control_msgs.msg


#force_tourqe sensors
from geometry_msgs.msg._WrenchStamped import WrenchStamped
#force_tourqe sensors

t_x=0
t_y=0
t_z=0
f_x=0
f_y=0
f_z=0

def callback(msg):
    # log the message data to the terminal
    #rospy.loginfo("The value is %d", msg.)
    global f_x
    f_x=msg.wrench.force.x
    global f_y
    f_y=msg.wrench.force.y
    global f_z
    f_z=msg.wrench.force.z
    
    global t_x
    t_x=msg.wrench.torque.x
    global t_y
    t_y=msg.wrench.torque.y
    global t_z
    t_z=msg.wrench.torque.z

    
    rospy.loginfo( msg.wrench.force)
    rospy.loginfo( msg.wrench.torque)
    rospy.signal_shutdown(0)

def ft_sensors_listener():
    # start the node
    rospy.init_node("listener")
    # subscribe to the '/sr_tactile/touch/ff' topic
#    tactile_sub = rospy.Subscriber("/ft_sensor/raw", WrenchStamped, callback ,queue_size=10)



    #tactile_sub = rospy.Subscriber("/ft_sensor/raw", WrenchStamped, callback ,queue_size=10)
    

    tactile_sub = rospy.Subscriber("/ft_sensor/raw", WrenchStamped, callback ,queue_size=10)
    rospy.spin()


reg = register(
    id='UR5env-v0',
    entry_point='UR5_env:UR5Env',
)

traj = JointTrajectory()
traj.header = Header()
pts = JointTrajectoryPoint()
pts.time_from_start = rospy.Duration(0.1)


 # Joint names for UR5
traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                'wrist_3_joint']
class UR5Env(gym.Env):

    def __init__(self):
        rospy.init_node('send_joints')
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.pub = rospy.Publisher('/trajectory_controller/command',
                          JointTrajectory,
                          queue_size=10)

       	self.action_space = spaces.Discrete(3)
        self.reward_range = (-np.inf, np.inf)
       	self._seed()


    def discretize_observation (self,data,new_ranges):
    	discretized_ranges = []
        min_range = 0.2
        done = False
        mod = 10
        for i, item in enumerate(data):
            if (i%mod==0):
                if data[i] == float ('Inf') or np.isinf(data[i]):
                    discretized_ranges.append(6)
                elif np.isnan(data[i]):
                    discretized_ranges.append(0)
                else:
                    discretized_ranges.append(int(data[i]))
            if (min_range > data[i] > 0):
                done = True
        return discretized_ranges,done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        if action == 0:
            print("pup")
            traj.points = []
            j1 = .25
            j2 = -1.8
            j3 =  1.9
            j4 =  0.3
            j5 =  1.7
            j6 =  random.uniform(2.2,-2.2)
            pts.positions = [j1, j2, j3, j4, j5, j6]
            traj.points.append(pts)
            self.pub.publish( traj)
            ft_sensors_listener()
            rospy.loginfo(f_x)
            rospy.loginfo(f_y)
            rospy.loginfo(f_z)
            rospy.loginfo(t_x)
            rospy.loginfo(t_y)
            rospy.loginfo(t_z)
            rospy.sleep(1)

        elif action == 1: #LEFT
            print("puplishing 1")
            traj.points = []
            j1 = .25
            j2 = -1.8
            j3 =  1.9
            j4 =  0.3
            j5 =  1.7		# 3.2 --> -3.2
            j6 =  random.uniform(2.2,-2.2)
            pts.positions = [j1, j2, j3, j4, j5, j6]
            traj.points.append(pts)
            self.pub.publish(traj)
            ft_sensors_listener()
            rospy.loginfo(f_x)
            rospy.loginfo(f_y)
            rospy.loginfo(f_z)
            rospy.loginfo(t_x)
            rospy.loginfo(t_y)
            rospy.loginfo(t_z)
            rospy.sleep(1)

        elif action == 2: #RIGHT
            print("puplishing 3")
            traj.points = []
            j1 = .25
            j2 = -1.8
            j3 =  1.9
            j4 =  0.3
            j5 =  1.7
            j6 = random.uniform(2.2,-2.2)
            pts.positions = [j1, j2, j3, j4, j5, j6]
            print(j1, j2, j3, j4, j5, j6)
            traj.points.append(pts)
            self.pub.publish(traj)
            rospy.sleep(1)


        data = range(6)
        # x=0
        # while x > 10:
        #     x=x+1
        #     data=range(6)

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state,done = self.discretize_observation(data,5)

        if not done:
            if action == 0:
                reward = 5
            else:
                reward = 1
        else:
            reward = -200
            print reward

        return state, reward, done, {}

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        #read laser data
        data = range(6)


        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = self.discretize_observation(data,5)

        return state
