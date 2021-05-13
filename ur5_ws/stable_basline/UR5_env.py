#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import collections
import geometry_msgs.msg
import numpy as np
import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import argparse
import actionlib
import control_msgs.msg

#!/usr/bin/envpython

import rospy

import rospy


#force_tourqesensors
from geometry_msgs.msg._WrenchStamped import WrenchStamped
#force_tourqesensors

import rospy
from nav_msgs.msg import Odometry
import gym
import rospy
import roslaunch
import time
import numpy as np
from gym import utils,spaces
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from gym.utils import seeding
import gym
import rospy
import time

import tf
import time
from gym import utils,spaces
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

reg=register(
id='UR5env-v0',
entry_point='UR5_env:UR5Env',)


class UR5Env(gym.Env):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_group_python_interface',anonymous=True)
        self.unpause=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        self.pause=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.reset_proxy=rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
        self.robot=moveit_commander.RobotCommander()
        self.arm_group=moveit_commander.MoveGroupCommander("arm")

        self.reward_range=(-np.inf,np.inf)
                

        self.depth_axis,self.elvation_dis,self.lateral_dist= 0,0,0
        self.f_t =[0,0,0,0,0,0]
        self.privous_coord = [0, 0, 0]
        self.init = [0.42, 0.02, 0.51]
        self.target = [0.47, 0.0, 0.522]
        self.coord=[0.42, 0.02, 0.51]


        self.done = False
        self.target_reached = 3
        self.reward = 0
        self.epoch = 0
        self.wait = 20
        self.step_size = 0.001
        self.action_space = spaces.Discrete(17)
        self.observation_space = spaces.Box(low=-100, high=100, shape=(6,), dtype=np.float32)

    def ft_sensors_listener_filter(self):
        # print("ft_sensors_listener_filter")
        msg = rospy.wait_for_message("/ft_sensor/raw",WrenchStamped)
        rospy.loginfo("/ft_sensor/raw")
        rospy.sleep(0.2)

        force_torque=[0,0,0,0,0,0]
        for i in range(70):
            force_torque[0] += msg.wrench.force.x
            force_torque[1] += msg.wrench.force.y
            force_torque[2] += msg.wrench.force.z

            force_torque[3] += msg.wrench.torque.x
            force_torque[4] += msg.wrench.torque.y
            force_torque[5] += msg.wrench.torque.z
        self.ft = [i / 70 for i in force_torque]
        #print(self.ft)
        #print(type(self.ft))
        self.ft[1]=self.ft[1]-15
        #print("force_torque:",self.ft)
        
        if(np.isnan(self.ft[0])):
            rospy.wait_for_service('/gazebo/pause_physics')
            try:
                self.pause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/pause_physics service call failed")

        return self.ft

    def calculate_euclidean_distances(self):

        #distance between new self.coordinate and last coordinate

        self.dist_T_O = [np.linalg.norm(self.target[i]-self.privous_coord[i]) for i in range(3)]
        
        #distance between new coordinate and target
        self.dist_T_N = [np.linalg.norm(self.target[i]-self.coord[i]) for i in range(3)]
        

        #define depth axis, elvation_dis, lateral_dist
        self.lateral_dist = np.linalg.norm(self.target[self.geoAxes[0]]-self.coord[self.geoAxes[0]])
        self.depth_axis = self.coord[self.geoAxes[1]]
        self.elvation_dis = np.linalg.norm(self.target[self.geoAxes[2]]-self.coord[self.geoAxes[2]])


    def _seed(self,seed=None):
        self.np_random,seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physicsservicecallfailed")


        self.reward = 0
        self.privous_coord = self.coord.copy()
        self.move(action)
        self.calculate_euclidean_distances()

        self.f_t = self.ft_sensors_listener_filter()
        #self.f_t = [number / 10 for number in self.controller.get_ft()]
        self.check_collision()
        self.check_approching_target()
        self.check_target()


        state = [0,0,0,0,0,0]
        state =  self.f_t

        print("state",state )
        self.reward = round(self.reward, 2)



        return np.array(state, dtype=np.float32), self.reward, self.done, {}


   


    def move(self, action):


        if (action == 0):#Movingonxaxis X
            self.coord[0] += self.step_size

        elif (action == 1):#Movingonx axis Y
            self.coord[1] += self.step_size

        elif (action == 2):##Movingony axis Y
            self.coord[1] -= self.step_size
            
        elif (action == 3):##Movingonz axis Z
             self.coord[2] += self.step_size

        elif (action == 4):##Movingonz axis Z
             self.coord[2] -= self.step_size

        elif (action == 5):##Movingonz axis XY
            self.coord[0] += self.step_size
            self.coord[1] += self.step_size

        elif (action == 6):##Movingonz axis XY
            self.coord[0] += self.step_size
            self.coord[1] -= self.step_size

        elif (action == 7):##Movingonz axis XZ
            self.coord[0] += self.step_size
            self.coord[2] += self.step_size

        elif (action == 8):##Movingonz axis XZ
            self.coord[0] += self.step_size
            self.coord[2] -= self.step_size

        elif (action == 9):##Movingonz axis YZ
            self.coord[1] += self.step_size
            self.coord[2] -= self.step_size
            self.movment = [1,0,0.5]
            

        elif (action == 10):##Movingonz axis YZ
            self.coord[1] -= self.step_size
            self.coord[2] += self.step_size

        elif (action == 11):##Movingonz axis YZ
            self.coord[1] -= self.step_size
            self.coord[2] -= self.step_size

        elif action == 12 :##Movingonz axis YZ
            self.coord[1] += self.step_size
            self.coord[2] += self.step_size

        elif action == 13 :##Movingonz axis XYZ
            self.coord[0] += self.step_size
            self.coord[1] -= self.step_size
            self.coord[2] -= self.step_size

        elif action == 14 :##Movingonz axis XYZ
            self.coord[0] += self.step_size
            self.coord[1] += self.step_size
            self.coord[2] += self.step_size
            self.movment = [0.5,0.5,1]
            

        elif action == 15 :##Movingonz axis XYZ
            self.coord[0] += self.step_size
            self.coord[1] += self.step_size
            self.coord[2] -= self.step_size

        elif action >= 16 :##Movingonz axis XYZ
           self.coord[0] += self.step_size
           self.coord[1] -= self.step_size
           self.coord[2] += self.step_size


        pose_targe = geometry_msgs.msg.Pose()
        pose_targe.orientation.w = -0.5
        pose_targe.orientation.x = 0.5
        pose_targe.orientation.y = -0.5
        pose_targe.orientation.z = 0.5
        pose_targe.position.x = self.coord[0]
        pose_targe.position.y = self.coord[1]
        pose_targe.position.z = self.coord[2]

        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.1)
        self.arm_group.set_pose_target(pose_targe)
        plan1 = self.arm_group.go()



    def reset(self):

        print("*"*30)
        self.done = False
        self.reward = 0
        self.geoAxes=[1,0,2]
        self.depth_target= 0.45


        if self.target_reached   == 3:
            self.target_reached = 0
            self.target[1]=random.choice([0.3,0,-0.3])
            self.init[0] = random.uniform(0.415, 0.425)
            self.init[1] = random.uniform(self.target[1] - 0.03, self.target[1] + 0.03)
            self.init[2] = random.uniform(0.5, 0.53)

        self.arm_group.set_max_velocity_scaling_factor(1)
        self.arm_group.set_max_acceleration_scaling_factor(1)
        
        pose_targe = geometry_msgs.msg.Pose()
        pose_targe.orientation.w = -0.5
        pose_targe.orientation.x = 0.5
        pose_targe.orientation.y = -0.5
        pose_targe.orientation.z = 0.5

        pose_targe.position.x = 0.4
        pose_targe.position.y = self.coord[1]
        pose_targe.position.z = self.coord[2]
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_pose_target(pose_targe)
        plan1 = self.arm_group.go()


 

        #rospy.Time.clear()
        
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
        #resp_pause=pause.call()
            self.unpause()
        except(rospy.ServiceException) as e:
            print("/gazebo/unpause_physicsservicecallfailed")


        self.coord = self.init.copy()

        self.f_t = self.ft_sensors_listener_filter()

        #Unpausesimulationtomakeobservation


        return np.array( self.f_t, dtype=np.float32)

    def check_collision(self):
        collision = sum([True  if self.f_t[i] > 50 \
             else True if self.f_t[i] < -50 \
                 else False for i in range(6)])
        if collision > 0 :
            self.done= True
            self.reward -= 20
            print("*"*10, "COLLISION", "*"*10)

        self.reward += sum([-1  if self.f_t[i] > 50 \
             else -1 if self.f_t[i] < -50 \
                 else 1 for i in range(3)])
        
       
        self.reward += sum ([1  if self.f_t[i] < 40  and  self.f_t[i] > -40 \
            and sum(self.dist_T_N) <   0.01 and sum(self.dist_T_N) < sum(self.dist_T_O) \
            else -1 for i in range(3)])
        

    def check_target(self):
        if self.elvation_dis > 0.05  or self.lateral_dist  > 0.05:
            self.done = True
            self.reward -= 100
            print(10*"*","WENT OUT OF BORDERS",10*"*")

        if np.abs(self.depth_axis) >= np.abs(self.depth_target) and self.elvation_dis < 0.02  and self.lateral_dist  < 0.02 :
            self.done = True
            self.reward += 100
            self.target_reached += 1
            print(10*"*","TASK ACCOMPLISHED",10*"*")

    def check_approching_target(self):

        self.reward += sum([ 2 if  self.dist_T_N[i] < self.dist_T_O[i] \
            else 0 if  self.dist_T_N[i] == self.dist_T_O[i] else -2 for i in range(3)])
        
        self.reward += (-2 if np.abs(self.coord[self.geoAxes[1]]) < np.abs(self.privous_coord[self.geoAxes[1]]) and sum(self.dist_T_N) <= 0.01 \
                         else 4 if sum(self.dist_T_N) <= 0.01 else 0 )
   

   