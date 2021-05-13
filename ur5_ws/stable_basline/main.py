

import gym
import time
from std_srvs.srv import Empty
import numpy
import random
import time
import qlearn
from gym import wrappers

# ROS packages required
import rospy
import rospkg

# import our training environment
import UR5_env


import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import argparse
import actionlib
import control_msgs.msg

import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time

import qlearn

import pickle

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

from gym.utils import seeding
from stable_baselines import A2C, ACKTR, SAC
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.cmd_util import make_vec_env



if __name__ == '__main__':

    env = gym.make('UR5env-v0')
    print ("start tra")

    outdir = '/tmp/gazebo_gym_UR5'
    #env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo ( "Monitor Wrapper started")


    last_time_steps = numpy.ndarray(0)
    #load model
    #with open('/home/ros/test/ur5_ws/ur5.pkl', 'rb') as input:


    # wrap it
    #env = make_vec_env(lambda: env, n_envs=4)
    # Train the agent
    model = ACKTR(MlpPolicy, env, verbose=100,  tensorboard_log="./logs/")
    #model.learn(steps_per_epoch=5000, epochs=100)#
    model.learn(total_timesteps=1000000)
    env.close()
