#!/usr/bin/python
#
# Send a value to change the opening of the Robotiq gripper using an action
#
# Send joint values to UR5 using messages
#
import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import argparse
import actionlib
import control_msgs.msg

flag = False
def gripper_client():

    # Create an action client
    client = actionlib.SimpleActionClient(
        '/gripper_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    # Wait until the action server has been started and is listening for goals
    client.wait_for_server()

    # Create a goal to send (to the action server)
    goal = control_msgs.msg.GripperCommandGoal()
    #goal.command.position =  random.uniform(0.0,0.8)    # From 0.0 to 0.8
    goal.command.position =  0.52
    goal.command.max_effort = 0.0  # Do not limit the effort
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()    


def main():

    rospy.init_node('send_joints')
    pub = rospy.Publisher('/trajectory_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']
    

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()
        j1 = random.uniform(2.2,-2.2)		# 3.2 --> -3.2
        j2 = -1.6    	# -3.3 --1.6--> 0.15
        j3 =  random.uniform(2.2,-2.2)		# -3 --0--> 3   #1.5 = 90	
        j4 =  random.uniform(2.2,-2.2)   	# 3.2 ----> -3.2  0=90
        j5 =  random.uniform(2.2,-2.2)			# 3.2 --> -3.2
        j6 =  random.uniform(2.2,-2.2)   	# 3.2 --> -3.2
        pts.positions = [j1, j2, j3, j4, j5, j6]

        #pts.positions = [0.0, -2.33, 1.57, 0.0, 0.0, 0.0]
        pts.time_from_start = rospy.Duration(0.1)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)
        rospy.sleep(1)

        #gripper_client()





def pick_pin():
    rospy.init_node('send_joints')
    pub = rospy.Publisher('/trajectory_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()
        pts.time_from_start = rospy.Duration(0.1)
        
        pts.positions = [0.27,-2.3,2, 0.3, 1.7, 0]
        
        traj.points = []
        traj.points.append(pts)
        
        pub.publish(traj)
        
        rospy.sleep(1)
        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()
        pts.time_from_start = rospy.Duration(0.1)
        
        pts.positions = [0.27,-2.1,2, 0.3, 1.7, 0]
        
        traj.points = []
        traj.points.append(pts)
        
        pub.publish(traj)
        
        rospy.sleep(2)
        traj.header.stamp = rospy.Time.now()
        pts1 = JointTrajectoryPoint()
        pts1.time_from_start = rospy.Duration(0.1)
        
        pts1.positions = [0.27,-1.8,2, 0.3, 1.7, 0]
        
        traj.points = []
        traj.points.append(pts1)
        
        pub.publish(traj)
        rospy.sleep(1)
        
        gripper_client()
        rospy.sleep(2)
        
        main()


if __name__ == '__main__':
    try:
        pick_pin()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
#[.25,-1.8,1.9, 0.3, 1.7, 3.1]
#[.25,-1.9,1.9, 0.3, 1.7, 3.1]
#[.25,-1.9,2, 0.3, 1.6, 3.1]

#[.25,-2,2, 0.3, 1.7, 3.1]
