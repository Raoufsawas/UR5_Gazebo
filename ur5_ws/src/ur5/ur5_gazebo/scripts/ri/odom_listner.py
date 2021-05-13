#!/usr/bin/env python33

import rospy

from nav_msgs.msg import Odometry


odo_x=0
odo_z=0
odo_y=0
def odometryCb(msg):
	print msg.child_frame_id
	global odo_x
	odo_x=msg.pose.pose.position.x
	global odo_y
	odo_y=msg.pose.pose.position.y
	global odo_z
	odo_z=msg.pose.pose.position.z

	print msg.pose.pose.position.x
	print msg.pose.pose.position.y
	print msg.pose.pose.position.z
	rospy.signal_shutdown(0)


    
   	




def listener():
    # start the node
    rospy.init_node('oodometry', anonymous=True) #make node 
    rospy.Subscriber('/ur5_odom',Odometry,odometryCb,queue_size=10)
    rospy.spin()




if __name__ == '__main__':
    listener()
    rospy.loginfo(odo_x)
    rospy.loginfo(odo_y)
    rospy.loginfo(odo_z)
    rospy.loginfo(odo_x)
    rospy.loginfo(odo_y)
    rospy.loginfo(odo_z)