#!/usr/bin/env python33

import rospy

import rospy


#force_tourqe sensors
from geometry_msgs.msg._WrenchStamped import WrenchStamped
#force_tourqe sensors

t_x=0
t_y=0
t_z=0
f_x=0
f_y=0
f_z=0

tactile_sub=None

class UR5Env():
    """docstring for UR5Env"""
 
    def callback(self,msg):
        print("his")
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
        #rospy.signal_shutdown(0)
        tactile_sub.unregister()

    def ft_sensors_listener(self):
        # start the node
        rospy.init_node("listener")
        global tactile_sub
        # subscribe to the '/sr_tactile/touch/ff' topic
    #    tactile_sub = rospy.Subscriber("/ft_sensor/raw", WrenchStamped, callback ,queue_size=10)



        #tactile_sub = rospy.Subscriber("/ft_sensor/raw", WrenchStamped, callback ,queue_size=10)
        

        tactile_sub = rospy.Subscriber("/ft_sensor/raw", WrenchStamped, self.callback ,queue_size=10)
        rospy.spin()
    

tmp = UR5Env()
tmp.ft_sensors_listener()
   
