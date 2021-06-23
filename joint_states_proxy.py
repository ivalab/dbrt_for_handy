#!/usr/bin/env python
 
import rospy, actionlib, math, numpy, tf
 
from std_msgs.msg import Float64
       
       
def jointStateCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    rospy.init_node('jointStateListener', anonymous=True)
    
    jointStateTopic = '/joint_states'
    rospy.Subscriber(jointStateTopic, String, jointStateCallback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__='__main__':
    listener()