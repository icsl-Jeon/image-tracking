#!/usr/bin/env python
import getch
import rospy
from std_msgs.msg import String #String message 
from std_msgs.msg import Int8


################################
# created by yuvaram
#yuvaramsingh94@gmail.com
################################


def keys():
    pub = rospy.Publisher('key',Int8,queue_size=10) # "key" is the publisher name
    rospy.init_node('keypress')
    rate = rospy.Rate(30)#try removing this line ans see what happens
    rospy.loginfo("w=forward a=left d=right s=backward\nq=left turn e=right turn z=elevation c=falling")
    while not rospy.is_shutdown():
		k=ord(getch.getch())
		rospy.loginfo(str(k))
		pub.publish(k)
		
		rate.sleep()
		       
	   #s=115,e=101,g=103,b=98

if __name__=='__main__':
    
	rospy.loginfo("keypress node started: enter keyboard in this window\n")	
	keys()
