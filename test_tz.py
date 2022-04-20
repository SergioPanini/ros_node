#!/usr/bin/env python3.8

import random
from setuptools import Command
import rospy
import datetime
import time
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point

from custom_messages import ComandMessage


def publishMethod():
    #pub = rospy.Publisher('talker', Float64, queue_size=10) # defining the publisher by topic, message type
    #pub = rospy.Publisher('talker', String, queue_size=10) # defining the publisher by topic, message type
    pub = rospy.Publisher('coordinates', Point, queue_size=10) # defining the publisher by topic, message type
    pub2 = rospy.Publisher('command', ComandMessage, queue_size=10) # defining the publisher by topic, message type
    
    rospy.init_node('publish_node', anonymous=True) # defining the ros node - publish node 
    rate = rospy.Rate(10) # 10hz # fequency at which the publishing occurs
    while not rospy.is_shutdown():
        
        
        #print(msg)
        #publish_msg = str(datetime.datetime.now())
        #pub.publish("Currenet time: " + publish_msg) # publishing 
        #pub.publish(point) # publishing 


        count_points = random.randint(1, 9)

        msg = ComandMessage(START='START', END='END', command_type='START', name='', step=0)
        rospy.loginfo("Send message: START") 

        pub2.publish(msg) 
        
        for i in range(count_points):
        
            x, y, z = (random.randint(0, 5), random.randint(0, 5), random.randint(0, 5))
            point = Point(x, y, z)
            pub.publish(point)
            rospy.loginfo("Send point: x={}, y={},z={}".format(x, y, z))
            time.sleep(random.randint(1, 2))
        

        msg2 = ComandMessage(START='START', END='END', command_type='END', name='my trace', step=1)
        rospy.loginfo("Send message: END")
        pub2.publish(msg2)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publishMethod()
    except rospy.ROSInterruptException: 
        pass