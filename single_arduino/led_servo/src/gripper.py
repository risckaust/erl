#! /usr/bin/env python

#Modules import
import rospy
import numpy as np
import tf
import datetime

from math import *
from std_msgs.msg import *


def talker():
    rospy.init_node('servo',anonymous=True)
    servoPub = rospy.Publisher('/servo', UInt16, queue_size=10)
    servoSub = rospy.Subscriber('/servo',UInt16, callback)
    print "I am here"
    n = UInt16()
    HOLD = UInt16()
    RELEASE = UInt16()
    print n, HOLD, RELEASE
    HOLD.data = 131
    RELEASE.data = 80
    
    flag = True
    while not rospy.is_shutdown():
        if(flag):
            servoPub.publish(RELEASE)
            rospy.sleep(2)
            for n.data in range(RELEASE.data,HOLD.data):
                if(rospy.is_shutdown()):
                    continue
                servoPub.publish(n)
                rospy.sleep(0.1)
        flag = False
#==============================================================================
#         rospy.spin()
#==============================================================================

def callback(msg):
    if msg.data is not None:
        print "there are messages"
        print msg.data
    else:
        print "no messages"


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

