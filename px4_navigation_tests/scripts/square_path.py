#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point, PoseStamped

def make_straight_line(set_point,sp_pub,rate):
	i=0
        set_point.x = 0
	set_point.y = 0
	set_point.z = 3

	while not rospy.is_shutdown():
	     sp_pub.publish(set_point)
	     if i < 100 and i> 0:
		set_point.x=set_point.x+0.02
		set_point.y=set_point.y+0.02

	     if i >= 100 and i < 150:
		set_point.x=set_point.x
		set_point.y=set_point.y

	     if i >= 150 and i <250:
		set_point.x=set_point.x-0.02
		set_point.y=set_point.y-0.02

	     if i >= 250 and i < 300:
		set_point.x=set_point.x
		set_point.y=set_point.y

	     if i>= 300:
		 i=0
	     print i
	     i=i+1
	     rate.sleep()

def make_square_path(set_point,sp_pub,rate):
        i=0
	set_point.x = 0
	set_point.y = 0
	set_point.z = 3

	while not rospy.is_shutdown():
	     sp_pub.publish(set_point)
	     if i >= 1 and i <= 100:
		set_point.x=set_point.x+0.02
		set_point.y=set_point.y
		print "x++"

	     if i >= 101 and i <= 200:
		set_point.x=set_point.x
		set_point.y=set_point.y
		

	     if i >= 201 and i <= 300:
		set_point.x=set_point.x
		set_point.y=set_point.y+0.02
		print "y++"

	     if i >= 301 and i <= 400:
		set_point.x=set_point.x
		set_point.y=set_point.y
	      
	     if i >= 401 and i <= 500:
		set_point.x=set_point.x-0.02
		set_point.y=set_point.y
		print "x--"

	     if i >= 501 and i <= 600:
		set_point.x=set_point.x
		set_point.y=set_point.y

	     if i>= 601 and i <= 700:
		set_point.x=set_point.x
		set_point.y=set_point.y-0.02
		print "y--"
	    
	     if i>= 701 and i <= 800:
		set_point.x=set_point.x
		set_point.y=set_point.y

	  
	     if i>= 801:
		 i=1
	     print i
	     i=i+1
	     rate.sleep()

    
def make_circular_path(set_point,sp_pub,rate):
	
	set_point.x = 0
	set_point.y = 0
	set_point.z = 3


	i=0
	#move from (0,0) to (2,0)
	while not rospy.is_shutdown() and i<201:
	     sp_pub.publish(set_point)
	     if i >= 1 and i <= 100:
		set_point.x=set_point.x+0.02
		set_point.y=set_point.y
		print "x++"

	     if i >= 101 and i <= 200:
		set_point.x=set_point.x
		set_point.y=set_point.y

	     i=i+1
	     rate.sleep()

	#move about a circle of radius r=2 
	i=0
	theta1 = 0
	theta2 = theta1+(2*math.pi/1000)
	r= 2
	while not rospy.is_shutdown():
	     
	     dx=r*(math.cos(theta1)-math.cos(theta2))
	     dy=r*(math.sin(theta1)-math.sin(theta2))
	 
	     set_point.x=set_point.x+dx
	     set_point.y=set_point.y+dy
	     sp_pub.publish(set_point)

	     theta1=theta2
	     theta2=theta1+(2*math.pi/1000)
	     i=i+1
	     if i> 1000:
		 i=0
	     print "x++"
	     print "y++"
	     rate.sleep()

def main():

	rospy.init_node('path_publishing_node')

	rate = rospy.Rate(10.0)

	sp_pub=rospy.Publisher("relative_position_setpoint", Point, queue_size=1)
        
        set_point=Point()
  
        path=int(input("Choose a path number [1 for straight path , 2 for square path  and 3 for circular path] : "))
       
        if path == 1:
           make_straight_line(set_point,sp_pub,rate)

        if path == 2:
           make_square_path(set_point,sp_pub,rate)

        if path == 3:
           make_circular_path(set_point,sp_pub,rate)

		#Just comment

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass




