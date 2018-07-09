#!/usr/bin/env python

import rospy # ROS interface

from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

class Controller:

	def __init__(self):
                
                # current distances measured by the opticalflow sensor
		self.current_integrated_x                       = 0.0        #positive x is at the left of the quadcopter
                self.current_integrated_y                       = 0.0        #positive y is at the front of the quadcopter

		# Current local ENU coordinates
		self.current_local_x				= 0.0
		self.current_local_y				= 0.0
		self.current_local_z				= 0.0

                # Altitude setpoint, [meters]
                self.ALT_SP        = 1.0

		# Instantiate a setpoint topic structure
		self.mavros_sp					= PositionTarget()
		# use position setpoints
		self.mavros_sp.type_mask		= int('101111111000', 2)
		# FRAME_LOCAL_NED
		self.mavros_sp.coordinate_frame = 1

		######### Callbacks #########

	def opticalFlowRateCb(self, msg):
		if msg is not None:
			self.current_integrated_x = self.current_integrated_x + self.current_local_z* msg.integrated_x
			self.current_integrated_y = self.current_integrated_y + self.current_local_z*msg.integrated_y
		#	print "dx = ",self.current_integrated_x
                #        print "dy = ",self.current_integrated_y
                       
	def localPoseCb(self, msg):
		if msg is not None:
			self.current_local_x = msg.pose.position.x
			self.current_local_y = msg.pose.position.y
			
                        

        def localAltitudeCb(self,msg):
               if msg is not None:
                       self.current_local_z = msg.local

	def positionSpCb(self, msg):
		# lat = msg.x; lon = msg.y;
		# msg.z is the altitude setpoint in ENU
		if msg is not None:
			target_x = msg.x
			target_y = msg.y
			#target_alt = msg.z # in ENU

			delta_x = target_x - self.current_integrated_x
                        delta_y = target_y - self.current_integrated_y
			
			self.mavros_sp.position.x = self.current_local_x + delta_x
			self.mavros_sp.position.y = self.current_local_y + delta_y
			self.mavros_sp.position.z = self.ALT_SP
                        print self.current_local_x
                        print self.current_local_y


def main():
	rospy.init_node('opticalflow_setpoint_node', anonymous=True)

	rospy.logwarn("optical flow  setpoints node is started")

	K = Controller()

	########## Subscribers ##########

	# Subscriber: Optical Flow rate
	rospy.Subscriber("mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, K.opticalFlowRateCb)
	# Subscriber: Local pose
	rospy.Subscriber("mavros/local_position/pose", PoseStamped, K.localPoseCb)
        # Subscriber: altitude topic
        rospy.Subscriber("mavros/altitude",Altitude,K.localAltitudeCb)
	# User position setpoint
	rospy.Subscriber("position_setpoint", Point, K.positionSpCb)

	########## Publishers ##########

	# Publisher: PositionTarget
	setp_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=1)

	rate = rospy.Rate(10.0)

	i = 0
	while not rospy.is_shutdown() and i  < 10:
		# Initialize setpoint
		K.mavros_sp.position.x = K.current_local_x
		K.mavros_sp.position.y = K.current_local_y
		K.mavros_sp.position.z = K.ALT_SP
		i = i +1
		rate.sleep()

	while not rospy.is_shutdown():

		setp_pub.publish(K.mavros_sp)

		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
