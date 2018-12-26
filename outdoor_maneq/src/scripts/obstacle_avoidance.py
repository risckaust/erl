#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion
import tf
import numpy as np

from math import *
from sensor_msgs.msg import NavSatFix, LaserScan
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *


class fcuModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set."%e

    def setReturnToHome(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.RTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. ReturnToHome Mode could not be set."%e

class Controller:

	def __init__(self):

		# Current GPS Coordinates
		self.current_lat = 0.0
		self.current_lon = 0.0
		self.current_alt = 0.0

		# GPS Fence
		self.lat_max = 22.312#22.3076#22.3176 # Location of Testing Field at KAUST
		self.lat_min = 22.310#22.3065#22.31725# Location of Testing Field at KAUST
		self.lon_max = 39.0955#39.1055#39.0983 # Location of Testing Field at KAUST
		self.lon_min = 39.0949#39.1045#39.0977 # Location of Testing Field at KAUST
		self.z_limit = 30

		#Waypoints GPS Coordiantes

		# self.home_lat = ENTER_HOME_COORDINATES
		# self.home_lon = ENTER_HOME_COORDINATES # THIS WILL BE NEEDED EVENTUALLY BECAUSE THERE WILL BE TWO TAKEOFF/LANDING LOCATIONS

		self.waypoint1_lat = 22.3118631#22.3070405#22.317490
		self.waypoint1_lon = 39.0952322#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint1_alt = 4

		self.waypoint2_lat = 22.3118631#22.3070405#22.317490
		self.waypoint2_lon = 39.0952322#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint2_alt = 4

		self.waypoint3_lat = 22.3118631#22.3070405#22.317490
		self.waypoint3_lon = 39.0952322#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint3_alt = 4

		# Current local ENU coordinates
		self.current_local_x = 0.0
		self.current_local_y = 0.0
		self.current_local_z = 0.0

		#Waypoints ENU Coordinates
		self.home_x = 0
		self.home_y = 0
		self.home_z = 0

		self.waypoint1_x = 20
		self.waypoint1_y = 0
		self.waypoint1_z = 1.5

		self.waypoint2_x = 20
		self.waypoint2_y = 0
		self.waypoint2_z = 1.5

		self.waypoint3_x = 20
		self.waypoint3_y = 0
		self.waypoint3_z = 1.5

		self.takeoff_height = 1.5

		# Instantiate setpoint topic structures
		self.positionSp	= PositionTarget()
		# set the flag to use position setpoints and yaw angle
		self.positionSp.type_mask    = int('010111111000', 2) # THIS COULD BE WRONG
		# LOCAL_NED
		self.positionSp.coordinate_frame= 1

        #defining the modes
		self.modes = fcuModes()

		# States
		self.TAKEOFF = 0
		self.WAYPOINT1 = 0
		self.WAYPOINT2 = 0
		self.WAYPOINT3 = 0
		self.GOHOME = 0
		self.LAND = 0
		self.HOVER = 0

		# AVOIDANCE STUFF
		self.front=np.zeros(20)
		self.left=np.zeros(20)
		self.right=np.zeros(20)
		self.back=np.zeros(20)

	def resetStates(self):
		self.TAKEOFF = 0
		self.WAYPOINT1 = 0
		self.WAYPOINT2 = 0
		self.WAYPOINT3 = 0
		self.GOHOME = 0
		self.LAND = 0
		self.HOVER = 0

	######### Callbacks #########
	def gpsCb(self, msg):
		if msg is not None:
			self.current_lat = msg.latitude
			self.current_lon = msg.longitude
			self.current_alt = msg.altitude

	def localPoseCb(self, msg):
		if msg is not None:
			self.current_local_x = msg.pose.position.x
			self.current_local_y = msg.pose.position.y
			self.current_local_z = msg.pose.position.z

	# Land state callback
	def landStateCb(self, msg):
		if msg is not None:
			if msg.landed_state == 1:
				self.IS_LANDED = True
			else:
				self.IS_LANDED = False

	def rangessCb(self, msg):
		if msg is not None:
			self.front = [msg.ranges[-11:-1],msg.range[0:10]] # Grabs values as [350:359, 0:10] going CCW rotation
			self.left = msg.ranges[80:100]
			self.right = msg.ranges[260:280]
			self.back = msg.ranges[170:190]

	def setWayoints_and_Fence(self):

		rospy.logwarn('Current lat')
		rospy.logwarn(self.current_lat)

		rospy.logwarn('Current lon')
		rospy.logwarn(self.current_lon)

		rospy.logwarn('Current alt')
		rospy.logwarn(self.current_alt)

		rospy.logwarn('Current x')
		rospy.logwarn(self.current_local_x)

		rospy.logwarn('Current y')
		rospy.logwarn(self.current_local_y)

		rospy.logwarn('Current z')
		rospy.logwarn(self.current_local_z)

		i = 0
		while i < 100:

			self.home_x = self.home_x + self.current_local_x
			self.home_y = self.home_y + self.current_local_y # THIS WILL NEED TO CHANGE EVENTUALLY IN FINAL SCRIPT BECAUSE
			self.home_z = self.home_z + self.current_local_z
			i = i + 1

		self.home_x = self.home_x/(i-1)
		self.home_y = self.home_y/(i-1)
		self.home_z = self.home_z/(i-1)

		rospy.logwarn('Home x')
		rospy.logwarn(self.home_x)

		rospy.logwarn('Home y')
		rospy.logwarn(self.home_y)

		rospy.logwarn('Home z')
		rospy.logwarn(self.home_z)

		#UNCOMMENT THIS CODE FOR GPS USE
		#####################################################################################################################################

		# self.waypoint1_x, self.waypoint1_y, _ = pm.geodetic2enu(self.waypoint1_lat, self.waypoint1_lon, self.waypoint1_alt, self.current_lat, self.current_lon, self.current_alt)
		# self.waypoint2_x, self.waypoint2_y, _ = pm.geodetic2enu(self.waypoint2_lat, self.waypoint2_lon, self.waypoint2_alt, self.current_lat, self.current_lon, self.current_alt)
		# self.waypoint3_x, self.waypoint3_y, _ = pm.geodetic2enu(self.waypoint3_lat, self.waypoint3_lon, self.waypoint3_alt, self.current_lat, self.current_lon, self.current_alt)
		# rospy.loginfo('Successfully set waypoints to x,y,z')

		#####################################################################################################################################

		rospy.logwarn('Waypoint x')
		rospy.logwarn(self.waypoint1_x)
		rospy.logwarn('Waypoint y')
		rospy.logwarn(self.waypoint1_y)
		rospy.logwarn('Waypoint z')
		rospy.logwarn(self.waypoint1_z)



		# States: {Idle, Takeoff, Waypoint1, Waypoint2, Waypoint3, Worker1Search, DeliverAid1, EntranceSearch, EnterBuilding, Worker2Search, DeliverAid2, FinishMapping, ExitBuilding, GoHome, Land, Hover, EmergencyLandOutside, EmergencyLandInside}
		# Possible Signals for each state:
		#   Start :                 {'Done'}
		#	Takeoff:				{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	Waypoint1:				{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	Waypoint2:				{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	Waypoint3:				{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	Worker1Search:			{'Done', 'Running', 'Failed', 'Interrupted', 'BatteryLow'}	# Failed when can't find worker in max time limit, or after comprehensive search
		#	DeliverAid1:			{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	EntranceSearch:			{'Done', 'Running', 'Failed', 'Interrupted', 'BatteryLow'}	# Failed when can't find entrance at all
		#	EnterBuilding:			{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	Worker2Search:			{'Done', 'Running', 'Failed', 'Interrupted', 'BatteryLow'}	# Failed when can't find worker in max time limit, or after comprehensive search
		#	DeliverAid2:			{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	FinishMapping:			{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	ExitBuilding:			{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	GoHome:					{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	Land:					{'Done', 'Running', 'Interrupted', 'BatteryLow'}
		#	Hover:					{'Done', 'Running'}    # state should go to Hover when interrupted, NOT when battery low

		#------------------------------------------------ Maybe Deal With These Last Two After We Know Everything Else Works ------------#
		#	EmergencyLandOutside:	{'Done', 'Running'}	   # state should go to EmergencyLandOutside when Battery Low and current state is before EnterBuilding state
		#	EmergencyLandInside:	{'Done', 'Running'}    # state should go to EmergencyLandInside when Battery Low and current state is, or is after, EnterBuilding state
	def check_and_avoid():

		minfront=min(self.front)
		minleft=min(self.left)
		minright=min(self.right)

		if (minfront < 3):
			self.positionSp.position.x = self.current_local_x   # change it when u meet abdulqader
			self.positionSp.position.y = self.current_local_y
			self.positionSp.position.z = target_alt

			setp_pub.publish(self.positionSp)

			if (minleft >= minright) and (minleft > 3):
				while (minfront < 3):

					self.positionSp.position.x = self.positionSp.position.x  # change it when u meet abdulqader
					self.positionSp.position.y = self.positionSp.position.y+.1
					self.positionSp.position.z = target_alt

					setp_pub.publish(self.positionSp)

				self.positionSp.position.x = self.current_local_x+4  # change it when u meet abdulqader
				self.positionSp.position.y = self.current_local_y
				self.positionSp.position.z = target_alt

				setp_pub.publish(self.positionSp)


				while (abs(self.positionSp.position.x - self.current_local_x) < .2 and abs(self.positionSp.position.y - self.current_local_y) < .2):
					rospy.loginfo('Heading to avoid obstacle')
					rospy.sleep()

				while (minright < 3):
					self.positionSp.position.x = self.positionSp.position.x+.1 # change it when u meet abdulqader
					self.positionSp.position.y = self.positionSp.position.y
					self.positionSp.position.z = target_alt
					setp_pub.publish(self.positionSp)

			elif (minright >= minleft) and (minright > 3):

				while (minfront < 3):

					self.positionSp.position.x = self.positionSp.position.x  # change it when u meet abdulqader
					self.positionSp.position.y = self.positionSp.position.y-.1
					self.positionSp.position.z = target_alt

					setp_pub.publish(self.positionSp)

				self.positionSp.position.x = self.positionSp.position.x+4  # change it when u meet abdulqader
				self.positionSp.position.y = self.positionSp.position.y
				self.positionSp.position.z = target_alt
				setp_pub.publish(self.positionSp)


				while (abs(self.positionSp.position.x - self.current_local_x) < .2 and abs(self.positionSp.position.y - self.current_local_y) < .2):
					rospy.loginfo('Heading to avoid obstacle')
					rospy.sleep()

				while (minleft < 3):
					self.positionSp.position.x = self.positionSp.position.x+.1 # change it when u meet abdulqader
					self.positionSp.position.y = self.positionSp.position.y
					self.positionSp.position.z = target_alt

					setp_pub.publish(self.positionSp)

def main():
	rospy.init_node('gps_setpoint_node', anonymous=True)
	rospy.logwarn("GPS setpoints node is started")

	K = Controller()

	########## Subscribers ##########

	# Subscriber: GPS
	rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, K.gpsCb)

	# Subscriber: Local pose
	rospy.Subscriber("mavros/local_position/pose", PoseStamped, K.localPoseCb)

	# Get landing state
	rospy.Subscriber("mavros/extended_state", ExtendedState, K.landStateCb)

	# FOR RPLIDAR
	rospy.Subscriber("scan", LaserScan, K.rangessCb)

	########## Publishers ##########

	# Publisher: PositionTarget
	avoid_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
	rate = rospy.Rate(10.0)

	# Do initial checks
	while (K.current_lat*K.current_lon*K.current_alt) == 0:

		rospy.loginfo('Waiting for current gps location to execute setWaypoints_and_Fence') # Initializes waypoints and fence in local x,y,z and checks to see if they make sense
		rate.sleep()

	K.setWayoints_and_Fence()

	K.resetStates()

	# We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
	k=0
	while k<10:
		avoid_pub.publish(K.positionSp)
		rate.sleep()
		k = k+1

	# K.modes.setOffboardMode()
	# K.modes.setArm()

	###################### TAKEOFF STUFF ####################
	K.TAKEOFF = 1

	#########################################################

	while not rospy.is_shutdown():

		if K.TAKEOFF:
			rospy.logwarn('Vehicle is taking off')
			K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
			K.positionSp.position.x = K.home_x
			K.positionSp.position.y = K.home_y
			K.positionSp.position.z = K.takeoff_height # Should be 1

			rospy.loginfo('Home X Position')
			rospy.loginfo(K.positionSp.position.x)
			rospy.loginfo('Home Y Position')
			rospy.loginfo(K.positionSp.position.y)
			rospy.loginfo('Takeoff Height')
			rospy.loginfo(K.positionSp.position.z)

			if abs(K.current_local_z - K.takeoff_height) < .1:
				rospy.logwarn("Reached Takeoff Height")
				K.resetStates()
				K.WAYPOINT1 = 1

		if K.WAYPOINT1:
			rospy.loginfo('Vehicle heading to waypoint 1')
			K.check_and_avoid()
			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.position.x = K.waypoint1_x
			K.positionSp.position.y = K.waypoint1_y
			K.positionSp.position.z = K.waypoint1_z

			tempz1 = K.waypoint1_z
			rospy.loginfo('Waypoint1_z')
			rospy.loginfo(tempz1)

			tempx = K.current_local_x - K.waypoint1_x
			rospy.loginfo('Current_x - Waypoint1_x')
			rospy.loginfo(tempx)

			tempy = K.current_local_y - K.waypoint1_y
			rospy.loginfo('Current_y - Waypoint1_y')
			rospy.loginfo(tempy)

			tempz = K.current_local_z - K.waypoint1_z
			rospy.loginfo('Current_z - Waypoint1_z')
			rospy.loginfo(tempz)

			if abs(K.current_local_x - K.waypoint1_x)<= 1 and abs(K.current_local_y - K.waypoint1_y) <= 1 and abs(K.current_local_z - K.waypoint1_z) <= 0.5:   # Rules give a 3m radius from goal
				rospy.loginfo("Current position close enough to desired waypoint")
				rospy.loginfo("Reached waypoint 1")
				K.resetStates()
				K.WAYPOINT2 = 1

		if K.WAYPOINT2:
			rospy.loginfo('Vehicle heading to waypoint 2')

			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.position.x = K.waypoint2_x
			K.positionSp.position.y = K.waypoint2_y
			K.positionSp.position.z = K.waypoint2_z

			if abs(K.current_local_x - K.waypoint2_x)<= 1 and abs(K.current_local_y - K.waypoint2_y) <= 1 and abs(K.current_local_z - K.waypoint2_z) <= 0.5:   # Rules give a 3m radius from goal
				rospy.loginfo("Current position close enough to desired waypoint")
				rospy.loginfo("Reached waypoint 2")
				K.resetStates()
				K.WAYPOINT3 = 1

		if K.WAYPOINT3:
			rospy.loginfo('Vehicle heading to waypoint 3')

			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.position.x = K.waypoint3_x
			K.positionSp.position.y = K.waypoint3_y
			K.positionSp.position.z = K.waypoint3_z

			if abs(K.current_local_x - K.waypoint3_x)<= 1 and abs(K.current_local_y - K.waypoint3_y) <= 1 and abs(K.current_local_z - K.waypoint3_z) <= 0.5:   # Rules give a 3m radius from goal
				rospy.loginfo('Current position close enough to desired waypoint')
				rospy.loginfo('Reached waypoint 3')
				K.resetStates()
				K.GOHOME = 1

		if K.GOHOME:

			rospy.loginfo('Mission Complete - Vehicle heading back to home')
			K.modes.setReturnToHome()
			# K.positionSp.header.frame_id = 'local_origin'
			# K.positionSp.pose.position.x = K.home_x
			# K.positionSp.pose.position.y = K.home_y
			# K.positionSp.pose.position.z = K.home_z + 2 # I GUESS 2 IS A GOOD ALTITUDE TO RETURN

			# if abs(K.current_local_x - K.home_x)<= 1 and abs(K.current_local_y - K.home_y) <= 1:   # Rules give a 3m radius from goal
			# 	rospy.loginfo('Reached home')
			# 	K.resetStates()
			# 	K.LAND = 1

		# if K.LAND:
		# 	rospy.loginfo('Vehicle is landing')
		# 	K.modes.setAutoLandMode()
		# 	if K.IS_LANDED:
		# 		K.modes.setDisarm()
		# 		K.resetStates()

		if K.HOVER:
			rospy.logwarn('Vehicle in Hover mode until something else happens')
			# NEED TO FIGURE OUT HOW TO EXIT THIS STATE

		avoid_pub.publish(K.positionSp)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
