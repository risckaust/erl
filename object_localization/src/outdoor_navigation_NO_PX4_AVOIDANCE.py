#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion
import tf

from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped, Quaternion
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
    def setVelocity(self,velocity):
    	rospy.wait_for_service('param/set')
    	try:
    		velocitySetService = rospy.ServiceProxy('param/set', mavros_msgs.srv.ParamSet)
    		velocitySetService(velocity)
    		rospy.logwarn('Just tried to set velocity parameter')
    	except rospy.ServiceException, e:
    	 	rospy.logwarn('FAILED to set velocity parameter')
    	 	print "service ParamSet call failed: %s. Velocity could not be set."%e
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

		# Outside Worker Search Variables in GPS

		self.building_center_lat = 22.317575 # Roughly the center of the KAUST field
		self.building_center_lon = 39.0984	# Roughly the center of the KAUST field

		# Current local ENU coordinates
		self.current_local_x = 0.0
		self.current_local_y = 0.0
		self.current_local_z = 0.0

		# Current angles
		self.current_yaw = 0


		#Waypoints ENU Coordinates
		self.home_x = 0
		self.home_y = 0
		self.home_z = 0

		self.waypoint1_x = 0
		self.waypoint1_y = 0
		self.waypoint1_z = 1.5

		self.waypoint2_x = 0
		self.waypoint2_y = 0
		self.waypoint2_z = 1.5

		self.waypoint3_x = 0
		self.waypoint3_y = 0
		self.waypoint3_z = 1.5
		
		self.x_FenceLimit_max = 30 # 300 meters downfield from the starting position
		self.x_FenceLimit_min = -5 # 5 meters behind the starting the position 
		self.y_FenceLimit_max = 15 # 15 meters to the left of the starting position
		self.y_FenceLimit_min = -15 # 15 meters to the right of the starting position

		self.x_fence_max_warn = 29 # 295 meters downfield from the starting position
		self.x_fence_min_warn = -4 # 5 meters in back of starting position
		self.y_fence_max_warn = 14 # 14 meters to the left of starting position
		self.y_fence_min_warn = -14 # 14 meters to the right of the starting position
		self.z_limit_warn = 35 # Maximum height above ground that drone is allowed to go (MAX height is 40 meters - TO BE VERIFIED BY COMPETITION ORGANIZERS)

		self.building_center_x = 5#10
		self.building_center_y = 0

		# SEARCH WAYPOINTS ARE TO BE SET BASED ON LOCATION OF BUILDING. WORKER IS PRESUMED TO BE FURTHER DOWNFIELD THAN BUILDING
		self.worker1_search_FAR_x = 0 
		self.worker1_search_NEAR_x = 0 
		self.worker1_search_LEFT_y = 0
		self.worker1_search_RIGHT_y = 0
		self.worker1_search_z = 8

		self.worker1_found_flag = 0

		self.worker1_search_WP1_FLAG = 0
		self.worker1_search_WP2_FLAG = 0
		self.worker1_search_WP3_FLAG = 0
		self.worker1_search_WP4_FLAG = 0

		self.takeoff_height = 1.5

		self.counterCb = 0
		self.verifyPOI_flag = 0

		# Instantiate setpoint topic structures
		self.positionSp	= PoseStamped()
		self.worker1Sp = PoseStamped()
		# self.maxvelocity = ParamSet()

		#defining the modes
		self.modes = fcuModes()

		# States
		self.TAKEOFF = 0
		self.WAYPOINT1 = 0
		self.WAYPOINT2 = 0
		self.WAYPOINT3 = 0
		self.WORKER1SEARCH = 0
		self.DELIVERAID1 = 0
		self.ENTRANCESEARCH = 0
		self.ENTERBUILDING = 0
		self.WORKER2SEARCH = 0
		self.DELIVERAID2 = 0
		self.FINISHMAPPING = 0
		self.EXITBUILDING = 0
		self.GOHOME = 0
		self.LAND = 0
		self.HOVER = 0

		######### GRIPPER #############
		self.n = UInt16()
		self.HOLD = UInt16()
		self.RELEASE = UInt16()
		self.HOLD.data = 131
		self.RELEASE.data = 80
		self.gripper_flag = True

	def resetStates(self):
		self.TAKEOFF = 0
		self.WAYPOINT1 = 0
		self.WAYPOINT2 = 0
		self.WAYPOINT3 = 0
		self.WORKER1SEARCH = 0
		self.DELIVERAID1 = 0
		self.ENTRANCESEARCH = 0
		self.ENTERBUILDING = 0
		self.WORKER2SEARCH = 0
		self.DELIVERAID2 = 0
		self.FINISHMAPPING = 0
		self.EXITBUILDING = 0
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

			(_, _, self.current_yaw) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])


	def objectPoseCb(self, msg):
		if msg is not None and self.WORKER1SEARCH:
			rospy.logwarn('Item of interest found')             
			self.worker1Sp = msg
			self.worker1Sp.header.frame_id='local_origin'
			self.verifyPOI_flag = 1
			self.counterCb = self.counterCb + 1 # This counter is used to tell verifyPOI() that the message has been updated, meaning that it is still seeing something
			self.worker1Sp.pose.position.z = self.worker1_search_z
		if msg is not None and self.worker1_found_flag:
			self.worker1Sp = msg
			self.worker1Sp.header.frame_id='local_origin'
			self.worker1Sp.pose.position.z = self.worker1_search_z
						
	# Land state callback
	def landStateCb(self, msg):
		if msg is not None:
			if msg.landed_state == 1:
				self.IS_LANDED = True
			else:
				self.IS_LANDED = False

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

		#UNCOMMENT THIS CODE FOR GPS USE
		#####################################################################################################################################
		
		# self.x_FenceLimit_max, self.y_FenceLimit_max, _ = pm.geodetic2enu(self.lat_max, self.lon_max, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# self.x_FenceLimit_min, self.y_FenceLimit_min, _ = pm.geodetic2enu(self.lat_min, self.lon_min, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# rospy.loginfo('Successfully set fence limit maxes and mins')

		# self.waypoint1_x, self.waypoint1_y, _ = pm.geodetic2enu(self.waypoint1_lat, self.waypoint1_lon, self.waypoint1_alt, self.current_lat, self.current_lon, self.current_alt)
		# self.waypoint2_x, self.waypoint2_y, _ = pm.geodetic2enu(self.waypoint2_lat, self.waypoint2_lon, self.waypoint2_alt, self.current_lat, self.current_lon, self.current_alt)
		# self.waypoint3_x, self.waypoint3_y, _ = pm.geodetic2enu(self.waypoint3_lat, self.waypoint3_lon, self.waypoint3_alt, self.current_lat, self.current_lon, self.current_alt)
		# rospy.loginfo('Successfully set waypoints to x,y,z')

		# self.x_fence_max_warn = self.x_FenceLimit_max - 1
		# self.x_fence_min_warn = self.x_FenceLimit_min + 1
		# self.y_fence_max_warn = self.y_FenceLimit_max - 1
		# self.y_fence_min_warn = self.y_FenceLimit_min + 1
		# self.z_limit_warn = self.z_limit - .5

		# self.building_center_x, self.building_center_y, _ = pm.geodetic2enu(self.building_center_lat, self.building_center_lon, self.z_limit, self.current_lat, self.current_lon, self.current_alt)

		#####################################################################################################################################

		rospy.logwarn('x Fence Max Warn')
		rospy.logwarn(self.x_fence_max_warn)
		rospy.logwarn('x Fence Min Warn')
		rospy.logwarn(self.x_fence_min_warn)
		rospy.logwarn('y Fence Max Warn')
		rospy.logwarn(self.y_fence_max_warn)
		rospy.logwarn('y Fence Min Warn')
		rospy.logwarn(self.y_fence_min_warn)
		rospy.logwarn('z Height Limit Warn')
		rospy.logwarn(self.z_limit_warn)

		rospy.logwarn('Waypoint x')
		rospy.logwarn(self.waypoint1_x)
		rospy.logwarn('Waypoint y')
		rospy.logwarn(self.waypoint1_y)
		rospy.logwarn('Waypoint z')
		rospy.logwarn(self.waypoint1_z)

		rospy.loginfo('Trying to test validity of waypoints')
		self.isValidWaypoint(self.waypoint1_x,self.waypoint1_y,self.waypoint1_z) # Test whether waypoint 1 is within fence
		self.isValidWaypoint(self.waypoint2_x,self.waypoint2_y,self.waypoint2_z) # Test whether waypoint 2 is within fence
		self.isValidWaypoint(self.waypoint3_x,self.waypoint3_y,self.waypoint3_z) # Test whether waypoint 3 is within fence

	def verifyPOI(self):
		if self.verifyPOI_flag:
			counter = 0
			for i in range(500):
				previous_counter = self.counterCb # self.counterCb is constantly getting updated every time objectPoseCb is called
				rospy.sleep(.01) # Give self.counterCb chance to update if objectPoseCb is called again
				if self.counterCb is not previous_counter:
					counter = counter + 1
			if counter >= 30:
				self.worker1_found_flag = 1
				rospy.loginfo("Outside worker found!")
			elif counter < 30:
				rospy.loginfo("Detected false positive. Continuing search.")
		
		self.verifyPOI_flag = 0
		self.counterCb = 0

	def Worker1SearchPattern(self):

		# Assuming center of building is in center of width of field, search algorithm is to 
		# go down field on right side, turn left, and come back on left side to cover 
		# the entire field, downfield of the building from where the control station is. 
		# Field is 30 meters wide, so in order to cover as much of the field as possible,
		# the drone should fly within 7.5 meters of the edge on the way down and on the way back

		#!!!!!!!!!!!!!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		# Drone should go to altitude that is definitely above buliding after completing waypoint 3
		#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		self.worker1_search_LEFT_y = self.y_FenceLimit_max - 7.5 # 7.5 meters from the left edge of the field
		self.worker1_search_RIGHT_y = self.y_FenceLimit_min + 7.5 # 7.5 meters from the right edge of the field

		self.worker1_search_NEAR_x = self.building_center_x # DRONE WILL START AT SAME DISTANCE DOWNFIELD AS CENTER OF BUILDING
		self.worker1_search_FAR_x = self.x_FenceLimit_max - 7.5 # STOP 7.5 METERS SHORT OF EDGE OF FIELD

		if self.worker1_search_WP1_FLAG == 0:

			self.positionSp.header.frame_id = 'local_origin'
			desired_yaw = atan2((self.worker1_search_RIGHT_y-self.current_local_y),(self.worker1_search_NEAR_x-self.current_local_x))
			quaternion_yaw = quaternion_from_euler(0, 0, desired_yaw)
			self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

			if abs(self.current_yaw-desired_yaw)<.1:

				self.positionSp.pose.position.x = self.worker1_search_NEAR_x
				self.positionSp.pose.position.y = self.worker1_search_RIGHT_y
				self.positionSp.pose.position.z = self.worker1_search_z

				if abs(self.current_local_x - self.worker1_search_NEAR_x)<= 1 and abs(self.current_local_y - self.worker1_search_RIGHT_y) <= 1 and abs(self.current_local_z - self.worker1_search_z) <= 0.5:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached worker search waypoint 1")
					self.worker1_search_WP1_FLAG = 1
				
		elif self.worker1_search_WP2_FLAG == 0:

			self.positionSp.header.frame_id = 'local_origin'
			desired_yaw = 0
			quaternion_yaw = quaternion_from_euler(0, 0, desired_yaw)
			self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

			if abs(self.current_yaw-desired_yaw)<.1:

				self.positionSp.pose.position.x = self.worker1_search_FAR_x
				self.positionSp.pose.position.y = self.worker1_search_RIGHT_y
				self.positionSp.pose.position.z = self.worker1_search_z

				if abs(self.current_local_x - self.worker1_search_FAR_x)<= 1 and abs(self.current_local_y - self.worker1_search_RIGHT_y) <= 1 and abs(self.current_local_z - self.worker1_search_z) <= 0.5:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached worker search waypoint 2")
					self.worker1_search_WP2_FLAG = 1

		elif self.worker1_search_WP3_FLAG == 0:

			self.positionSp.header.frame_id = 'local_origin'
			desired_yaw = pi/2
			quaternion_yaw = quaternion_from_euler(0, 0, desired_yaw)
			self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

			if abs(self.current_yaw-desired_yaw)<.1:
				self.positionSp.pose.position.x = self.worker1_search_FAR_x
				self.positionSp.pose.position.y = self.worker1_search_LEFT_y
				self.positionSp.pose.position.z = self.worker1_search_z
				if abs(self.current_local_x - self.worker1_search_FAR_x)<= 1 and abs(self.current_local_y - self.worker1_search_LEFT_y) <= 1 and abs(self.current_local_z - self.worker1_search_z) <= 0.5:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached worker search waypoint 3")
					self.worker1_search_WP3_FLAG = 1

		elif self.worker1_search_WP4_FLAG == 0:

			self.positionSp.header.frame_id = 'local_origin'
			desired_yaw = pi
			quaternion_yaw = quaternion_from_euler(0, 0, desired_yaw)
			self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

			if abs(self.current_yaw-desired_yaw)<.1:
				self.positionSp.pose.position.x = self.worker1_search_NEAR_x
				self.positionSp.pose.position.y = self.worker1_search_LEFT_y			
				self.positionSp.pose.position.z = self.worker1_search_z
				if abs(self.current_local_x - self.worker1_search_NEAR_x)<= 1 and abs(self.current_local_y - self.worker1_search_LEFT_y) <= 1 and abs(self.current_local_z - self.worker1_search_z) <= 0.5:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached worker search waypoint 4")
					self.worker1_search_WP4_FLAG = 1

		else:
			rospy.logwarn(':[ COULD NOT FIND MISSING WOKRER IN FIELD! TRYING AGAIN')
			self.worker1_search_WP1_FLAG = 0
			self.worker1_search_WP2_FLAG = 0
			self.worker1_search_WP3_FLAG = 0
			self.worker1_search_WP4_FLAG = 0

	def isTooCloseToFence(self):

		value = False
		
		if((self.current_local_x > self.x_fence_max_warn) or (self.current_local_y > self.y_fence_max_warn) or (self.current_local_z > self.z_limit_warn)):
			rospy.logwarn('Vehicle is too close to fence!')
			value = True
		elif((self.current_local_x < self.x_fence_min_warn) or (self.current_local_y < self.y_fence_min_warn)):
			rospy.logwarn('Vehicle is too close to fence!')
			value = True
		return value

	def isValidWaypoint(self,waypoint_x,waypoint_y,waypoint_z):
		if((waypoint_x > self.x_fence_max_warn) or (waypoint_y > self.y_fence_max_warn) or (waypoint_z > self.z_limit_warn)):
			rospy.logwarn('The given waypoint is either too close to or beyond the boundary! Please restart with different waypoint')
			rospy.logwarn('Rospy is shutting down')
			rospy.is_shutdown() == True
			rospy.signal_shutdown('Target waypoints out of bounds.')
		elif((waypoint_x < self.x_fence_min_warn) or (waypoint_y < self.y_fence_min_warn)):
			rospy.logwarn('The given waypoint is either too close to or beyond the boundary! Please restart with different waypoint')
			rospy.logwarn('Rospy is shutting down')
			rospy.is_shutdown() == True
			rospy.signal_shutdown('Target waypoints out of bounds.')
		rospy.loginfo('Successfully tested validity of waypoint')

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

	# Subscriber: object_localization setpoints
	rospy.Subscriber("/detected_object_3d_pos", PoseStamped, K.objectPoseCb)

	########## Publishers ##########

	# Publisher: PositionTarget
	avoid_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
	rate = rospy.Rate(10.0)

	servo_pub = rospy.Publisher("/servo", UInt16, queue_size=10)

	# Do initial checks
	while (K.current_lat*K.current_lon*K.current_alt) == 0 and not rospy.is_shutdown():

		rospy.loginfo('Waiting for current gps location to execute setWaypoints_and_FenceCb') # Initializes waypoints and fence in local x,y,z and checks to see if they make sense
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

		#GRADUAL CLOSURE OF GRIPPER
		if(K.gripper_flag):
			rospy.loginfo('Gripper closing')
			servo_pub.publish(K.RELEASE)
			rospy.sleep(2)
			for K.n.data in range(K.RELEASE.data,K.HOLD.data):
				if(rospy.is_shutdown()):
					continue
				servo_pub.publish(K.n)
				rospy.sleep(0.1)
		K.gripper_flag = False

		############ Check if vehicle is currently in warning area first #############
		if (K.isTooCloseToFence()== True):
				rospy.logwarn('Changing to Hover mode because vehicle was too close to boundary')
				K.resetStates()
				K.HOVER = 1
				# PUT HOVER POINTS HERE SO THAT THEY DON'T CONSTANTLY GET UPDATED AND THE VEHICLE DRIFTS
				K.positionSp.pose.position.x =  K.current_local_x 
				K.positionSp.pose.position.y =  K.current_local_y 
				K.positionSp.pose.position.z =  K.current_local_z
		#######################################################

		if K.TAKEOFF:
			rospy.logwarn('Vehicle is taking off')
			K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
			K.positionSp.pose.position.x = K.home_x 
			K.positionSp.pose.position.y = K.home_y
			K.positionSp.pose.position.z = K.takeoff_height # Should be 1

			rospy.loginfo('Home X Position')
			rospy.loginfo(K.positionSp.pose.position.x)
			rospy.loginfo('Home Y Position')
			rospy.loginfo(K.positionSp.pose.position.y)
			rospy.loginfo('Takeoff Height')
			rospy.loginfo(K.positionSp.pose.position.z)

			K.positionSp.pose.orientation.w = 1.0	# IS THIS NEEDED?
			if abs(K.current_local_z - K.takeoff_height) < .2:
				rospy.logwarn("Reached Takeoff Height")	
				K.resetStates()
				K.WAYPOINT1 = 1

		if K.WAYPOINT1:
			rospy.loginfo('Vehicle heading to waypoint 1')

			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.waypoint1_x
			K.positionSp.pose.position.y = K.waypoint1_y
			K.positionSp.pose.position.z = K.waypoint1_z

			# K.maxvelocity.param_id = 'MPC_XY_VEL_MAX'
			# K.maxvelocity.value = 2
			# rospy.logwarn('Am about to try to set velocity')
			# K.modes.setVelocity(K.maxvelocity)
			# rospy.logwarn('Just tried to set velocity parameter')

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
			K.positionSp.pose.position.x = K.waypoint2_x
			K.positionSp.pose.position.y = K.waypoint2_y
			K.positionSp.pose.position.z = K.waypoint2_z

			if abs(K.current_local_x - K.waypoint2_x)<= 1 and abs(K.current_local_y - K.waypoint2_y) <= 1 and abs(K.current_local_z - K.waypoint2_z) <= 0.5:   # Rules give a 3m radius from goal
				rospy.loginfo("Current position close enough to desired waypoint")
				rospy.loginfo("Reached waypoint 2")
				K.resetStates()
				K.WAYPOINT3 = 1

		if K.WAYPOINT3:
			rospy.loginfo('Vehicle heading to waypoint 3')

			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.waypoint3_x
			K.positionSp.pose.position.y = K.waypoint3_y
			K.positionSp.pose.position.z = K.waypoint3_z

			if abs(K.current_local_x - K.waypoint3_x)<= 1 and abs(K.current_local_y - K.waypoint3_y) <= 1 and abs(K.current_local_z - K.waypoint3_z) <= 0.5:   # Rules give a 3m radius from goal
				rospy.loginfo('Current position close enough to desired waypoint')
				rospy.loginfo('Reached waypoint 3')
				K.resetStates()
				K.WORKER1SEARCH = 1

		if K.WORKER1SEARCH:
			rospy.loginfo('Vehicle searching for missing worker outside') 
			K.verifyPOI()
			if K.worker1_found_flag:
				#rospy.loginfo('Outside worker found!')
				#rospy.loginfo('Setting worker position to go to')

				K.positionSp.header.frame_id = 'local_origin'
				K.positionSp.pose.position.x = K.worker1Sp.pose.position.x
				K.positionSp.pose.position.y = K.worker1Sp.pose.position.y
				K.positionSp.pose.position.z = K.worker1_search_z
				K.resetStates()
				K.DELIVERAID1 = 1

			else:
				# EXECUTE WORKER SEARCH WAYPOITNS. IF CAN'T FIND ANYTHING, DO IT OVER AGAIN
				K.Worker1SearchPattern()

		if K.DELIVERAID1:
			rospy.loginfo('Trying to reach outside worker')
			if (abs(K.current_local_x-K.positionSp.pose.position.x)<.1 and abs(K.current_local_y-K.positionSp.pose.position.y)<.1):
				servo_pub.publish(K.RELEASE)
				rospy.sleep(1)
				rospy.loginfo("Aid Dropped")
				K.resetStates()
				#K.ENTRANCESEARCH = 1
				K.GOHOME = 1

		if K.ENTRANCESEARCH:
			rospy.loginfo('Drone searching for entrance')
			# if K.entrance_found_flag:
			# 	rospy.logwarn('Unblocked entrance successfully found!')
			# 	K.resetStates()
			# 	K.ENTERBUILDING = 1

		if K.ENTERBUILDING:
			rospy.loginfo('Drone entering building')
			#Execute Enter Building algorithm
			#if Enter Building is success
				#rospy.logwarn('Vehicle successfully')
				#K.resetStates()
				#K.WORKER2SEARCH = 1

		if K.WORKER2SEARCH:
			rospy.loginfo('Vehicle searching for missing worker inside') 
			#Execute OUTSIDE WORKER SEARCH ALGORITHM
			#if WORKER2FOUND
				#rospy.logwarn('Inside worker found!')
				#K.resetStates()
				#K.DELIVERAID2 = 1

		if K.DELIVERAID2:
			rospy.loginfo('Delivering the first aid kit to inside worker')
			#Execute DELIVER AID 2
			#if Deliver Aid 2 is success
				#rospy.logwarn('Aid successfully delivered')
				#K.resetStates()
				#K.FINSIHMAPPING = 1

		if K.FINISHMAPPING:
			rospy.loginfo('Finishing mapping of inside of tent')
			#Execute FINISH MAPPING Algorithm
			#if Finish Mapping is success
				#rospy.logwarn('Building successfully mapped')
				#K.resetStates()
				#K.EXITBUILDING = 1

		if K.EXITBUILDING:
			rospy.loginfo('Exiting building')
			#Execute EXIT BUILDING Algorithm
			#if EXIT BUILDING is success
				#rospy.logwarn('Vehicle has successfully exited building')
				#K.resetStates()
				#K.GOHOME = 1

		if K.GOHOME:

			rospy.loginfo('Mission Complete - Vehicle heading back to home')
			# K.modes.setReturnToHome() # Not sure if this works. It didn't work in simulation
			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.home_x
			K.positionSp.pose.position.y = K.home_y
			K.positionSp.pose.position.z = K.takeoff_height

			if abs(K.current_local_x - K.home_x)<= 1 and abs(K.current_local_y - K.home_y) <= 1:   # Rules give a 3m radius from goal
				rospy.loginfo('Reached home')
				K.resetStates()
				K.LAND = 1

		if K.LAND:
			rospy.loginfo('Vehicle is landing')
			K.modes.setAutoLandMode()
			if K.IS_LANDED:
				K.modes.setDisarm()
				K.resetStates()

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
