#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion
import tf

from math import *
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PoseWithCovarianceStamped
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

		# self.home_lat = ENTER_HOME_COORDINATES
		# self.home_lon = ENTER_HOME_COORDINATES # THIS WILL BE NEEDED EVENTUALLY BECAUSE THERE WILL BE TWO TAKEOFF/LANDING LOCATIONS

		self.building_center_lat = 22.317575 # Roughly the center of the KAUST field
		self.building_center_lon = 39.0984	# Roughly the center of the KAUST field

		# Current local ENU coordinates
		self.current_local_x = 0.0
		self.current_local_y = 0.0
		self.current_local_z = 0.0

		#Waypoints ENU Coordinates
		self.home_x = 0
		self.home_y = 0
		self.home_z = 0
		
		self.x_FenceLimit_max = 300 # 300 meters downfield from the starting position
		self.x_FenceLimit_min = -300 # 5 meters behind the starting the position 
		self.y_FenceLimit_max = 30 # 15 meters to the left of the starting position
		self.y_FenceLimit_min = -30 # 15 meters to the right of the starting position

		self.x_fence_max_warn = 295 # 295 meters downfield from the starting position
		self.x_fence_min_warn = -295 # 5 meters in back of starting position
		self.y_fence_max_warn = 29 # 14 meters to the left of starting position
		self.y_fence_min_warn = -29 # 14 meters to the right of the starting position
		self.z_limit_warn = 35 # Maximum height above ground that drone is allowed to go (MAX height is 40 meters - TO BE VERIFIED BY COMPETITION ORGANIZERS)

		self.building_center_x = -5#10
		self.building_center_y = 0

		self.takeoff_height_building = 1

		# Instantiate setpoint topic structures
		self.positionSp	= PoseStamped()

		#defining the modes
		self.modes = fcuModes()

		# States
		self.TAKEOFF = 0
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



		############# ENTRANCE SEARCH VARIABLES ########
		self.current_euler_xr = 0 # Current x rotation (roll)
		self.current_euler_yr = 0 # Current y rotation (pitch)
		self.current_euler_zr = 0 # Current z rotation (yaw)
		self.current_euler_w = 0

		self.current_quat_xr = 0 # In quaternion
		self.current_quat_yr = 0
		self.quat_local_zr = 0
		self.quat_local_w = 0

		self.entrance_search_FRONT_x = 0 # positive x is downfield
		self.entrance_search_BACK_x = 0
		self.entrance_search_RIGHT_y = 0
		self.entrance_search_LEFT_y = 0
		self.entrance_search_FRONT_AND_BACK_y = 0 # positive y is left
		self.entrance_search_RIGHT_AND_LEFT_x = 0
		self.entrance_search_z = 1

		self.euler_front_z = 0
		self.euler_right_z = pi/2
		self.euler_back_z = pi
		self.euler_left_z = -pi/2

		self.target_euler_x = 0
		self.target_euler_y = 0
		self.target_euler_z = 0

		self.entrance_search_WPS_FLAG = [0, 0, 0, 0, 0, 0, 0, 0]

		self.entrance_found_flag = 0

		self.greentagSp = PoseStamped()
		self.worker2Sp = PoseStamped()
		self.rotation = PoseStamped()

		self.worker2_found_flag = 0

		# This publisher is used because the PX4 avoidance package does not accept yaw inputs to the move_base_simple/goal topic
		# Setpoint publisher
		self.yaw_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
		##########################################################################################################################################
		
	

	def resetStates(self):
		self.TAKEOFF = 0
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

			self.current_quat_xr = msg.pose.orientation.x
			self.current_quat_yr = msg.pose.orientation.y
			self.current_quat_zr = msg.pose.orientation.z
			self.current_quat_w = msg.pose.orientation.w

			(self.current_euler_xr, self.current_euler_yr, self.current_euler_zr) = tf.transformations.euler_from_quaternion([self.current_quat_xr, self.current_quat_yr, self.current_quat_zr, self.current_quat_w])

	def objectPoseCb(self, msg):
		if msg is not None and K.WORKER2SEARCH:
			rospy.logwarn('Worker Found!')             
			self.worker2Sp = msg
			self.worker2Sp.header.frame_id='local_origin'
			self.worker2_found_flag = 1

	################# THIS NEEDS TO BE FILLED OUT MORE - ENTRANCE SEARCH CALL BACK
	def tagFoundCb(self,msg):
		if msg is not None and K.ENTRANCESEARCH:
			rospy.logwarn('Unblcoked Entrance Found!')
			self.entrance_found_flag = 1
	##################################################


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
		
		# self.x_FenceLimit_max, self.y_FenceLimit_max, _ = pm.geodetic2enu(self.lat_max, self.lon_max, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# self.x_FenceLimit_min, self.y_FenceLimit_min, _ = pm.geodetic2enu(self.lat_min, self.lon_min, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# rospy.loginfo('Successfully set fence limit maxes and mins')

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

	
	def EntranceSearchPattern(self):

		# Building is supposed to be 20x20. So from the center of the building, the walls
		# extend out 10 meters on all sides. We want our drone to do the scan from 3 meters
		# away from the buliding. THE FOLLOWING CODE ASSUMES BUIDLING CENTER Y = 0 (IS CENTERED ALONG WIDTH OF FIELD) !!!!!!

		self.entrance_search_FRONT_x = self.building_center_x - 5#13 # 3 meters in front of buliding looking towards it
		self.entrance_search_RIGHT_y = self.building_center_y - 5#13 # 3 meters to the right of the building
		self.entrance_search_BACK_x = self.building_center_x + 5#13 # 3 meters in back of the building
		self.entrance_search_LEFT_y = self.building_center_y + 5#13
		self.entrance_search_FRONT_AND_BACK_y = self.building_center_y
		self.entrance_search_RIGHT_AND_LEFT_x = self.building_center_x

		# THIS SECTION OF CODE IS INCREDIBLY BULKY. CONSIDER REVISING

		if self.entrance_search_WPS_FLAG[0] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_FRONT_x
			self.positionSp.pose.position.y = self.entrance_search_FRONT_AND_BACK_y
			self.positionSp.pose.position.z = self.entrance_search_z
			self.positionSp.pose.orientation.w = 1

			rospy.loginfo("Heading to front of building for tag search")

			if abs(self.current_local_x - self.entrance_search_FRONT_x)<= 1 and abs(self.current_local_y - self.entrance_search_FRONT_AND_BACK_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:
				
				self.rotation.header.frame_id = 'local_origin'
				(self.rotation.pose.orientation.x, self.rotation.pose.orientation.y, self.rotation.pose.orientation.z, self.rotation.pose.orientation.w) = tf.transformations.quaternion_from_euler(0, 0, self.euler_front_z)
				self.rotation.pose.position.x = self.positionSp.pose.position.x
				self.rotation.pose.position.y = self.positionSp.pose.position.y
				self.rotation.pose.position.z = self.positionSp.pose.position.z

				# This is a brute force method to override the commands being sent by the local planner. This may cause issues...
				while abs(self.current_euler_zr - self.euler_front_z) > .1:
					self.yaw_pub.publish(self.rotation)
					rospy.loginfo('Turning to face building')
				t = 0
				while t < 5000:
					self.yaw_pub.publish(self.rotation)
					rospy.loginfo('Vehicle oriented to building, waiting for 2 seconds to search for tag')	
					t = t + 1
				
				self.entrance_search_WPS_FLAG[0] = 1
			
		elif self.entrance_search_WPS_FLAG[1] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_FRONT_x
			self.positionSp.pose.position.y = self.entrance_search_RIGHT_y
			self.positionSp.pose.position.z = self.entrance_search_z
			self.positionSp.pose.orientation.w = 1

			if abs(self.current_local_x - self.entrance_search_FRONT_x)<= 1 and abs(self.current_local_y - self.entrance_search_RIGHT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:

				self.entrance_search_WPS_FLAG[1] = 1

		elif self.entrance_search_WPS_FLAG[2] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_RIGHT_AND_LEFT_x
			self.positionSp.pose.position.y = self.entrance_search_RIGHT_y
			self.positionSp.pose.position.z = self.entrance_search_z
			self.positionSp.pose.orientation.w = 1

			rospy.loginfo("Heading to right side of building for tag search")

			if abs(self.current_local_x - self.entrance_search_RIGHT_AND_LEFT_x)<= 1 and abs(self.current_local_y - self.entrance_search_RIGHT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:
				
				self.rotation.header.frame_id = 'local_origin'
				(self.rotation.pose.orientation.x, self.rotation.pose.orientation.y, self.rotation.pose.orientation.z, self.rotation.pose.orientation.w) = tf.transformations.quaternion_from_euler(0, 0, self.euler_right_z)
				self.rotation.pose.position.x = self.positionSp.pose.position.x
				self.rotation.pose.position.y = self.positionSp.pose.position.y
				self.rotation.pose.position.z = self.positionSp.pose.position.z

				# This is a brute force method to override the commands being sent by the local planner. This may cause issues...
				while abs(self.current_euler_zr - self.euler_right_z) > .1:
					self.yaw_pub.publish(self.rotation)
					rospy.loginfo('Turning to face building')
				t = 0
				while t < 5000:
					self.yaw_pub.publish(self.rotation)
					rospy.loginfo('Vehicle oriented to building, waiting for 2 seconds to search for tag')	
					t = t + 1

				self.entrance_search_WPS_FLAG[2] = 1

		elif self.entrance_search_WPS_FLAG[3] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_BACK_x
			self.positionSp.pose.position.y = self.entrance_search_RIGHT_y
			self.positionSp.pose.position.z = self.entrance_search_z
			self.positionSp.pose.orientation.w = 1

			if abs(self.current_local_x - self.entrance_search_BACK_x)<= 1 and abs(self.current_local_y - self.entrance_search_RIGHT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:

				self.entrance_search_WPS_FLAG[3] = 1

		elif self.entrance_search_WPS_FLAG[4] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_BACK_x
			self.positionSp.pose.position.y = self.entrance_search_FRONT_AND_BACK_y
			self.positionSp.pose.position.z = self.entrance_search_z
			self.positionSp.pose.orientation.w = 1

			rospy.loginfo("Heading to back side of building for tag search")

			if abs(self.current_local_x - self.entrance_search_BACK_x)<= 1 and abs(self.current_local_y - self.entrance_search_FRONT_AND_BACK_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:
				
				self.rotation.header.frame_id = 'local_origin'
				(self.rotation.pose.orientation.x, self.rotation.pose.orientation.y, self.rotation.pose.orientation.z, self.rotation.pose.orientation.w) = tf.transformations.quaternion_from_euler(0, 0, self.euler_back_z)
				self.rotation.pose.position.x = self.positionSp.pose.position.x
				self.rotation.pose.position.y = self.positionSp.pose.position.y
				self.rotation.pose.position.z = self.positionSp.pose.position.z

				# This is a brute force method to override the commands being sent by the local planner. This may cause issues...
				while abs(self.current_euler_zr - self.euler_back_z) > .1:
					self.yaw_pub.publish(self.rotation)
					rospy.loginfo('Turning to face building')
				t = 0
				while t < 5000:
					self.yaw_pub.publish(self.rotation)
					rospy.loginfo('Vehicle oriented to building, waiting for 2 seconds to search for tag')	
					t = t + 1

				self.entrance_search_WPS_FLAG[4] = 1

		elif self.entrance_search_WPS_FLAG[5] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_BACK_x
			self.positionSp.pose.position.y = self.entrance_search_LEFT_y
			self.positionSp.pose.position.z = self.entrance_search_z
			self.positionSp.pose.orientation.w = 1

			if abs(self.current_local_x - self.entrance_search_BACK_x)<= 1 and abs(self.current_local_y - self.entrance_search_LEFT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:

				self.entrance_search_WPS_FLAG[5] = 1

		elif self.entrance_search_WPS_FLAG[6] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_RIGHT_AND_LEFT_x
			self.positionSp.pose.position.y = self.entrance_search_LEFT_y
			self.positionSp.pose.position.z = self.entrance_search_z
			self.positionSp.pose.orientation.w = 1
			
			rospy.loginfo("Heading to left side of building for tag search")

			if abs(self.current_local_x - self.entrance_search_RIGHT_AND_LEFT_x)<= 1 and abs(self.current_local_y - self.entrance_search_LEFT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:
				
				self.rotation.header.frame_id = 'local_origin'
				(self.rotation.pose.orientation.x, self.rotation.pose.orientation.y, self.rotation.pose.orientation.z, self.rotation.pose.orientation.w) = tf.transformations.quaternion_from_euler(0, 0, self.euler_left_z)
				self.rotation.pose.position.x = self.positionSp.pose.position.x
				self.rotation.pose.position.y = self.positionSp.pose.position.y
				self.rotation.pose.position.z = self.positionSp.pose.position.z

				# This is a brute force method to override the commands being sent by the local planner. This may cause issues...
				while abs(self.current_euler_zr - self.euler_left_z) > .1:
					self.yaw_pub.publish(self.rotation)
					rospy.loginfo('Turning to face building')
				t = 0
				while t < 5000:
					self.yaw_pub.publish(self.rotation)
					rospy.loginfo('Vehicle oriented to building, waiting for 2 seconds to search for tag')	
					t = t + 1
					rospy.loginfo(t)

				self.entrance_search_WPS_FLAG[6] = 1

		elif self.entrance_search_WPS_FLAG[7] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_FRONT_x
			self.positionSp.pose.position.y = self.entrance_search_LEFT_y
			self.positionSp.pose.position.z = self.entrance_search_z
			self.positionSp.pose.orientation.w = 1

			if abs(self.current_local_x - self.entrance_search_FRONT_x)<= 1 and abs(self.current_local_y - self.entrance_search_LEFT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:

				self.entrance_search_WPS_FLAG[7] = 1
		
		else:
			rospy.logwarn(':[ COULD NOT FIND ANY TAGS')
			self.entrance_search_WPS_FLAG = [0, 0, 0, 0, 0, 0, 0, 0]

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
	# # This publisher is used because the PX4 avoidance package does not accept yaw inputs to the move_base_simple/goal topic
	# # Setpoint publisher
	# yaw_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

	# yaw_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=1)
	rate = rospy.Rate(10.0)

	# Publisher: PositionTarget
	avoid_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
	rate = rospy.Rate(10.0)

	servo_pub = rospy.Publisher("/servo", UInt16, queue_size=10)

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

		#GRADUAL CLOSURE OF GRIPPER
		if(K.gripper_flag):
			rospy.loginfo('Gripper closing')
			servo_pub.publish(K.RELEASE)
			rospy.sleep(0.2)
			for K.n.data in range(K.RELEASE.data,K.HOLD.data):
				if(rospy.is_shutdown()):
					continue
				servo_pub.publish(K.n)
				rospy.sleep(0.1)
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
			K.positionSp.pose.position.z = K.takeoff_height_building # Should be 1

			rospy.loginfo('Home X Position')
			rospy.loginfo(K.positionSp.pose.position.x)
			rospy.loginfo('Home Y Position')
			rospy.loginfo(K.positionSp.pose.position.y)
			rospy.loginfo('Takeoff Height')
			rospy.loginfo(K.positionSp.pose.position.z)

			K.positionSp.pose.orientation.w = 1.0	# IS THIS NEEDED?
			if abs(K.current_local_z - K.takeoff_height_building) < .1:
				rospy.logwarn("Reached Takeoff Height")	
				K.resetStates()
				K.ENTRANCESEARCH = 1


		if K.ENTRANCESEARCH:
			rospy.loginfo('Drone searching for entrance')
			if K.entrance_found_flag:
				rospy.logwarn('Unblocked entrance successfully found!')
				K.resetStates()
				K.ENTERBUILDING = 1
			else:
				# Execute Entrance Search Waypoints. If can't find anything, do it over again
				K.EntranceSearchPattern()
				# (K.target_euler_x, K.target_euler_y, K.target_euler_z) = tf.transformations.euler_from_quaternion([K.rotation.pose.pose.orientation.x, K.rotation.pose.pose.orientation.y, K.rotation.pose.pose.orientation.z, K.rotation.pose.pose.orientation.w])
				# rospy.loginfo('Desired Yaw Euler')
				# rospy.loginfo(K.target_euler_z)
				# rospy.loginfo('Current Yaw Euler')
				# rospy.loginfo(K.current_euler_zr)

		if K.ENTERBUILDING:
			rospy.loginfo('Drone entering building')
			#Execute Enter Building algorithm
			#if Enter Building is success
				#rospy.logwarn('Vehicle successfully')
				#K.resetStates()
				#K.WORKER2SEARCH = 1

		if K.WORKER2SEARCH:
			rospy.logfino('Vehicle searching for missing worker inside') 
			#Execute OUTSIDE WORKER SEARCH ALGORITHM
			#if WORKER2FOUND
				#ropsy.logwarn('Inside worker found!')
				#K.resetStates()
				#K.DELIVERAID2 = 1

		if K.DELIVERAID2:
			rospy.logfino('Delivering the first aid kit to inside worker')
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