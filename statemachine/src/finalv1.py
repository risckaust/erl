#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion
import tf

from tf.transformations import quaternion_from_euler
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
    def setVelocity(self,velocity):
    	rospy.wait_for_service('param/set')
    	try:
    		velocitySetService = rospy.ServiceProxy('param/set', mavros_msgs.srv.ParamSet)
    		velocitySetService(velocity)
    		rospy.logwarn('Just tried to set velocity parameter')
    	except rospy.ServiceException, e:
    	 	rospy.logwarn('FAILED to set velocity parameter')
    	 	print "service ParamSet call failed: %s. Velocity could not be set."%e

class RedTag:
	def __init__(self):
		self.x = []
		self.y = []
		self.total_num = 0
class BlueTag:
	def __init__(self):
		self.x = []
		self.y = []
		self.total_num = 0
class GreenTag:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.drone_x = 0
		self.drone_y = 0
		self.found = 0
class TEMPORARY_TAG_DETECT:
	def __init__(self):
		self.color = 'Red'
		self.x = 5
		self.y = 2

class Controller:

	def __init__(self):

	###################################################################################################################
	#------------------------------------------ GPS INFO -------------------------------------------------------------# 
	###################################################################################################################

		#------------------------------------ DATA TO ENTER -----------------------------------------#

		# GPS Fence
		self.lat_max = 22.312#22.3076#22.3176 # Location of Testing Field at KAUST
		self.lat_min = 22.310#22.3065#22.31725# Location of Testing Field at KAUST
		self.lon_max = 39.0955#39.1055#39.0983 # Location of Testing Field at KAUST
		self.lon_min = 39.0949#39.1045#39.0977 # Location of Testing Field at KAUST
		self.z_limit = 30
		
		#Waypoints GPS Coordiantes

		self.landing_zone_lat = 0
		self.landing_zone_lon = 0

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

		#----------------------------------------------------------------------------------------------#

		# Current GPS Coordinates
		self.current_lat = 0.0
		self.current_lon = 0.0
		self.current_alt = 0.0

	###################################################################################################################
	#------------------------------------------ LOCAL FRAME INFO -----------------------------------------------------# 
	###################################################################################################################

		# Current local ENU coordinates
		self.current_local_x = 0.0
		self.current_local_y = 0.0
		self.current_local_z = 0.0

		# Current angles
		self.current_yaw = 0

		#Waypoints ENU Coordinates
		self.landing_zone_x = 0
		self.landing_zone_y = 0

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

		self.x_fence_max_warn = 35#29 # 295 meters downfield from the starting position
		self.x_fence_min_warn = -10#-4 # 5 meters in back of starting position
		self.y_fence_max_warn = 20#14 # 14 meters to the left of starting position
		self.y_fence_min_warn = -20#-14 # 14 meters to the right of the starting position
		self.z_limit_warn = 35 # Maximum height above ground that drone is allowed to go (MAX height is 40 meters - TO BE VERIFIED BY COMPETITION ORGANIZERS)

		self.building_center_x = -5#10
		self.building_center_y = 0

		# SEARCH WAYPOINTS ARE TO BE SET BASED ON LOCATION OF BUILDING. WORKER IS PRESUMED TO BE FURTHER DOWNFIELD THAN BUILDING
		self.worker1_search_FAR_x = 0 
		self.worker1_search_NEAR_x = 0 
		self.worker1_search_LEFT_y = 0
		self.worker1_search_RIGHT_y = 0
		self.worker1_search_z = 8

		self.worker1_found_flag = 0

		self.worker1_search_WP_FLAG = [0, 0, 0]

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
		self.LOADPACKAGE1 = 0
		self.TAKEOFF1 = 0
		self.WAYPOINT1 = 0
		self.WAYPOINT2 = 0
		self.WAYPOINT3 = 0
		self.WORKER1SEARCH = 0
		self.DELIVERAID1 = 0
		self.GOTOREFULE = 0
		self.REFULE = 0
		self.LOADPACKAGE2 = 0
		self.TAKEOFF2 = 0
		self.GOTOBUILDING = 0
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
		self.building_entr1_lat = 0
		self.building_entr1_lon = 0
		self.building_entr2_lat = 0
		self.building_entr2_lon = 0
		self.building_entr3_lat = 0
		self.building_entr3_lon = 0

		self.building_entr1_x = self.building_center_x + 5
		self.building_entr1_y = self.building_center_y + 3
		self.building_entr2_x = self.building_center_x
		self.building_entr2_y = self.building_center_y - 5
		self.building_entr3_x = self.building_center_x
		self.building_entr3_y = self.building_center_y + 5

		self.yaw_building_entr1 = pi
		self.yaw_building_entr2 = pi/2
		self.yaw_building_entr3 = -pi/2

		# Positive x is downfield
		self.entrance_search_FRONT_x = self.building_center_x - 5#13 # 3 meters in front of buliding looking towards it
		self.entrance_search_RIGHT_y = self.building_center_y - 5#13 # 3 meters to the right of the building
		self.entrance_search_BACK_x = self.building_center_x + 5#13 # 3 meters in back of the building
		self.entrance_search_LEFT_y = self.building_center_y + 5#13
		self.entrance_search_z = 1

		self.yaw_front_z = 0
		self.yaw_right_z = pi/2
		self.yaw_back_z = pi
		self.yaw_left_z = -pi/2

		self.entrance_number_search = 1
		self.building_scan_WPS_FLAG = [0, 0, 0, 0, 0]
		self.entrance_search_WPS_FLAG = [0, 0, 0]

		self.red_tag = RedTag()
		self.blue_tag = BlueTag()
		self.green_tag = GreenTag()

		self.completed_full_rev = 0

		self.tag_info = TEMPORARY_TAG_DETECT() # THIS WILL BE A CUSTOM MESSAGE!!!!!!!!!!!!!
		self.worker2Sp = PoseStamped()

		self.worker2_found_flag = 0

		self.verifyTAG_flag = 0
		self.counterTAGCb = 0
		##########################################################################################################################################
		
		############# ENTER BUILDING VARIABLES ########################################
		self.enter_bldg_flag = [0, 0, 0, 0]
		self.enter_bldg_orien = 0

	def resetStates(self):
		self.LOADPACKAGE1 = 0
		self.TAKEOFF1 = 0
		self.WAYPOINT1 = 0
		self.WAYPOINT2 = 0
		self.WAYPOINT3 = 0
		self.WORKER1SEARCH = 0
		self.DELIVERAID1 = 0
		self.GOTOREFULE = 0
		self.REFULE = 0
		self.LOADPACKAGE2 = 0
		self.TAKEOFF2 = 0
		self.GOTOBUILDING = 0
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
		if msg is not None and (self.WORKER1SEARCH or self.WORKER2SEARCH):
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
			rospy.logwarn('sabatoor')			

	################# THIS NEEDS TO BE FILLED OUT MORE - ENTRANCE SEARCH CALL BACK
	def tagFoundCb(self,msg):
		if msg is not None and K.ENTRANCESEARCH:
			rospy.loginfo('Tag found')
			self.tag_info = msg
			self.verifyTAG_flag = 1
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

		#UNCOMMENT THIS CODE FOR GPS USE
		#####################################################################################################################################
		
		# self.x_FenceLimit_max, self.y_FenceLimit_max, _ = pm.geodetic2enu(self.lat_max, self.lon_max, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# self.x_FenceLimit_min, self.y_FenceLimit_min, _ = pm.geodetic2enu(self.lat_min, self.lon_min, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# rospy.loginfo('Successfully set fence limit maxes and mins')

		# self.landing_zone_x, self.landing_zone_y, _ = pm.geodetic2enu(self.landing_zone_lat, self.landing_zone_lon, self.z_limit, self.current_lat, self.current_lon, self.current_alt)

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

		# self.building_entr1_x, self.building_entr1_y = pm.geodetic2enu(self.building_entr1_lat, self.building_entr1_lon, 1.5, self.current_lat, self.current_lon, self.current_alt)
		# self.building_entr2_x, self.building_entr2_y = pm.geodetic2enu(self.building_entr2_lat, self.building_entr2_lon, 1.5, self.current_lat, self.current_lon, self.current_alt)
		# self.building_entr3_x, self.building_entr3_y = pm.geodetic2enu(self.building_entr3_lat, self.building_entr3_lon, 1.5, self.current_lat, self.current_lon, self.current_alt)

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

		if self.worker1_search_WP_FLAG[0] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			desired_yaw = atan2((self.worker1_search_RIGHT_y-self.current_local_y),(self.worker1_search_NEAR_x-self.current_local_x))
			quaternion_yaw = quaternion_from_euler(0, 0, desired_yaw)
			self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

			if abs(self.current_yaw-desired_yaw)<.3:

				self.positionSp.pose.position.x = self.worker1_search_NEAR_x
				self.positionSp.pose.position.y = self.worker1_search_RIGHT_y
				self.positionSp.pose.position.z = self.worker1_search_z

				if abs(self.current_local_x - self.worker1_search_NEAR_x)<= 1 and abs(self.current_local_y - self.worker1_search_RIGHT_y) <= 1 and abs(self.current_local_z - self.worker1_search_z) <= 0.5:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached worker search waypoint 1")
					self.worker1_search_WP_FLAG[0] = 1
				
		elif self.worker1_search_WP_FLAG[1] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			desired_yaw = 0
			quaternion_yaw = quaternion_from_euler(0, 0, desired_yaw)
			self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

			if abs(self.current_yaw-desired_yaw)<.1:

				self.positionSp.pose.position.x = self.worker1_search_FAR_x
				self.positionSp.pose.position.y = self.worker1_search_RIGHT_y
				self.positionSp.pose.position.z = self.worker1_search_z

				# THIS IS FOR SIMULATION PURPOSES ONLY!!!!! #######################################################################################
				###################################################################################################################################
				self.verifyPOI_flag = 1
				self.worker1_found_flag = 1
				self.worker1Sp.pose.position.x = self.current_local_x + 2
				self.worker1Sp.pose.position.y = self.current_local_y - 5
				self.worker1Sp.pose.position.z = 0
				###################################################################################################################################
				###################################################################################################################################

				if abs(self.current_local_x - self.worker1_search_FAR_x)<= 1 and abs(self.current_local_y - self.worker1_search_RIGHT_y) <= 1 and abs(self.current_local_z - self.worker1_search_z) <= 0.5:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached worker search waypoint 2")
					self.worker1_search_WP_FLAG[1] = 1

		elif self.worker1_search_WP_FLAG[2] == 0:

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
					self.worker1_search_WP_FLAG[2] = 1

		else:
			self.positionSp.header.frame_id = 'local_origin'
			desired_yaw = pi
			quaternion_yaw = quaternion_from_euler(0, 0, desired_yaw)
			self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

			if abs(abs(self.current_yaw)-desired_yaw)<.1:
				self.positionSp.pose.position.x = self.worker1_search_NEAR_x
				self.positionSp.pose.position.y = self.worker1_search_LEFT_y			
				self.positionSp.pose.position.z = self.worker1_search_z
				if abs(self.current_local_x - self.worker1_search_NEAR_x)<= 1 and abs(self.current_local_y - self.worker1_search_LEFT_y) <= 1 and abs(self.current_local_z - self.worker1_search_z) <= 0.5:
				
					rospy.logwarn(':[ COULD NOT FIND MISSING WOKRER IN FIELD! TRYING AGAIN')
					self.worker1_search_WP_FLAG = [0, 0, 0]

	def verifyTag(self):
		at_entrance = 0
		already_detected = 0
		if self.verifyTAG_flag:
			if self.tag_info.color == 'Red':
				for i in range(len(self.red_tag.x)):
					if abs(self.tag_info.x-self.red_tag.x[i]) < 1 and abs(self.tag_info.y-self.red_tag.y[i]) < 1:
						rospy.logwarn('Tag was already detected')
						already_detected = 1
				if already_detected != 1:
					self.red_tag.x.append(self.tag_info.x)
					self.red_tag.y.append(self.tag_info.y)
					self.red_tag.total_num = self.red_tag.total_num + 1
					rospy.loginfo("Found a new red tag")

			elif self.tag_info.color == 'Blue':
				for i in range(len(self.blue_tag.x)):
					if abs(self.tag_info.x-self.blue_tag.x[i]) < 1 and abs(self.tag_info.y-self.blue_tag.y[i]) < 1:
						rospy.logwarn('Tag was already detected')
						already_detected = 1
				if already_detected != 1:
					if (abs(self.tag_info.x - self.building_entr1_x) < 1 and abs(self.tag_info.y - self.building_entr1_y) < 1):
						at_entrance = 1
					elif (abs(self.tag_info.x - self.building_entr2_x) < 1 and abs(self.tag_info.y - self.building_entr2_y) < 1):
						at_entrance = 2
					elif (abs(self.tag_info.x - self.building_entr3_x) < 1 and abs(self.tag_info.y - self.building_entr3_y) < 1):
						at_entrance = 3

					if at_entrance > 1:
						self.blue_tag.x.append(self.tag_info.x)
						self.blue_tag.y.append(self.tag_info.y)
						self.blue_tag.total_num = self.blue_tag.total_num + 1
						rospy.loginfo('Found a new blue tag')

					elif at_entrance == 0:
						rospy.logwarn('Detected Blue Tag but it is not near an entrance so ignoring')

			elif self.tag_info.color == 'Green' and self.green_tag.found == 0:
				if (abs(self.tag_info.x - self.building_entr1_x) < 1 and abs(self.tag_info.y - self.building_entr1_y) < 1):
					at_entrance = 1
				elif (abs(self.tag_info.x - self.building_entr2_x) < 1 and abs(self.tag_info.y - self.building_entr2_y) < 1):
					at_entrance = 2
				elif (abs(self.tag_info.x - self.building_entr3_x) < 1 and abs(self.tag_info.y - self.building_entr3_y) < 1):
					at_entrance = 3

				if at_entrance > 0:
					self.green_tag.x = self.tag_info.x
					self.green_tag.y = self.tag_info.y
					self.green_tag.found = 1
					self.green_tag.drone_x = self.current_local_x
					self.green_tag.drone_y = self.current_local_y

					(_, _, self.enter_bldg_orien) = tf.transformations.euler_from_quaternion([self.positionSp.pose.orientation.x, self.positionSp.pose.orientation.y, self.positionSp.pose.orientation.z, self.positionSp.pose.orientation.w])

					rospy.loginfo('Green tag found!')
		
		self.verifyTAG_flag = 0
		self.counterTAGCb = 0

	def BuildingScan(self):

		# Building is supposed to be 20x20. So from the center of the building, the walls
		# extend out 10 meters on all sides. We want our drone to do the scan from 3 meters
		# away from the buliding. THE FOLLOWING CODE ASSUMES BUIDLING CENTER Y = 0 (IS CENTERED ALONG WIDTH OF FIELD) !!!!!!

		self.entrance_search_FRONT_x = self.building_center_x - 5#13 # 3 meters in front of buliding looking towards it
		self.entrance_search_RIGHT_y = self.building_center_y - 5#13 # 3 meters to the right of the building
		self.entrance_search_BACK_x = self.building_center_x + 5#13 # 3 meters in back of the building
		self.entrance_search_LEFT_y = self.building_center_y + 5#13

		if self.building_scan_WPS_FLAG[0] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_FRONT_x
			self.positionSp.pose.position.y = self.entrance_search_LEFT_y
			self.positionSp.pose.position.z = self.entrance_search_z

			if abs(self.current_local_x - self.entrance_search_FRONT_x)<= 1 and abs(self.current_local_y - self.entrance_search_LEFT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:
				quaternion_yaw = quaternion_from_euler(0, 0, self.yaw_front_z)
				self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)
				if abs(self.current_yaw-self.yaw_front_z) <= 0.2:
					self.building_scan_WPS_FLAG[0] = 1
			
		elif self.building_scan_WPS_FLAG[1] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_FRONT_x
			self.positionSp.pose.position.y = self.entrance_search_RIGHT_y
			self.positionSp.pose.position.z = self.entrance_search_z

			rospy.loginfo("Searching front side of building for tags")

			if abs(self.current_local_x - self.entrance_search_FRONT_x)<= 1 and abs(self.current_local_y - self.entrance_search_RIGHT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:
				quaternion_yaw = quaternion_from_euler(0, 0, self.yaw_right_z)
				self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)
				if abs(self.current_yaw-self.yaw_right_z) <= 0.2:
					self.building_scan_WPS_FLAG[1] = 1

		elif self.building_scan_WPS_FLAG[2] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_BACK_x
			self.positionSp.pose.position.y = self.entrance_search_RIGHT_y
			self.positionSp.pose.position.z = self.entrance_search_z

			rospy.loginfo("Searching right side of building for tags")

			if abs(self.current_local_x - self.entrance_search_BACK_x)<= 1 and abs(self.current_local_y - self.entrance_search_RIGHT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:
				quaternion_yaw = quaternion_from_euler(0, 0, self.yaw_back_z)
				self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)
				if abs(self.current_yaw-self.yaw_back_z) <= 0.2:
					self.building_scan_WPS_FLAG[2] = 1

		elif self.building_scan_WPS_FLAG[3] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_BACK_x
			self.positionSp.pose.position.y = self.entrance_search_LEFT_y
			self.positionSp.pose.position.z = self.entrance_search_z

			rospy.loginfo("Searching back side of building for tags")

			# FOR SIMULATION PURPOSES ONLY!!!!!!!!!!!!!!!!
			####################################################################
			# self.verifyTAG_flag = 1
			# self.tag_info.color = 'Blue'
			# self.tag_info.x = self.building_center_x - .5
			# self.tag_info.y = self.building_center_y - 5
			####################################################################

			if abs(self.current_local_x - self.entrance_search_BACK_x)<= 1 and abs(self.current_local_y - self.entrance_search_LEFT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:
				quaternion_yaw = quaternion_from_euler(0, 0, self.yaw_left_z)
				self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)
				if abs(self.current_yaw-self.yaw_left_z) <= 0.2:
					self.building_scan_WPS_FLAG[3] = 1

		elif self.building_scan_WPS_FLAG[4] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_FRONT_x
			self.positionSp.pose.position.y = self.entrance_search_LEFT_y
			self.positionSp.pose.position.z = self.entrance_search_z

			# FOR SIMULATION PURPOSES ONLY!!!!!!!!!!!!!!!!
			####################################################################
			# self.verifyTAG_flag = 1
			# self.tag_info.color = 'Red'
			# self.tag_info.x = self.building_center_x - 3
			# self.tag_info.y = self.building_center_y - 5
			####################################################################

			rospy.loginfo("Searching left side of building for tags")

			if abs(self.current_local_x - self.entrance_search_FRONT_x)<= 1 and abs(self.current_local_y - self.entrance_search_LEFT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:

				self.building_scan_WPS_FLAG[4] = 1

		else:
			rospy.loginfo('Completed full revolution')
			rospy.loginfo('Number of red tags found:')
			rospy.loginfo(self.red_tag.total_num)
			self.completed_full_rev = 1
			self.building_scan_WPS_FLAG = [0, 0, 0, 0, 0]

	def EntranceSearch(self,entrance_x,entrance_y,entrance_view_angle):
		# This function is only activated if the vehicle could not find the green tag on the first revolution
		entrance_search_counter = 0
		if (self.entrance_search_WPS_FLAG[0] == 0):
			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.z = 8

			if abs(self.current_local_z-self.positionSp.pose.position.z)<0.5:
				self.positionSp.header.frame_id = 'local_origin'
				self.positionSp.pose.position.x = entrance_x
				self.positionSp.pose.position.y = entrance_y

				self.entrance_search_WPS_FLAG[0] = 1

		elif (self.entrance_search_WPS_FLAG[1] == 0):

			if abs(self.current_local_x-entrance_x)<.5 and abs(self.current_local_y-entrance_y)<.5:
				self.positionSp.header.frame_id = 'local_origin'
				self.positionSp.pose.position.z = self.entrance_search_z

				quaternion_yaw = quaternion_from_euler(0, 0, entrance_view_angle)
				self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)
				self.entrance_search_WPS_FLAG[1] = 1

		elif (self.entrance_search_WPS_FLAG[2] == 0):

			if abs(self.current_local_z-self.entrance_search_z)<.1 and abs(self.current_yaw-entrance_view_angle) < 0.2:
				rospy.loginfo('Checking entrance to see what the color of the tag is')

				while entrance_search_counter < 80:

					# FOR SIMULATION PURPOSES ONLY!!!!!!!!!!!!!
					#################################################################################################
					# if self.entrance_number_search == 3 and abs(self.current_local_z-self.entrance_search_z)<.5:
					# 	self.verifyTAG_flag = 1
					# 	self.tag_info.color = 'Green'
					# 	self.tag_info.x = self.current_local_x
					# 	self.tag_info.y = self.current_local_y
					#################################################################################################

					self.verifyTag()
					entrance_search_counter = entrance_search_counter + 1

				self.entrance_search_WPS_FLAG[2] = 1
		else:
			rospy.loginfo('Could not find tag at current entrance')
			self.entrance_search_WPS_FLAG = [0, 0, 0]
			self.entrance_number_search = self.entrance_number_search + 1

	def verifyTag(self):
		at_entrance = 0
		already_detected = 0
		if self.verifyTAG_flag:
			if self.tag_info.color == 'Red':
				for i in range(len(self.red_tag.x)):
					if abs(self.tag_info.x-self.red_tag.x[i]) < 1 and abs(self.tag_info.y-self.red_tag.y[i]) < 1:
						rospy.logwarn('Tag was already detected')
						already_detected = 1
				if already_detected != 1:
					self.red_tag.x.append(self.tag_info.x)
					self.red_tag.y.append(self.tag_info.y)
					self.red_tag.total_num = self.red_tag.total_num + 1
					rospy.loginfo("Found a new red tag")

			elif self.tag_info.color == 'Blue':
				for i in range(len(self.blue_tag.x)):
					if abs(self.tag_info.x-self.blue_tag.x[i]) < 1 and abs(self.tag_info.y-self.blue_tag.y[i]) < 1:
						rospy.logwarn('Tag was already detected')
						already_detected = 1
				if already_detected != 1:
					if (abs(self.tag_info.x - self.building_entr1_x) < 1 and abs(self.tag_info.y - self.building_entr1_y) < 1):
						at_entrance = 1
					elif (abs(self.tag_info.x - self.building_entr2_x) < 1 and abs(self.tag_info.y - self.building_entr2_y) < 1):
						at_entrance = 2
					elif (abs(self.tag_info.x - self.building_entr3_x) < 1 and abs(self.tag_info.y - self.building_entr3_y) < 1):
						at_entrance = 3

					if at_entrance > 1:
						self.blue_tag.x.append(self.tag_info.x)
						self.blue_tag.y.append(self.tag_info.y)
						self.blue_tag.total_num = self.blue_tag.total_num + 1
						rospy.loginfo('Found a new blue tag')

					elif at_entrance == 0:
						rospy.logwarn('Detected Blue Tag but it is not near an entrance so ignoring')

			elif self.tag_info.color == 'Green' and self.green_tag.found == 0:
				if (abs(self.tag_info.x - self.building_entr1_x) < 1 and abs(self.tag_info.y - self.building_entr1_y) < 1):
					at_entrance = 1
				elif (abs(self.tag_info.x - self.building_entr2_x) < 1 and abs(self.tag_info.y - self.building_entr2_y) < 1):
					at_entrance = 2
				elif (abs(self.tag_info.x - self.building_entr3_x) < 1 and abs(self.tag_info.y - self.building_entr3_y) < 1):
					at_entrance = 3

				if at_entrance > 0:
					self.green_tag.x = self.tag_info.x
					self.green_tag.y = self.tag_info.y
					self.green_tag.found = 1
					self.green_tag.drone_x = self.current_local_x
					self.green_tag.drone_y = self.current_local_y

					(_, _, self.enter_bldg_orien) = tf.transformations.euler_from_quaternion([self.positionSp.pose.orientation.x, self.positionSp.pose.orientation.y, self.positionSp.pose.orientation.z, self.positionSp.pose.orientation.w])

					rospy.loginfo('Green tag found!')
		
		self.verifyTAG_flag = 0
		self.counterTAGCb = 0

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

	K.LOADPACKAGE1 = 1
	K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
	K.positionSp.pose.position.x = K.current_local_x
	K.positionSp.pose.position.y = K.current_local_y
	K.positionSp.pose.position.z = K.takeoff_height # Should be 1

	rospy.loginfo('Home X Position')
	rospy.loginfo(K.positionSp.pose.position.x)
	rospy.loginfo('Home Y Position')
	rospy.loginfo(K.positionSp.pose.position.y)
	rospy.loginfo('Takeoff Height')
	rospy.loginfo(K.positionSp.pose.position.z)
	#########################################################

	while not rospy.is_shutdown():

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

		if K.LOADPACKAGE1:
			rospy.loginfo('Gripper closing')
			servo_pub.publish(K.RELEASE)
			rospy.sleep(0.2)
			for K.n.data in range(K.RELEASE.data,K.HOLD.data):
				if(rospy.is_shutdown()):
					continue
				servo_pub.publish(K.n)
				rospy.sleep(0.1)
			K.resetStates()
			K.TAKEOFF1 = 1
		# !!!!!!!!!!!!!!!!!!!!! SAFETY PILOT WILL PUT INTO OFFBOARD MODE HERE !!!!!!!!!!!!!!!!!!!!!!!!!!
		if K.TAKEOFF1:
			rospy.logwarn('Vehicle is taking off')
			K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
			K.positionSp.pose.position.z = K.takeoff_height # Should be 1
			K.positionSp.pose.orientation.w = 1.0	# IS THIS NEEDED?
			if abs(K.current_local_z - K.takeoff_height) < .5:
				rospy.logwarn("Reached Takeoff Height")	
				K.resetStates()
				K.WAYPOINT1 = 1

		if K.WAYPOINT1:
			rospy.loginfo('Vehicle heading to waypoint 1')

			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.waypoint1_x
			K.positionSp.pose.position.y = K.waypoint1_y
			K.positionSp.pose.position.z = K.waypoint1_z

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
				rospy.loginfo('Outside worker found!')
				rospy.loginfo('Setting worker position to go to')

				K.positionSp.header.frame_id = 'local_origin'
				K.positionSp.pose.position.x = K.worker1Sp.pose.position.x
				K.positionSp.pose.position.y = K.worker1Sp.pose.position.y
				K.positionSp.pose.position.z = K.worker1_search_z
				K.resetStates()
				K.DELIVERAID1 = 1

			else:
				# EXECUTE WORKER SEARCH WAYPOINTS. IF CAN'T FIND ANYTHING, DO IT OVER AGAIN
				K.Worker1SearchPattern()

		if K.DELIVERAID1:
			rospy.loginfo('Trying to reach outside worker')
			K.positionSp.pose.position.z = K.worker1_search_z
			if (abs(K.current_local_x - K.positionSp.pose.position.x)<.1 and abs(K.current_local_y - K.positionSp.pose.position.y)<.1):
				servo_pub.publish(K.RELEASE)
				rospy.sleep(1)
				rospy.loginfo("Aid Dropped")
				K.resetStates()
				#K.ENTRANCESEARCH = 1
				K.GOTOREFULE = 1

		if K.GOTOREFULE:

			rospy.loginfo('Headed to building for entrance search')
			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.landing_zone_x
			K.positionSp.pose.position.y = K.landing_zone_y
			K.positionSp.pose.position.z = 10

			if abs(K.current_local_x - K.landing_zone_x)<= 1 and abs(K.current_local_y - K.landing_zone_y) <= 1:

				K.resetStates()
				K.REFULE = 1

		if K.REFULE:
			rospy.loginfo('Vehicle is landing to change battery and load next aid kit')
			K.modes.setAutoLandMode()
			if K.IS_LANDED:
				K.modes.setDisarm()
				K.resetStates()
				K.LOADPACKAGE2 = 1

		if K.LOADPACKAGE2:
			rospy.loginfo('Gripper closing')
			servo_pub.publish(K.RELEASE)
			rospy.sleep(5)
			for K.n.data in range(K.RELEASE.data,K.HOLD.data):
				if(rospy.is_shutdown()):
					continue
				servo_pub.publish(K.n)
				rospy.sleep(0.1)
			K.resetStates()
			K.TAKEOFF2 = 1
		# !!!!!!!!!!!!!!!!!!!!! SAFETY PILOT Will ARM AND PUT BACK INTO OFFBOARD MODE AGAIN HERE !!!!!!!!!!!!!!!!!!!!!!!!!!
		if K.TAKEOFF2:
			rospy.logwarn('Vehicle is taking off again')
			K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
			K.positionSp.pose.position.x = K.current_local_x 
			K.positionSp.pose.position.y = K.current_local_y
			K.positionSp.pose.position.z = K.entrance_search_z # Should be 1
			K.positionSp.pose.orientation.w = 1.0	# IS THIS NEEDED?

			quaternion_yaw = quaternion_from_euler(0, 0, K.yaw_front_z)
			K.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

			if abs(K.current_local_z - K.entrance_search_z) < .1:
				rospy.logwarn("Reached Takeoff Height")	
				K.resetStates()
				K.GOTOBUILDING = 1

		if K.GOTOBUILDING:

			rospy.loginfo('Headed to building for entrance search')
			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.entrance_search_FRONT_x
			K.positionSp.pose.position.y = K.entrance_search_LEFT_y
			K.positionSp.pose.position.z = K.entrance_search_z

			if abs(K.current_local_x - K.entrance_search_FRONT_x) <= 1 and abs(K.current_local_y - K.entrance_search_LEFT_y) <= 1:

				K.positionSp.pose.position.z = K.entrance_search_z
				quaternion_yaw = quaternion_from_euler(0, 0, K.yaw_front_z)
				K.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

				if abs(K.current_local_z-K.entrance_search_z) < 0.5 and abs(K.current_yaw-K.yaw_front_z) <= 0.2:

					K.resetStates()
					K.ENTRANCESEARCH = 1

		if K.ENTRANCESEARCH:
			K.verifyTag()
			rospy.loginfo('Searching for entrance')
			if (K.green_tag.found == 1) and (K.completed_full_rev == 1):
				rospy.logwarn('Unblocked entrance successfully found!')
				K.resetStates()
				K.ENTERBUILDING = 1
				rospy.loginfo('Number of Blue Tags found and locations (x/y):')
				rospy.loginfo(K.blue_tag.total_num)
				rospy.loginfo(K.blue_tag.x)
				rospy.loginfo(K.blue_tag.y)
				rospy.loginfo('Green Tag found at location (x/y):')
				rospy.loginfo(K.green_tag.x)
				rospy.loginfo(K.green_tag.y)
				rospy.loginfo('Number of Red Tags found and locations (x/y):')
				rospy.loginfo(K.red_tag.total_num)
				rospy.loginfo(K.red_tag.x)
				rospy.loginfo(K.red_tag.y)

			elif (K.green_tag.found == 0) and (K.completed_full_rev == 1):
				if K.entrance_number_search == 1:	# This variable is set in the EntranceSearch code
					K.EntranceSearch(K.building_entr1_x,K.building_entr1_y,K.yaw_building_entr1)
					rospy.loginfo('Checking first entrance')
				elif K.entrance_number_search == 2:
					K.EntranceSearch(K.building_entr2_x,K.building_entr2_y,K.yaw_building_entr2)
					rospy.loginfo('Checking second entrance')
				elif K.entrance_number_search == 3:
					rospy.loginfo('Checking third entrance')
					K.EntranceSearch(K.building_entr3_x,K.building_entr3_y,K.yaw_building_entr3)
				else:
					K.entrance_number_search = 1 # So that vehicle will check first entrance again
					rospy.loginfo('Still cannot find unblocked entrance. Try again at first entrance')
			else:
				# Execute building scan until full revolution
				K.BuildingScan()

		if K.ENTERBUILDING:
			# AS CODE IS WRITTEN, IT WILL GO TO WHERE THE DRONE WAS (WITH SAME ORIENTATION) WHEN IT DETECTED THE GREEN TAG, NOT THE DETECTED LOCATION OF THE GREEN TAG
			if K.enter_bldg_flag[0] == 0:
				if abs(K.current_local_x-K.green_tag.drone_x) > 1 or abs(K.current_local_y-K.green_tag.drone_y) > 1:
					K.positionSp.pose.position.z = 10
					if abs(K.current_local_z - 10) < .5:
						K.positionSp.pose.position.x = K.green_tag.drone_x
						K.positionSp.pose.position.y = K.green_tag.drone_y
				else:
					K.enter_bldg_flag[0] = 1
			elif K.enter_bldg_flag[1] == 0:
				if abs(K.current_local_x-K.green_tag.drone_x) < 1 and abs(K.current_local_y-K.green_tag.drone_y) < 1:
					K.positionSp.pose.position.z = K.entrance_search_z
					
					quaternion_yaw = quaternion_from_euler(0, 0, K.enter_bldg_orien)
					K.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

					K.enter_bldg_flag[1] = 1
			elif K.enter_bldg_flag[2] == 0:
				if abs(K.current_local_z - K.entrance_search_z) < .2 and abs(K.current_local_x-K.green_tag.drone_x) < .2 and abs(K.current_local_y-K.green_tag.drone_y) < .2:

					K.enter_bldg_flag[2] = 1

				
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