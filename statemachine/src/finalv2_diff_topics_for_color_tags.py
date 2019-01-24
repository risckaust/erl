#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion
import tf
import roslaunch

from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PoseWithCovarianceStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

import cv2


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
		self.lat = 0
		self.lon = 0
		self.total_num = 0
class BlueTag:
	def __init__(self):
		self.x = []
		self.y = []
		self.lat = 0
		self.lon = 0
		self.total_num = 0
class GreenTag:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.lat = 0
		self.lon = 0
		self.drone_x = 0
		self.drone_y = 0
		self.found = 0
class TEMPORARY_TAG_DETECT: # This will eventually get deleted
	def __init__(self):
		self.color = 'Red'
		self.x = 5
		self.y = 2

class Controller:

	def __init__(self):

	###################################################################################################################
	#--------------------------------------------- DATA TO ENTER -----------------------------------------------------# 
	###################################################################################################################

		#----------------Start state: Uncomment one-----------------#
		#############################################################
		self.start_state = 'LOADPACKAGE1'
		# self.start_state = 'WAYPOINT1'
		# self.start_state = 'WAYPOINT2'
		# self.start_state = 'WAYPOINT3'
		# self.start_state = 'WORKER1SEARCH'
		# self.start_state = 'LOADPACKAGE2'
		# self.start_state = 'GOTOBUILDING'
		# self.start_state = 'WORKER2SEARCH'
		# self.start_state = 'MAPPING'
		#############################################################

		# GPS Fence
		self.lat_max = 22.312#22.3076#22.3176 # Location of Testing Field at KAUST
		self.lat_min = 22.310#22.3065#22.31725# Location of Testing Field at KAUST
		self.lon_max = 39.0955#39.1055#39.0983 # Location of Testing Field at KAUST
		self.lon_min = 39.0949#39.1045#39.0977 # Location of Testing Field at KAUST
		self.z_limit = 30
		
		#Waypoints GPS Coordiantes

		self.landing_zone_lat = 47.397742 #Lat in simulation
		self.landing_zone_lon = 8.5455939 #Lon in simulation

		self.waypoint1_lat = 22.3118631#22.3070405#22.317490
		self.waypoint1_lon = 39.0952322#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint1_alt = 4

		self.waypoint2_lat = 22.3118631#22.3070405#22.317490
		self.waypoint2_lon = 39.0952322#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint2_alt = 4

		self.waypoint3_lat = 22.3118631#22.3070405#22.317490
		self.waypoint3_lon = 39.0952322#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint3_alt = 4

		# ----THIS INFO IS ONLY IF WE DON'T WANT TO BASE SEARCH OFF OF BUILDING GPS CENTER POINT----#
		#############################################################################################
		# These are the close/right (corner1) and far/left (corner2) corners for the building respectively
		self.bldg_search_corner1_lat = 47.3977007 # 47.3977007 FOR SIMULATION, close/right latitude value
		self.bldg_search_corner1_lon = 8.54546	# 8.54546 FOR SIMULATION, close/right longitude value
		self.bldg_search_corner2_lat = 47.3977828 # 47.3977828 FOR SIMULATION, far/left latitude value
		self.bldg_search_corner2_lon = 8.545595 # 8.545595 FOR SIMULATION, far/left longitude value
		#############################################################################################

		# Outside Worker Search Variables in GPS

		self.building_center_lat = 22.317575 # 22.317575 Roughly the center of the KAUST field
		self.building_center_lon = 39.0984	# 39.0984 Roughly the center of the KAUST field

		self.building_entr1_lat = 0	# I haven't set these yet
		self.building_entr1_lon = 0
		self.building_entr2_lat = 0
		self.building_entr2_lon = 0
		self.building_entr3_lat = 0
		self.building_entr3_lon = 0

		self.yaw_building_entr1 = pi # MAYBE SHOULD BE CHANGED? THIS WILL DEPEND ON DRONE BOOTUP ORIENTATION (currently, we'd have to make sure that the drone is started with 0 deg exactly North)
		self.yaw_building_entr2 = pi/2 # ditto
		self.yaw_building_entr3 = -pi/2	# ditto

	###################################################################################################################
	#--------------------------------------- VARIABLES USED IN CODE --------------------------------------------------# 
	###################################################################################################################

		#--------------------Random Variables---------------------#
		###########################################################

		# Instantiate setpoint topic structures
		self.positionSp	= PoseStamped()
		#defining the modes
		self.modes = fcuModes()

		#-------------------Data Log Variables--------------------#
		###########################################################
		self.current_state = 'LOADPACKAGE1'
		self.current_signal = 'Running'
		self.previous_cycle_state = 'LOADPACKAGE1'
		self.previous_cycle_signal = 'Running'
		self.timer_sec = 0
		self.start_time = 0
		self.previous_timer_sec = 0
		self.worker_lat = 0
		self.worker_lon = 0

		#--------------Current Position Information---------------#
		###########################################################

		# Current local ENU coordinates
		self.current_local_x = 0.0
		self.current_local_y = 0.0
		self.current_local_z = 0.0

		# Current angles
		self.current_yaw = 0
		self.current_yaw_deg = 0

		# Current GPS Coordinates
		self.current_lat = 0.0
		self.current_lon = 0.0
		self.current_alt = 0.0

		#-----------------------Field Setup------------------------#
		############################################################

		self.home_lat = 0
		self.home_lon = 0

		self.landing_zone_x = 0
		self.landing_zone_y = 0

		self.home_x = 0
		self.home_y = 0
		self.home_z = 0

		self.x_FenceLimit_max = 30 # 300 meters downfield from the starting position
		self.x_FenceLimit_min = -20 # 5 meters behind the starting the position 
		self.y_FenceLimit_max = 15 # 15 meters to the left of the starting position
		self.y_FenceLimit_min = -15 # 15 meters to the right of the starting position

		self.x_fence_max_warn = self.x_FenceLimit_max - 2 #29 # 295 meters downfield from the starting position
		self.x_fence_min_warn = self.x_FenceLimit_min + 2 #-4 # 5 meters in back of starting position
		self.y_fence_max_warn = self.y_FenceLimit_max - 2 #14 # 14 meters to the left of starting position
		self.y_fence_min_warn = self.y_FenceLimit_min + 2 #-14 # 14 meters to the right of the starting position
		self.z_limit_warn = 35 # Maximum height above ground that drone is allowed to go (MAX height is 40 meters - TO BE VERIFIED BY COMPETITION ORGANIZERS)

		self.building_center_x = -5#10
		self.building_center_y = 0

		#----------------------State Machine-----------------------#
		############################################################
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
		self.MAPPING = 0
		self.EXITBUILDING = 0
		self.GOHOME = 0
		self.LAND = 0
		self.HOVER = 0

		#----------------------Load Aid Kit-----------------------#
		###########################################################
		self.n = UInt16()
		self.HOLD = UInt16()
		self.RELEASE = UInt16()
		self.HOLD.data = 131
		self.RELEASE.data = 80
		self.gripper_flag = True

		#----------------------Takeoff----------------------------#
		###########################################################
		self.takeoff_height = 1.5
		
		#-------------------Waypoint Variables--------------------#
		###########################################################
		self.waypoint1_x = 0
		self.waypoint1_y = 0
		self.waypoint1_z = 1.5

		self.waypoint2_x = 0
		self.waypoint2_y = 0
		self.waypoint2_z = 1.5

		self.waypoint3_x = 0
		self.waypoint3_y = 0
		self.waypoint3_z = 1.5

		#-----------------Worker Search Variables-----------------#
		###########################################################

		# SEARCH WAYPOINTS ARE TO BE SET BASED ON LOCATION OF 
		# BUILDING. WORKER IS PRESUMED TO BE FURTHER DOWNFIELD 
		# THAN BUILDING

		self.verifyTAG_flag = 0
		self.counterTAGCb = 0

		# For camera, taking pictures
		self.bridge = CvBridge()

		#------------------WORKER1-------------------#
		self.worker1_search_FAR_x = 0 
		self.worker1_search_NEAR_x = 0 
		self.worker1_search_LEFT_y = 0
		self.worker1_search_RIGHT_y = 0
		self.worker1_search_z = 8

		self.worker1_found_flag = 0

		self.worker1_search_WP_FLAG = [0, 0, 0]

		self.worker1Sp = PoseStamped()

		#------------------WORKER2-------------------#
		self.worker2Sp = PoseStamped()

		self.worker2_found_flag = 0

		# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1THERE WILL BE MORE VARIABLES HERE AFTER NEW AVOIDANCE PACKAGE ADDED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		#---------------ENTRANCE SEARCH VARIABLES-----------------#
		###########################################################

		self.building_entr1_x = self.building_center_x + 5
		self.building_entr1_y = self.building_center_y + 3
		self.building_entr2_x = self.building_center_x
		self.building_entr2_y = self.building_center_y - 5
		self.building_entr3_x = self.building_center_x
		self.building_entr3_y = self.building_center_y + 5

		# Positive x is downfield, positive y is to the left
		self.entrance_search_FRONT_x = self.building_center_x - 5#13 # 3 meters in front of buliding looking towards it
		self.entrance_search_RIGHT_y = self.building_center_y - 5#13 # 3 meters to the right of the building
		self.entrance_search_BACK_x = self.building_center_x + 5#13 # 3 meters in back of the building
		self.entrance_search_LEFT_y = self.building_center_y + 5#13
		self.entrance_search_z = 1

		# Yaw angles so that drone will always face a side of the building
		self.yaw_front_z = 0
		self.yaw_right_z = pi/2
		self.yaw_back_z = pi
		self.yaw_left_z = -pi/2

		# Variables to handle transitions within the state (i.e. reaching the updated yaw position before actually heading to the next corner)
		self.entrance_number_search = 1
		self.building_scan_WPS_FLAG = [0, 0, 0, 0, 0]
		self.entrance_search_WPS_FLAG = [0, 0, 0]

		self.red_tag = RedTag()
		self.blue_tag = BlueTag()
		self.green_tag = GreenTag()

		self.redtagSp = PoseStamped()
		self.bluetagSp = PoseStamped()
		self.greentagSp = PoseStamped()

		self.tag_color = "None"

		self.completed_full_rev = 0

		self.counterCb = 0
		self.verifyPOI_flag = 0

		self.ready2capturered = 0
		self.ready2captureblue = 0
		self.ready2capturegreen = 0

		#----------------ENTER BUILDING VARIABLES-----------------#
		###########################################################

		# Variables to handle transitions within the state
		self.enter_bldg_flag = [0, 0, 0, 0]
		self.enter_bldg_orien = 0

	######### Callbacks #########

	# Land state callback
	def landStateCb(self, msg):
		if msg is not None:
			if msg.landed_state == 1:
				self.IS_LANDED = True
			else:
				self.IS_LANDED = False

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
			
			#This is for heading information to put into KML file. 0 Degrees is North and 90 Degrees is East.
			if self.current_yaw < 0:
				self.current_yaw_deg = abs(self.current_yaw*180/pi)
			elif self.current_yaw >= 0:
				self.current_yaw_deg = (2*pi - self.current_yaw)*180/pi

	def WorkerPoseCb(self, msg):
		# Takes in possible worker location given from /detected_object_3d_pos topic and then sets the verifyPOI_flag to check if it was
		# actually a person. If it is, the verifyPOI_flag will set self.worker1_found_flag = 1 and then this function will update the
		# location of the person so that it can deliver the package accurately within ~2 meters of the mannequin

		if msg is not None and (self.WORKER1SEARCH or self.WORKER2SEARCH):
			rospy.logwarn('Item of interest found')             
			self.worker1Sp = msg
			self.worker1Sp.header.frame_id='local_origin'
			self.verifyPOI_flag = 1
			self.counterCb = self.counterCb + 1 # This counter is used to tell verifyPOI() that the message has been updated, meaning that it is still seeing something
			
			self.positionSp.pose.position.z = self.worker1_search_z
			# NOT SURE WHY WE DID THIS:
			# self.worker1Sp.pose.position.z = self.worker1_search_z

			(self.worker_lat, self.worker_lon, _) = pm.enu2geodetic(self.worker1Sp.pose.position.x,self.worker1Sp.pose.position.y,0,self.home_lat,self.home_lon,0)
		if msg is not None and self.worker1_found_flag:
			self.worker1Sp = msg
			self.worker1Sp.header.frame_id='local_origin'

			self.positionSp.pose.position.z = self.worker1_search_z
			# NOT SURE WHY WE DID THIS:
			# self.worker1Sp.pose.position.z = self.worker1_search_z			

			(self.worker_lat, self.worker_lon, _) = pm.enu2geodetic(self.worker1Sp.pose.position.x,self.worker1Sp.pose.position.y,0,self.home_lat,self.home_lon,0)

	################# THE SUBSCRIBER TOPIC FOR THESE FUNCTIONS STILL NEEDS TO BE PUBLISHED TO IN A DIFFERENT TAG_POSITION_LOCALIZATION
	def redtagPoseCb(self,msg):
		# Called if a red tag was thought to be seen. Then will hand off testing to verifyTag()
		if msg is not None and K.ENTRANCESEARCH:
			self.redtagSp = msg
			self.redtagSp.header.frame_id='local_origin'
			self.verifyTAG_flag = 1
			self.tag_color = 'Red'

	def bluetagPoseCb(self,msg):
		# Called if a blue tag was thought to be seen. Then will hand off testing to verifyTag()
		if msg is not None and K.ENTRANCESEARCH:
			self.bluetagSp = msg
			self.bluetagSp.header.frame_id='local_origin'
			self.verifyTAG_flag = 1
			self.tag_color = 'Blue'

	def greentagPoseCb(self,msg):
		# Called if a green tag was thought to be seen. Then will hand off testing to verifyTag()
		if msg is not None and K.ENTRANCESEARCH:
			self.greentagSp = msg
			self.greentagSp.header.frame_id='local_origin'
			self.verifyTAG_flag = 1
			self.tag_color = 'Green'
	###################################################################################################################################

	def WorkerImageCb(self, msg):
		# This function will capture a ZED camera shot only if it is currently in a workersearch state and if a worker is found.
		# It will also log the information in the ObjectOfInterest...txt file (The file should change name each time this entire
		# script is run again)
		if msg is not None and ((self.worker1_found_flag*self.WORKER1SEARCH>0) or (self.worker2_found_flag*self.WORKER2SEARCH)>0):
			print("Received an image!")
			self.timer_sec = round(rospy.get_time() - self.start_time)
			try:
			# Convert your ROS Image message to OpenCV2
				cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			except CvBridgeError, e:
				print(e)
			else:
				# Save your OpenCV2 image as a jpeg
				logname = "ObjectOfInterest_" + str(self.start_time) + ".txt"
				hs = open(logname,"a")
				s = "\nTime Stamp (sec):\n"
				s = s + str(self.timer_sec)
				if self.WORKER1SEARCH:
					picname = "Worker1_" + str(self.start_time) + ".txt"
					cv2.imwrite(picname, cv2_img)
					s = s + "\nTarget ID:\nOutside Worker\nTarget Position (Lat, Lon):\n"
					s = s + "(" + str(self.worker_lat) + ", " + str(self.worker_lon) + ")\n"
				elif self.WORKER2SEARCH:
					picname = "Worker2_" + str(self.start_time) + ".txt"
					cv2.imwrite(picname, cv2_img)
					s = s + "\nTarget ID:\nInside Worker\nTarget Position (Lat, Lon):\n"
					s = s + "(" + str(self.worker_lat) + ", " + str(self.worker_lon) + ")\n"
				hs.write(s)
				hs.close()

	def TagImageCb(self, msg):
		# Similar to WorkerImageCb. Will take an image of the tag and will log the tag as a particular color
		# depending on which flag (self.ready2capturered, self.ready2captureblue, or self.ready2capturegreen) 
		# was set by verifyTag(). Will only activate if in ENTRANCESEARCH state and one of the tags is set.
		if msg is not None and self.ENTRANCESEARCH and ((self.ready2capturered + self.ready2captureblue + self.ready2capturegreen)>0):
			print("Received an image!")
			self.timer_sec = round(rospy.get_time() - self.start_time)
			try:
			# Convert your ROS Image message to OpenCV2
				cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			except CvBridgeError, e:
				print(e)
			else:
				# Save your OpenCV2 image as a jpeg
				logname = "ObjectOfInterest_" + str(self.start_time) + ".txt"
				hs = open(logname,"a")
				if self.ready2capturered:
					name = "Red" + str(self.red_tag.total_num) + "_" + str(self.start_time) + ".jpeg"
					cv2.imwrite(name, cv2_img)
					r = "\nTime Stamp (sec):\n"
					r = r + str(self.timer_sec)
					r = r + "\nTarget ID:\nRed Tag\nTarget Position (Lat, Lon):\n"
					r = r + "(" + str(self.red_tag.lat) + ", " + str(self.red_tag.lon) + ")\n"
					hs.write(r)
					self.ready2capturered = 0
				if self.ready2captureblue:
					name = "Blue" + str(self.blue_tag.total_num) + "_" + str(self.start_time) + ".jpeg"
					cv2.imwrite(name, cv2_img)
					b = "\nTime Stamp (sec):\n"
					b = b + str(self.timer_sec)
					b = b + "\nTarget ID:\nBlue Tag\nTarget Position (Lat, Lon):\n"
					b = b + "(" + str(self.blue_tag.lat) + ", " + str(self.blue_tag.lon) + ")\n"
					hs.write(b)
					self.ready2captureblue = 0
				if self.ready2capturegreen:
					name = "Green_" + str(self.start_time) + ".jpeg"
					cv2.imwrite(name, cv2_img)
					g = "\nTime Stamp (sec):\n"
					g = g + str(self.timer_sec)
					g = g + "\nTarget ID:\nGreen Tag\nTarget Position (Lat, Lon):\n"
					g = g + "(" + str(self.green_tag.lat) + ", " + str(self.green_tag.lon) + ")\n"
					hs.write(g)
					self.ready2capturegreen = 0
				hs.close()

	def VehicleNavLog(self):
		# Drone navigation logger that activates either after each second, or if the
		# current state has changed. Also changes the name of the file to write to
		# after each start of the python script (unless using simulation because the
		# simulation resets time after each start whereas in actual testing, rospy.get_time()
		# is the amount of seconds passed from some date/time in 1970.)
		self.timer_sec = round(rospy.get_time() - self.start_time)
		if (self.previous_timer_sec != self.timer_sec) or (self.current_state != self.previous_cycle_state) or (self.current_signal != self.previous_cycle_signal):
			name = "VehicleNavData_" + str(self.start_time) + ".txt"
			hs = open(name,"a")
			s = "\nTime Stamp (sec):\n"
			s = s + str(self.timer_sec)
			s = s + "\nVehicle Location (Lat, Lon):\n"
			s = s + "(" + str(self.current_lat) + ", " + str(self.current_lon) + ")\n"
			s = s + "Current Heading (0=North, 90=East):\n"
			s = s + str(self.current_yaw_deg)
			s = s + "\nCurrent State:\n"
			s = s + self.current_state
			s = s + "\nCurrent Signal:\n"
			s = s + self.current_signal
			s = s + "\n"
			hs.write(s)
			hs.close()
			self.previous_timer_sec = round(rospy.get_time() - self.start_time)
			self.previous_cycle_state = self.current_state
			self.previous_cycle_signal = self.current_signal

	def setWayoints_and_Fence(self):
		# Converts all GPS locations to the local frame.

		rospy.loginfo('Current lat')
		rospy.loginfo(self.current_lat)

		rospy.loginfo('Current lon')
		rospy.loginfo(self.current_lon)

		rospy.loginfo('Current alt')
		rospy.loginfo(self.current_alt)

		rospy.loginfo('Current x')
		rospy.loginfo(self.current_local_x)

		rospy.loginfo('Current y')
		rospy.loginfo(self.current_local_y)

		rospy.loginfo('Current z')
		rospy.loginfo(self.current_local_z)

		#UNCOMMENT THIS CODE FOR GPS USE
		#####################################################################################################################################
		# self.home_lat = self.current_lat
		# self.home_lon = self.current_lon

		# self.x_FenceLimit_max, self.y_FenceLimit_max, _ = pm.geodetic2enu(self.lat_max, self.lon_max, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# self.x_FenceLimit_min, self.y_FenceLimit_min, _ = pm.geodetic2enu(self.lat_min, self.lon_min, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# rospy.loginfo('Successfully set fence limit maxes and mins')

		# self.landing_zone_x, self.landing_zone_y, _ = pm.geodetic2enu(self.landing_zone_lat, self.landing_zone_lon, self.z_limit, self.current_lat, self.current_lon, self.current_alt)

		# self.waypoint1_x, self.waypoint1_y, _ = pm.geodetic2enu(self.waypoint1_lat, self.waypoint1_lon, self.waypoint1_alt, self.current_lat, self.current_lon, self.current_alt)
		# self.waypoint2_x, self.waypoint2_y, _ = pm.geodetic2enu(self.waypoint2_lat, self.waypoint2_lon, self.waypoint2_alt, self.current_lat, self.current_lon, self.current_alt)
		# self.waypoint3_x, self.waypoint3_y, _ = pm.geodetic2enu(self.waypoint3_lat, self.waypoint3_lon, self.waypoint3_alt, self.current_lat, self.current_lon, self.current_alt)
		# rospy.loginfo('Successfully set waypoints to x,y,z')

		# self.x_fence_max_warn = self.x_FenceLimit_max - 2
		# self.x_fence_min_warn = self.x_FenceLimit_min + 2
		# self.y_fence_max_warn = self.y_FenceLimit_max - 2
		# self.y_fence_min_warn = self.y_FenceLimit_min + 2
		# self.z_limit_warn = self.z_limit - .5

		# self.building_center_x, self.building_center_y, _ = pm.geodetic2enu(self.building_center_lat, self.building_center_lon, self.z_limit, self.current_lat, self.current_lon, self.current_alt)

		# self.building_entr1_x, self.building_entr1_y, _ = pm.geodetic2enu(self.building_entr1_lat, self.building_entr1_lon, 1.5, self.current_lat, self.current_lon, self.current_alt)
		# self.building_entr2_x, self.building_entr2_y, _ = pm.geodetic2enu(self.building_entr2_lat, self.building_entr2_lon, 1.5, self.current_lat, self.current_lon, self.current_alt)
		# self.building_entr3_x, self.building_entr3_y, _ = pm.geodetic2enu(self.building_entr3_lat, self.building_entr3_lon, 1.5, self.current_lat, self.current_lon, self.current_alt)


		# #----- UNCOMMENT THIS ONLY IF YOU WANT TO SPECIFY LOCAL ENTRANCE SEARCH PATH BASED ON GPS LOCATIONS OF TWO CORNERs OF SQUARE PATH INSTEAD OF BUILDING CENTER POINT ------ #
		# self.entrance_search_FRONT_x, self.entrance_search_RIGHT_y, _ = pm.geodetic2enu(self.bldg_search_corner1_lat, self.bldg_search_corner1_lon, 1.5, self.current_lat, self.current_lon, self.current_alt)
		# self.entrance_search_BACK_X, self.entrance_search_LEFT_y, _ = pm.geodetic2enu(self.bldg_search_corner2_lat, self.bldg_search_corner2_lon, 1.5, self.current_lat, self.current_lon, self.current_alt)
		# #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------#


		#####################################################################################################################################

		rospy.loginfo('x Fence Max Warn')
		rospy.loginfo(self.x_fence_max_warn)
		rospy.loginfo('x Fence Min Warn')
		rospy.loginfo(self.x_fence_min_warn)
		rospy.loginfo('y Fence Max Warn')
		rospy.loginfo(self.y_fence_max_warn)
		rospy.loginfo('y Fence Min Warn')
		rospy.loginfo(self.y_fence_min_warn)
		rospy.loginfo('z Height Limit Warn')
		rospy.loginfo(self.z_limit_warn)

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
		# Function that will always check to see if the drone is too close to the hard
		# competition arena boundaries.
		value = False
		if((self.current_local_x > self.x_fence_max_warn) or (self.current_local_y > self.y_fence_max_warn) or (self.current_local_z > self.z_limit_warn)):
			rospy.logwarn('Vehicle is too close to fence!')
			value = True
		elif((self.current_local_x < self.x_fence_min_warn) or (self.current_local_y < self.y_fence_min_warn)):
			rospy.logwarn('Vehicle is too close to fence!')
			value = True
		return value

	def isValidWaypoint(self,waypoint_x,waypoint_y,waypoint_z):
		# Does an initial check at each start to make sure the waypoints are within
		# the given competition arena boundaries.
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
		self.MAPPING = 0
		self.EXITBUILDING = 0
		self.GOHOME = 0
		self.LAND = 0
		self.HOVER = 0

	def verifyPOI(self):
		# Gives vehicle time (500*0.01 seconds) to verify the correct identification of
		# a worker and that it wasn't a random false detection.
		if self.verifyPOI_flag:
			counter = 0
			for i in range(500):

				previous_counter = self.counterCb # self.counterCb is constantly getting updated every time objectPoseCb is called
				rospy.sleep(.01) # Give self.counterCb chance to update if objectPoseCb is called again
				if self.counterCb is not previous_counter:
					counter = counter + 1
			if counter >= 5:
				self.worker1_found_flag = 1
				rospy.loginfo("Outside worker found!")
			elif counter < 5:
				rospy.loginfo("Detected false positive. Continuing search.")
		
		self.verifyPOI_flag = 0
		self.counterCb = 0

	def Worker1SearchPattern(self):

		# Search algorithm is as follows: Drone follows a square pattern starting at the 
		# closest right corner, goes down field on right side, turn left at the end of 
		# the field, and comes back on left side. The search path side closest to the 
		# control station is basically at the same distance downfield as the building. 
		# Since the field is 30 meters wide, in order to span the entire field, the drone
		# should search at about 8 meters high and 7.5 meters inward from the field boundaries.

		#!!!!!!!!!!!!!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		# Drone should go to altitude that is definitely above buliding after completing waypoint 3
		#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		self.worker1_search_LEFT_y = self.y_FenceLimit_max - 7.5 # 7.5 meters from the left edge of the field
		self.worker1_search_RIGHT_y = self.y_FenceLimit_min + 7.5 # 7.5 meters from the right edge of the field

		self.worker1_search_NEAR_x = self.entrance_search_FRONT_x # DRONE WILL START AT SAME DISTANCE DOWNFIELD AS NEAREST X POINT FOR ENTRANCE SEARCH
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

					#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
					###########################################################################################
					self.current_signal = "Reached worker search waypoint 1"
					###########################################################################################

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

					#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
					###########################################################################################
					self.current_signal = "Reached worker search waypoint 3"
					###########################################################################################

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

				#----------THIS IS FOR SIMULATION PURPOSES ONLY TO SPOOF WORKER FOUND-------------------------#
				###############################################################################################
				# self.verifyPOI_flag = 1
				# self.worker1_found_flag = 1
				# self.worker1Sp.pose.position.x = self.current_local_x - 2
				# self.worker1Sp.pose.position.y = self.current_local_y - 5
				# self.worker1Sp.pose.position.z = 0
				###############################################################################################
				###############################################################################################

				if abs(self.current_local_x - self.worker1_search_NEAR_x)<= 1 and abs(self.current_local_y - self.worker1_search_LEFT_y) <= 1 and abs(self.current_local_z - self.worker1_search_z) <= 0.5:
				
					rospy.logwarn(':[ COULD NOT FIND MISSING WOKRER IN FIELD! TRYING AGAIN')
					self.worker1_search_WP_FLAG = [0, 0, 0]

	def BuildingScan(self):

		# This function generally follows the same type of path as the worker search except
		# that it always faces towards the building so that it can identify colored tags.

		# IF USING CENTER POINT OF BUILDING FOR SCAN (COMMENTED OUT THE GPS CONVERSIONS in setWaypoints_and_Fence):
		# User enters GPS locations of the building center location (or the local_origin coordinates if not using
		# setWaypoints_and_Fence). Since the building is 20x20 meters, the search corners should be 12 meters away 
		# from the center point so that the drone will do the scan from 2 meters away. THIS IS SUBJECT TO HOW WELL
		# THE TAG DETECTION PERFORMS! CURRENTLY, IN THE CODE THIS METHOD ALSO ASSUMES BUIDLING CENTER Y = 0 (IS CENTERED ALONG WIDTH OF FIELD) !!!!!!

		# IF USING TWO CORNERS FOR BUILDING SCAN (UNCOMMENTED THE APPROPRIATE LINES in setWaypoints_and_Fence):
		# User enters GPS locations of the near/right corner and far/left corner, and then 
		# setWaypoints_and_Fence converts them to local x/y to give the four corner coordinates.


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

				#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
				###########################################################################################
				self.current_signal = "Reached entrance search wapyoint NEAR/RIGHT"
				###########################################################################################

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

			# -------------------------------FOR SIMULATION PURPOSES ONLY----------------------------------#
			################################################################################################
			# self.verifyTAG_flag = 1
			# self.tag_color = 'Blue'
			# self.tag_info.x = self.building_center_x - .5
			# self.tag_info.y = self.building_center_y - 5
			################################################################################################

			if abs(self.current_local_x - self.entrance_search_BACK_x)<= 1 and abs(self.current_local_y - self.entrance_search_LEFT_y) <= 1 and abs(self.current_local_z - self.entrance_search_z) <= 0.5:
				quaternion_yaw = quaternion_from_euler(0, 0, self.yaw_left_z)
				self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

				#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
				###########################################################################################
				self.current_signal = "Reached entrance search waypoint FAR/LEFT"
				###########################################################################################

				if abs(self.current_yaw-self.yaw_left_z) <= 0.2:
					self.building_scan_WPS_FLAG[3] = 1

		elif self.building_scan_WPS_FLAG[4] == 0:

			self.positionSp.header.frame_id = 'local_origin'
			self.positionSp.pose.position.x = self.entrance_search_FRONT_x
			self.positionSp.pose.position.y = self.entrance_search_LEFT_y
			self.positionSp.pose.position.z = self.entrance_search_z

			#-----------------------------------FOR SIMULATION PURPOSES ONLY------------------------------#
			###############################################################################################
			# self.verifyTAG_flag = 1
			# self.tag_info.color = 'Red'
			# self.tag_info.x = self.building_center_x - 3
			# self.tag_info.y = self.building_center_y - 5
			###############################################################################################

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
		# This function executes if green tag was not found during BuildingScan, after a complete
		# revolution. This code should make the drone go to about 10 meters, go to a known location
		# of a possible entrance, and then go to 1.5 meters yawing so that it is facing the possible
		# entrance.

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

				#-------THIS IS SO I COULD GET THE GPS LOCATION AT THIS Location FROM THE LOG FILE--------#
				###########################################################################################
				self.current_signal = "Checking possible unblocked entrance"
				###########################################################################################

				while entrance_search_counter < 80:

					#----------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE-------------#
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
		# This function will activate if the tag subscribers set self.verifyTag_flag.
		# This function will first verify that the tag was not already discovered by 
		# comparing its location with all of the previously found tags' locations. For
		# the blue and green tags, this function will then verify that it is close enough
		# to one of the known locations of the entrances. (If a blue tag for example is
		# supposedly found a long way from the buildng, it is definitely a false detection)

		at_entrance = 0
		already_detected = 0
		if self.verifyTAG_flag:
			if self.tag_color == 'Red':
				temp_x = self.redtagSp.pose.position.x
				temp_y = self.redtagSp.pose.position.y

				for i in range(len(self.red_tag.x)):
					if abs(temp_x-self.red_tag.x[i]) < 1 and abs(temp_y-self.red_tag.y[i]) < 1:
						rospy.logwarn('Tag was already detected')
						already_detected = 1
				if already_detected != 1:
					self.red_tag.x.append(temp_x)
					self.red_tag.y.append(temp_y)
					self.red_tag.total_num = self.red_tag.total_num + 1
					rospy.loginfo("Found a new red tag")

					self.ready2capturered = 1
					(self.red_tag.lat, self.red_tag.lon, _) = pm.enu2geodetic(temp_x,temp_y,0,self.home_lat,self.home_lon,0)
					self.tag_color = "None"

			elif self.tag_color == 'Blue':
				temp_x = self.bluetagSp.pose.position.x
				temp_y = self.bluetagSp.pose.position.y

				for i in range(len(self.blue_tag.x)):
					if abs(temp_x-self.blue_tag.x[i]) < 1 and abs(temp_y-self.blue_tag.y[i]) < 1:
						rospy.logwarn('Tag was already detected')
						already_detected = 1
				if already_detected != 1:
					if (abs(temp_x - self.building_entr1_x) < 1 and abs(temp_y - self.building_entr1_y) < 1):
						at_entrance = 1
					elif (abs(temp_x - self.building_entr2_x) < 1 and abs(temp_y - self.building_entr2_y) < 1):
						at_entrance = 2
					elif (abs(temp_x - self.building_entr3_x) < 1 and abs(temp_y - self.building_entr3_y) < 1):
						at_entrance = 3

					if at_entrance > 1:
						self.blue_tag.x.append(temp_x)
						self.blue_tag.y.append(temp_y)
						self.blue_tag.total_num = self.blue_tag.total_num + 1
						rospy.loginfo('Found a new blue tag')

						self.ready2captureblue = 1
						(self.blue_tag.lat, self.blue_tag.lon, _) = pm.enu2geodetic(temp_x,temp_y,0,self.home_lat,self.home_lon,0)
						self.tag_color = "None"

					elif at_entrance == 0:
						rospy.logwarn('Detected Blue Tag but it is not near an entrance so ignoring')

			elif self.tag_color == 'Green' and self.green_tag.found == 0:
				temp_x = self.greentagSp.pose.position.x
				temp_y = self.greentagSp.pose.position.y

				if (abs(temp_x - self.building_entr1_x) < 1 and abs(temp_y - self.building_entr1_y) < 1):
					at_entrance = 1
				elif (abs(temp_x - self.building_entr2_x) < 1 and abs(temp_y - self.building_entr2_y) < 1):
					at_entrance = 2
				elif (abs(temp_x - self.building_entr3_x) < 1 and abs(temp_y - self.building_entr3_y) < 1):
					at_entrance = 3

				if at_entrance > 0:
					self.green_tag.x = temp_x
					self.green_tag.y = temp_y
					self.green_tag.found = 1
					self.green_tag.drone_x = self.current_local_x
					self.green_tag.drone_y = self.current_local_y

					(_, _, self.enter_bldg_orien) = tf.transformations.euler_from_quaternion([self.positionSp.pose.orientation.x, self.positionSp.pose.orientation.y, self.positionSp.pose.orientation.z, self.positionSp.pose.orientation.w])

					rospy.loginfo('Green tag found!')

					self.ready2capturegreen = 1
					(self.green_tag.lat, self.green_tag.lon, _) = pm.enu2geodetic(temp_x,temp_y,0,self.home_lat,self.home_lon,0)
					self.tag_color = "None"
		
		self.verifyTAG_flag = 0
		self.counterTAGCb = 0

def main():
	rospy.init_node('gps_setpoint_node', anonymous=True)
	rospy.logwarn("GPS setpoints node is started")

	K = Controller()
	K.start_time = rospy.get_time()
	rospy.logwarn("START TIME")
	rospy.logwarn(K.start_time)

	########## Subscribers ##########

	# Subscriber: GPS
	rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, K.gpsCb)

	# Subscriber: Local pose
	rospy.Subscriber("mavros/local_position/pose", PoseStamped, K.localPoseCb)

	# Subscriber: Get landing state
	rospy.Subscriber("mavros/extended_state", ExtendedState, K.landStateCb)

	# Subscriber: object_localization setpoints
	rospy.Subscriber("/detected_object_3d_pos", PoseStamped, K.WorkerPoseCb)




	#------------------------------- THESE TOPICS STILL NEED TO BE DEFINED (or figured out) -------------------------------------#
	##############################################################################################################################
	#Subscriber: tag_localization setpoints
	rospy.Subscriber("/detected_red_tag", PoseStamped, K.redtagPoseCb)
	rospy.Subscriber("/detected_blue_tag", PoseStamped, K.bluetagPoseCb)
	rospy.Subscriber("/detected_green_tag", PoseStamped, K.greentagPoseCb)
	# Subscriber: Worker Images
	rospy.Subscriber("/zed(down?)/left/image_rect_color", Image, K.WorkerImageCb)
	# Subscriber: Color Tag Images:
	rospy.Subscriber("/zed(forward?)/left/image_rect_color", Image, K.TagImageCb)
	##############################################################################################################################

	


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

	# This allows you to start at a partiular state instead of always starting at LOADPACKAGE1
	if K.start_state == 'LOADPACKAGE1': 
		K.LOADPACKAGE1 = 1
	elif K.start_state == 'WAYPOINT1':
		K.WAYPOINT1 = 1
	elif K.start_state == 'WAYPOINT2':
		K.WAYPOINT2 = 1
	elif K.start_state == 'WAYPOINT3':
		K.WAYPOINT3 = 1
	elif K.start_state == 'WORKER1SEARCH':
		K.WOKRER1SEARCH = 1
	elif K.start_state == 'LOADPACKAGE2':
		K.LOADPACKAGE2 = 1
	elif K.start_state == 'GOTOBUILDING':
		K.GOTOBUILDING = 1
	elif K.start_state == 'WORKER2SEARCH':
		K.WORKER2SEARCH = 1
	elif K.start_state == 'MAPPING':
		K.MAPPING = 1

	# Set the takeoff x/y to be current position
	K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
	K.positionSp.pose.position.x = K.current_local_x
	K.positionSp.pose.position.y = K.current_local_y
	K.positionSp.pose.position.z = K.takeoff_height # Should be 1

	rospy.loginfo('Takeoff X Position')
	rospy.loginfo(K.positionSp.pose.position.x)
	rospy.loginfo('Takeoff Y Position')
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
			# Load first package
			rospy.loginfo("Loading First Package")
			K.current_state = 'LOADPACKAGE1'
			servo_pub.publish(K.RELEASE)
			rospy.sleep(0.2)
			for K.n.data in range(K.RELEASE.data,K.HOLD.data):
				if(rospy.is_shutdown()):
					continue
				servo_pub.publish(K.n)
				rospy.sleep(0.1)
			K.resetStates()
			K.TAKEOFF1 = 1
			K.current_signal = 'Package Loaded'
		# !!!!!!!!!!!!!!!!!!!!! SAFETY PILOT WILL PUT INTO OFFBOARD MODE HERE !!!!!!!!!!!!!!!!!!!!!!!!!!
		elif K.TAKEOFF1:
			K.current_state = 'TAKEOFF1'
			K.current_signal = 'Running'
			rospy.logwarn("Taking off")
			K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
			K.positionSp.pose.position.z = K.takeoff_height # Should be 1
			K.positionSp.pose.orientation.w = 1.0	# IS THIS NEEDED?
			if abs(K.current_local_z - K.takeoff_height) < 1:
				rospy.logwarn("Reached Takeoff Height")
				K.current_signal = 'Takeoff Complete'	
				K.resetStates()
				K.WAYPOINT1 = 1

		elif K.WAYPOINT1:
			K.current_state = 'WAYPOINT1'
			K.current_signal = 'Running'
			rospy.loginfo("Heading to WP1")
			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.waypoint1_x
			K.positionSp.pose.position.y = K.waypoint1_y
			K.positionSp.pose.position.z = K.waypoint1_z

			if abs(K.current_local_x - K.waypoint1_x)<= 1 and abs(K.current_local_y - K.waypoint1_y) <= 1 and abs(K.current_local_z - K.waypoint1_z) <= 0.5:   # Rules give a 3m radius from goal
				rospy.loginfo("Current position close enough to desired waypoint")
				rospy.loginfo("Reached waypoint 1")
				K.resetStates()
				K.WAYPOINT2 = 1
				K.current_signal = 'Reached WP1'

		elif K.WAYPOINT2:
			K.current_state = 'WAYPOINT2'
			K.current_signal = 'Running'
			rospy.loginfo("Heading to WP2")
			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.waypoint2_x
			K.positionSp.pose.position.y = K.waypoint2_y
			K.positionSp.pose.position.z = K.waypoint2_z

			if abs(K.current_local_x - K.waypoint2_x)<= 1 and abs(K.current_local_y - K.waypoint2_y) <= 1 and abs(K.current_local_z - K.waypoint2_z) <= 0.5:   # Rules give a 3m radius from goal
				rospy.loginfo("Current position close enough to desired waypoint")
				rospy.loginfo("Reached waypoint 2")
				K.resetStates()
				K.WAYPOINT3 = 1
				K.current_signal = 'Reached WP2'

		elif K.WAYPOINT3:
			K.current_state = 'WAYPOINT3'
			K.current_signal = 'Running'
			rospy.loginfo("Heading to WP3")
			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.waypoint3_x
			K.positionSp.pose.position.y = K.waypoint3_y
			K.positionSp.pose.position.z = K.waypoint3_z

			if abs(K.current_local_x - K.waypoint3_x)<= 1 and abs(K.current_local_y - K.waypoint3_y) <= 1 and abs(K.current_local_z - K.waypoint3_z) <= 0.5:   # Rules give a 3m radius from goal
				rospy.loginfo('Current position close enough to desired waypoint')
				rospy.loginfo('Reached waypoint 3')
				K.resetStates()
				K.WORKER1SEARCH = 1
				K.current_signal = 'Reached WP3'

		elif K.WORKER1SEARCH:
			K.current_state = 'WORKER1SEARCH'
			K.current_signal = 'Running'
			K.verifyPOI()
			rospy.loginfo("Searching for outside worker")
			if K.worker1_found_flag:
				K.current_signal = 'Outside Worker Found'
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

		elif K.DELIVERAID1:
			K.current_state = 'DELIVERAID1'
			K.current_signal = 'Running'
			rospy.loginfo('Trying to reach outside worker')
			K.positionSp.pose.position.z = K.worker1_search_z
			if (abs(K.current_local_x - K.positionSp.pose.position.x)<.1 and abs(K.current_local_y - K.positionSp.pose.position.y)<.1):
				servo_pub.publish(K.RELEASE)
				rospy.sleep(1)
				rospy.loginfo("Aid Dropped")
				K.current_signal = 'Aid Delivered'
				K.resetStates()
				#K.ENTRANCESEARCH = 1
				K.GOTOREFULE = 1

		elif K.GOTOREFULE:
			K.current_state = 'GOTOREFULE'
			K.current_signal = 'Running'
			rospy.loginfo('Headed to landing zone to refule')
			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.landing_zone_x
			K.positionSp.pose.position.y = K.landing_zone_y
			K.positionSp.pose.position.z = 10

			if abs(K.current_local_x - K.landing_zone_x)<= 1 and abs(K.current_local_y - K.landing_zone_y) <= 1:

				K.resetStates()
				K.current_signal = 'Reached Landing Zone'
				K.REFULE = 1

		elif K.REFULE:
			K.current_state = 'REFULE'
			K.current_signal = 'Running'
			rospy.loginfo('Vehicle is landing to change battery and load next aid kit')
			K.modes.setAutoLandMode()
			if K.IS_LANDED:
				K.modes.setDisarm()
				K.resetStates()
				K.current_signal = 'Landed'
				rospy.sleep(5)
				K.LOADPACKAGE2 = 1

		elif K.LOADPACKAGE2:
			K.current_state = 'LOADPACKAGE2'
			rospy.logwarn('Gripper closing')
			servo_pub.publish(K.RELEASE)
			rospy.sleep(5)
			for K.n.data in range(K.RELEASE.data,K.HOLD.data):
				if(rospy.is_shutdown()):
					continue
				servo_pub.publish(K.n)
				rospy.sleep(0.1)
			K.resetStates()
			K.TAKEOFF2 = 1
			K.current_signal = 'Package Loaded'
		# !!!!!!!!!!!!!!!!!!!!! SAFETY PILOT Will ARM AND PUT BACK INTO OFFBOARD MODE AGAIN HERE !!!!!!!!!!!!!!!!!!!!!!!!!!
		elif K.TAKEOFF2:
			K.current_state = 'TAKEOFF2'
			K.current_signal = 'Running'
			rospy.logwarn('Vehicle is taking off again')
			K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
			K.positionSp.pose.position.x = K.current_local_x 
			K.positionSp.pose.position.y = K.current_local_y
			K.positionSp.pose.position.z = K.entrance_search_z # Should be 1
			K.positionSp.pose.orientation.w = 1.0	# IS THIS NEEDED?

			quaternion_yaw = quaternion_from_euler(0, 0, K.yaw_front_z)
			K.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

			if abs(K.current_local_z - K.entrance_search_z) < .3:
				rospy.loginfo("Reached Takeoff Height")	
				K.resetStates()
				K.GOTOBUILDING = 1
				K.current_signal = 'Takeoff Complete'

		elif K.GOTOBUILDING:
			K.current_state = 'GOTOBUILDING'
			K.current_signal = 'Running'
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
					K.current_signal = 'Reached Building'

		elif K.ENTRANCESEARCH:
			K.current_state = 'ENTRANCESEARCH'
			K.current_signal = 'Running'
			K.verifyTag()
			rospy.loginfo('Searching for entrance')
			if (K.green_tag.found == 1) and (K.completed_full_rev == 1):
				rospy.loginfo('Unblocked entrance successfully found!')
				K.resetStates()
				K.ENTERBUILDING = 1
				K.current_signal = 'Entrance Search Completed'

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
					K.current_signal = 'Searching first entrance'
				elif K.entrance_number_search == 2:
					K.EntranceSearch(K.building_entr2_x,K.building_entr2_y,K.yaw_building_entr2)
					rospy.loginfo('Checking second entrance')
					K.current_signal = 'Searching second entrance'
				elif K.entrance_number_search == 3:
					rospy.loginfo('Checking third entrance')
					K.current_signal = 'Searching third entrance'
					K.EntranceSearch(K.building_entr3_x,K.building_entr3_y,K.yaw_building_entr3)
				else:
					K.entrance_number_search = 1 # So that vehicle will check first entrance again
					rospy.loginfo('Still cannot find unblocked entrance. Try again at first entrance')
			else:
				# Execute building scan until full revolution
				K.BuildingScan()

		elif K.ENTERBUILDING:
			K.current_state = 'ENTERBUILDING'
			K.current_signal = 'Running'
			rospy.loginfo("Attempting to enter building")
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

			# CODE FOR ACTUALLY ENTERING BUIDLING GOES HERE
			###############################################################################

			#if Enter Building is success
				#rospy.logwarn('Vehicle successfully')
				#K.resetStates()
				#K.current_signal = 'Entered Building'
				#K.WORKER2SEARCH = 1

			################################################################################

		elif K.WORKER2SEARCH:
			K.current_state = 'WORKER2SEARCH'
			K.current_signal = 'Running'
			rospy.loginfo('Vehicle searching for missing worker inside')

			#Execute OUTSIDE WORKER SEARCH ALGORITHM
			#if WORKER2FOUND
				#rospy.logwarn('Inside worker found!')
				#K.resetStates()
				#K.current_signal = 'Inside Worker Found'
				#K.DELIVERAID2 = 1

		elif K.DELIVERAID2:
			K.current_state = 'DELIVERAID2'
			K.current_signal = 'Running'
			rospy.loginfo('Delivering the first aid kit to inside worker')
			#Execute DELIVER AID 2
			#if Deliver Aid 2 is success
				#rospy.logwarn('Aid successfully delivered')
				#K.resetStates()
				#K.current_signal = 'Aid Delivered'
				#K.FINSIHMAPPING = 1

		elif K.MAPPING:
			# NOTES: The mapping algorithm will be based on covering enough ground in the building.
			# Currently, I believe that this will have to be done independantly from the inside
			# worker search because if executed together, they probably would require too much
			# processing power from the Jetson Tx2. I'm not sure at this point whether
			# worker search or mapping should happen first.

			K.current_state = 'MAPPING'
			K.current_signal = 'Running'
			rospy.loginfo('Finishing mapping of inside of tent')
			
			############################## THESE ARE THE MAPPING VARIABLES ###########################################
			# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
			# roslaunch.configure_logging(uuid)
			# mapping = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nvidia/catkin_ws/src/mapping_zed/launch/zed_rtabmap.launch"])
			# mapping.start() # ------>>>>>>>>>> WHEN YOU WANT TO START MAPPING TYPE THIS
			# rospy.loginfo("Mapping started")
			# mapping.shutdown() ----->>>>>>>>>> WHEN YOU WANT TO STOP MAPPING TYPE THIS
			##########################################################################################################

			#Execute MAPPING Algorithm
			#if Finish Mapping is success
				#rospy.logwarn('Building successfully mapped')
				#K.resetStates()
				#K.current_signal = 'Mapping Finished'
				#K.EXITBUILDING = 1

		elif K.EXITBUILDING:
			K.current_state = 'EXITBUILDING'
			K.current_signal = 'Running'
			rospy.loginfo('Exiting building')
			#Execute EXIT BUILDING Algorithm
			#if EXIT BUILDING is success
				#rospy.logwarn('Vehicle has successfully exited building')
				#K.resetStates()
				#K.current_signal = 'Building Exited'
				#K.GOHOME = 1

		elif K.GOHOME:
			K.current_state = 'GOHOME'
			K.current_signal = 'Running'
			rospy.loginfo('Mission Complete - Vehicle heading back to home')
			# K.modes.setReturnToHome() # Not sure if this works. It didn't work in simulation
			K.positionSp.header.frame_id = 'local_origin'
			K.positionSp.pose.position.x = K.home_x
			K.positionSp.pose.position.y = K.home_y
			K.positionSp.pose.position.z = K.takeoff_height

			if abs(K.current_local_x - K.home_x)<= 1 and abs(K.current_local_y - K.home_y) <= 1:   # Rules give a 3m radius from goal
				rospy.loginfo('Reached home')
				K.current_signal = 'Reached Home'
				K.resetStates()
				K.LAND = 1

		elif K.LAND:
			K.current_state = 'LAND'
			K.current_signal = 'Running'
			rospy.loginfo('Vehicle is landing')
			K.modes.setAutoLandMode()
			if K.IS_LANDED:
				K.modes.setDisarm()
				K.resetStates()
				K.current_signal = 'Landed'

		elif K.HOVER:
			K.current_state = 'HOVER'
			K.current_signal = 'Running'
			rospy.logwarn('Vehicle in Hover mode until something else happens')
			# NEED TO FIGURE OUT HOW TO EXIT THIS STATE

		K.VehicleNavLog()
		avoid_pub.publish(K.positionSp)
		rate.sleep()	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass