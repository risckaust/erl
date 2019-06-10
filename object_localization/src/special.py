#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion
import tf
import numpy as np
import roslaunch
import math
from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import NavSatFix, Image, LaserScan
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

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

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
		self.start_state = 'TAKEOFF1'
		self.tolerance = 0.5
		self.timertolerence=500
		self.timerr=0
		self.waypoint1_lat = 37.199920       
		self.waypoint1_lon = -5.881140 #Location of Testing Field at KAUST - a little ways down field
		self.waypoint1_alt = 3
		self.takeoff_height = 3

		#############################################################

		# GPS Fence
		self.lat_max = 22.312 #22.3076#22.3176 # Location of Testing Field at KAUST
		self.lat_min = 22.310 #22.3065#22.31725# Location of Testing Field at KAUST
		self.lon_max = 39.0955 #39.1055#39.0983 # Location of Testing Field at KAUST
		self.lon_min = 39.0949 #39.1045#39.0977 # Location of Testing Field at KAUST
		self.z_limit = 30
		
		#Waypoints GPS Coordiantes

		self.landing_zone_lat = 47.3976247 # 37.1999209 #22.3184053#47.397742 #Lat in simulation
		self.landing_zone_lon = 8.5455773  #-5.8811473 #39.1020018#8.5455939 #Lon in simulation

		self.waypoint2_lat = 37.200102
		self.waypoint2_lon = -5.881273#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint2_alt = 2.5

		self.waypoint3_lat = 37.199807#22.3070405#22.317490
		self.waypoint3_lon = -5.881159#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint3_alt = 2.5

		self.waypoint4_lat = 37.199267
		self.waypoint4_lon = -5.881092
		self.waypoint4_alt = 2.5
		
		self.waypoint5_lat = 37.199374 
		self.waypoint5_lon = -5.880975
		self.waypoint5_alt = 2.5
		

		# ----THIS INFO IS ONLY IF WE DON'T WANT TO BASE SEARCH OFF OF BUILDING GPS CENTER POINT----#
		#############################################################################################
		self.bldg_search_corner1_lat = 47.3976247 # 37.19974167#37.1997257 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		self.bldg_search_corner1_lon = 8.5455773 #-5.8809611088#-5.8809547 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		self.bldg_search_corner1_5_lat = 37.199731945#37.1997087 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		self.bldg_search_corner1_5_lon = -5.881097221#-5.8810902 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		self.bldg_search_corner2_lat = 37.19972222#37.1996935 #22.3187271# Simulation: 47.3978486 # Looking down at the play-field, this corner is the top left
		self.bldg_search_corner2_lon = -5.8812333332#-5.8812258 #39.1021202# Simulation: 8.5455894 # Looking down at the play-field, this corner is the top left
		self.bldg_search_corner2_5_lat = 37.1996055524#37.1995832 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		self.bldg_search_corner2_5_lon = -5.8812124997#-5.8812114 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		self.bldg_search_corner3_lat = 37.1994888848#37.1994767 #22.3188133# Simulation: 47.3978593 # Looking down at the play-field, this corner is the bottom left
		self.bldg_search_corner3_lon = -5.8811916662#-5.8811883 #39.1019476# Simulation: 8.5454345 # Looking down at the play-field, this corner is the bottom left
		self.bldg_search_corner3_5_lat = 37.1995069405#37.1994881 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		self.bldg_search_corner3_5_lon = -5.8810569429#-5.8810503 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		self.bldg_search_corner4_lat = 37.1995249962#37.1995062 #22.3185138# Simulation: 47.3976353 # Looking down at the play-field, this corner is the bottom right
		self.bldg_search_corner4_lon = -5.8809222196#-5.8809181 #39.1017642# Simulation: 8.5454157 # Looking down at the play-field, this corner is the bottom right
		self.bldg_search_corner4_5_lat = 37.1996333331#37.1996296 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		self.bldg_search_corner4_5_lon = -5.8809416642#-5.8809335 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		#############################################################################################
		

		#------------TESTING VALUES---------------------------------------------------#
# ----THIS INFO IS ONLY IF WE DON'T WANT TO BASE SEARCH OFF OF BUILDING GPS CENTER POINT----#
		# #############################################################################################
		# self.bldg_search_corner1_lat = 37.2001492#37.1997257 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		# self.bldg_search_corner1_lon = -5.8810322#-5.8809547 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		# self.bldg_search_corner1_5_lat = 37.2001399#37.1997087 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		# self.bldg_search_corner1_5_lon = -5.8811639#-5.8810902 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		# self.bldg_search_corner2_lat = 37.2001383#37.1996935 #22.3187271# Simulation: 47.3978486 # Looking down at the play-field, this corner is the top left
		# self.bldg_search_corner2_lon = -5.8812739#-5.8812258 #39.1021202# Simulation: 8.5455894 # Looking down at the play-field, this corner is the top left
		# self.bldg_search_corner2_5_lat = 37.2000184#37.1995832 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		# self.bldg_search_corner2_5_lon = -5.8812611#-5.8812114 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		# self.bldg_search_corner3_lat = 37.19992999#37.1994767 #22.3188133# Simulation: 47.3978593 # Looking down at the play-field, this corner is the bottom left
		# self.bldg_search_corner3_lon = -5.8812312#-5.8811883 #39.1019476# Simulation: 8.5454345 # Looking down at the play-field, this corner is the bottom left
		# self.bldg_search_corner3_5_lat = 37.1999322#37.1994881 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		# self.bldg_search_corner3_5_lon = -5.8811193#-5.8810503 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		# self.bldg_search_corner4_lat = 37.1999445#37.1995062 #22.3185138# Simulation: 47.3976353 # Looking down at the play-field, this corner is the bottom right
		# self.bldg_search_corner4_lon = -5.8810127#-5.8809181 #39.1017642# Simulation: 8.5454157 # Looking down at the play-field, this corner is the bottom right
		# self.bldg_search_corner4_5_lat = 37.2000418#37.1996296 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		# self.bldg_search_corner4_5_lon = -5.8810096#-5.8809335 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		# #############################################################################################
		# # Outside Worker Search Variables in GPS
		# self.building_center_lat = 37.2000067
		# self.building_center_lon= -5.8811325           #39.0937654	# 39.0984 Roughly the center of the KAUST field

		# self.enter_bldg_hgt = 1.5

		# self.building_entr1_lat = 37.2001428 #22.3185652#47.3976886 # Simulation value
		# self.building_entr1_lon = -5.881084 #39.1020314#8.5455664 # Simulation value
		# self.building_entr2_lat = 37.2001362 #22.3186021 # Simlulation value
		# self.building_entr2_lon = -5.8811342 #39.1020595 # Simulation value
		# self.building_entr3_lat = 37.2001302#22.3186403
		# self.building_entr3_lon = -5.8811737#39.1020712 # Simulation value

		# self.building_entr4_lat = 37.2001302
		# self.building_entr4_lon = -5.8811737

		# self.building_entr1_inside_lat = 37.2001075
		# self.building_entr1_inside_lon = -5.8810848
		# self.building_entr2_inside_lat = 37.2001044
		# self.building_entr2_inside_lon = -5.8811263
		# self.building_entr3_inside_lat = 37.2001048
		# self.building_entr3_inside_lon = -5.8811722

		# self.building_entr4_inside_lat = 37.2001048
		# self.building_entr4_inside_lon = -5.8811722
	###################################################################################################################
	#--------------------------------------- VARIABLES USED IN CODE --------------------------------------------------# 
	###################################################################################################################

		#--------------------Random Variables---------------------#
		###########################################################
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

		roslaunch.configure_logging(self.uuid)
		# #self.person = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/risc/catkin_ws/src/erl/yolo_ros_vino/launch/person_yolo_ros_vino.launch"])
		# #self.tags = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/risc/catkin_ws/src/erl/yolo_ros_vino/launch/tags_yolo_ros_vino.launch"])#########################################################################################################3

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
		self.worker1_lat = 0
		self.worker1_lon = 0
		self.worker2_lat = 0
		self.worker2_lon = 0

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
		self.home_lat = self.landing_zone_lat # Starting location is the landing_zone_lat unless updated to be set at the actual local origin (0,0)
		self.home_lon = self.landing_zone_lon # Starting location is the landing_zone_lon unless updated to be set at the actual local origin (0,0)

		self.landing_zone_x = 15 #25
		self.landing_zone_y = 0

		self.x_FenceLimit_max = 400 # 300 meters downfield from the starting position
		self.x_FenceLimit_min = -400 # 5 meters behind the starting the position 
		self.y_FenceLimit_max = 460 # 15 meters to the left of the starting position
		self.y_FenceLimit_min = -460 # 15 meters to the right of the starting position

		self.x_fence_max_warn = self.x_FenceLimit_max - 1 #29 # 295 meters downfield from the starting position
		self.x_fence_min_warn = self.x_FenceLimit_min + 1 #-4 # 5 meters in back of starting position
		self.y_fence_max_warn = self.y_FenceLimit_max - 1 #14 # 14 meters to the left of starting position
		self.y_fence_min_warn = self.y_FenceLimit_min + 1 #-14 # 14 meters to the right of the starting position
		self.z_limit_warn = 35 # Maximum height above ground that drone is allowed to go (MAX height is 40 meters - TO BE VERIFIED BY COMPETITION ORGANIZERS)

		self.building_center_x = 0 #10
		self.building_center_y = 0

		#----------------------State Machine-----------------------#
		############################################################
		self.LOADPACKAGE1 = 0
		self.TAKEOFF1 = 1
		self.WAYPOINT1 = 0
		self.WAYPOINT2 = 0
		self.WAYPOINT3 = 0
		self.WAYPOINT4 = 0
		self.WAYPOINT5 = 0
		self.BUILDING_CORNER1 = 0
		self.BUILDING_CORNER4_5 = 0
		self.BUILDING_CORNER4 = 0
		self.BUILDING_CORNER3 = 0
		self.BUILDING_CORNER2_5= 0
		self.BUILDING_CORNER2 = 0
		self.GOHOME = 0
		self.LAND = 0
		self.HOVER = 0

		#----------------------Load Aid Kit-----------------------#
		###########################################################
		self.n = UInt16()
		self.HOLD = UInt16()
		self.RELEASE = UInt16()
		self.HOLD.data = 75
		self.RELEASE.data = 45
		self.gripper_flag = True
		#----------------------Takeoff----------------------------#
		###########################################################

		
		#-------------------Waypoint Variables--------------------#
		###########################################################
		self.waypoint1_x = -20
		self.waypoint1_y = -10
		self.waypoint1_z = self.takeoff_height

		self.waypoint2_x = 0
		self.waypoint2_y = -4
		self.waypoint2_z = self.takeoff_height

		self.waypoint3_x = 0
		self.waypoint3_y = 2
		self.waypoint3_z = self.takeoff_height

		self.waypoint4_x = 0
		self.waypoint4_y =40
		self.waypoint4_z = self.takeoff_height

		self.waypoint5_x=0
		self.waypoint5_y=45
		self.waypoint5_z=self.takeoff_height



		# Used in Execute_Waypoint function
		self.waypoint_status_flag = [0, 0, 0]

		#--------------- RPLIDAR AVOIDANCE VARIABLES -------------#
		###########################################################
		self.tf_listener = tf.TransformListener()
		yaw_degrees = 0  # North
		yaw = radians(yaw_degrees)
		quaternion = quaternion_from_euler(0, 0, yaw)
		self.positionSp.pose.orientation = Quaternion(*quaternion)

		self.front=np.zeros(60)
		self.left=np.zeros(30)
		self.right=np.zeros(30)
		self.back=np.zeros(60)
		self.minfront=min(abs(self.front))
		self.minleft=min(abs(self.left))
		self.minright=min(abs(self.right))
		self.minback=min(abs(self.back))

		self.tried_left = 0
		self.tried_right = 0
		self.avoid_radius = 5

		#-----------------Worker Search Variables-----------------#
		###########################################################

		# SEARCH WAYPOINTS ARE TO BE SET BASED ON LOCATION OF 
		# BUILDING. WORKER IS PRESUMED TO BE FURTHER DOWNFIELD 
		# THAN BUILDING

		self.verifyTAG_flag = 0
		self.counterTAGCb = 0

		# For camera, taking pictures
		self.bridge = CvBridge()
		self.already_took_picture = 1 # A 1 means it has not already taken a picture
		#------------------WORKER1-------------------#
		self.worker1_search_WP1_x = -15
		self.worker1_search_WP1_y = -9
		self.worker1_search_WP2_x = -30+3
		self.worker1_search_WP2_y = -9
		self.worker1_search_WP3_x = -30+3
		self.worker1_search_WP3_y = -3
		self.worker1_search_WP4_x = -15
		self.worker1_search_WP4_y = -3
		self.worker1_search_WP5_x = -15
		self.worker1_search_WP5_y = 3
		self.worker1_search_WP6_x = -30+3
		self.worker1_search_WP6_y =	3
		self.worker1_search_WP7_x = -30+3
		self.worker1_search_WP7_y = 9
		self.worker1_search_WP8_x = -15
		self.worker1_search_WP8_y = 9
		self.worker1_search_z = 4

		self.worker1_found_flag = 0
		self.already_set_worker1_pose = 1
		self.ready2captureworker1 = 0

		self.worker1_search_WP_FLAG = -1

		self.worker1Sp = PoseStamped()

		#------------------WORKER2-------------------#
		self.worker2Sp = PoseStamped()

		self.worker2_found_flag = 0

		self.ready2captureworker2 = 0

		# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!THERE WILL BE MORE VARIABLES HERE AFTER NEW AVOIDANCE PACKAGE ADDED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		#-----------------GOTOREFULE VARIABLES------------------#
		###########################################################
		self.go_to_refule = 1
		self.closest_point_to_return = 0

		#---------------ENTRANCE SEARCH VARIABLES-----------------#
		###########################################################

		# Positive x is downfield, positive y is to the left
		self.entrance_search_corner1_x = -12.0#22#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner1_y = 8.0#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner1_5_x = 0#22#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner1_5_y = 0#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner2_x = 12#22#11.5 # Looking down at the play-field, this corner is the top left
		self.entrance_search_corner2_y = 8#12#11.5 # Looking down at the play-field, this corner is the top left
		self.entrance_search_corner2_5_x = 12#22#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner2_5_y = 15#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner3_x = 12#2#-11.5 # Looking down at the play-field, this corner is the bottom left
		self.entrance_search_corner3_y = 35#11.5 # Looking down at the play-field, this corner is the bottom left
		self.entrance_search_corner3_5_x = 0.0#22#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner3_5_y = 0#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner4_x = -12#2#-11.5 # Looking down at the play-field, this corner is the bottom right
		self.entrance_search_corner4_y = 35.0#-12#-11.5 # Looking down at the play-field, this corner is the bottom right
		self.entrance_search_corner4_5_x = -12#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner4_5_y = 15#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_z = 0

		# Yaw angles so that drone will always face a side of the building. THIS ASSUMES CORNERS ARE PLACED ACCORDING TO GIVEN CONVENTION
		self.yaw_top_z = atan2((self.entrance_search_corner4_y-self.entrance_search_corner1_y),(self.entrance_search_corner4_x-self.entrance_search_corner1_x))
		self.yaw_right_z = atan2((self.entrance_search_corner2_y-self.entrance_search_corner1_y),(self.entrance_search_corner2_x-self.entrance_search_corner1_x))
		self.yaw_bottom_z = atan2((self.entrance_search_corner1_y-self.entrance_search_corner4_y),(self.entrance_search_corner1_x-self.entrance_search_corner4_x))
		self.yaw_left_z = atan2((self.entrance_search_corner1_y-self.entrance_search_corner2_y),(self.entrance_search_corner1_x-self.entrance_search_corner2_x))

		self.building_entr1_x = 11.5#22
		self.building_entr1_y = -6#-12
		self.building_entr2_x = 11.5#22
		self.building_entr2_y = -3
		self.building_entr3_x = 11.5#22
		self.building_entr3_y = 3#12

		self.building_entr4_x = 11.5#22
		self.building_entr4_y = 6#-12

		self.building_entr1_inside_x = 9
		self.building_entr1_inside_y = -6
		self.building_entr2_inside_x = 9
		self.building_entr2_inside_y = -3
		self.building_entr3_inside_x = 9
		self.building_entr3_inside_y = 3

		self.building_entr4_inside_x = 9#22
		self.building_entr4_inside_y = 6#-12

		self.yaw_building_entr1 = self.yaw_top_z
		self.yaw_building_entr2 = self.yaw_top_z
		self.yaw_building_entr3 = self.yaw_top_z

		self.yaw_building_entr4 = self.yaw_top_z

		self.current_side = 'Top'

		# Variables to handle transitions within the state (i.e. reaching the updated yaw position before actually heading to the next corner)
		self.entrance_number_search = 1
		self.building_scan_WP_FLAG = 1.5
		self.entrance_search_WPS_FLAG = 0

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

		self.unblocked_entrance = 0
		self.unblocked_entrance_x = 0
		self.unblocked_entrance_y = 0

		self.entrance_search_counter = 0

		#----------------ENTER BUILDING VARIABLES-----------------#
		###########################################################

		# Variables to handle transitions within the state
		self.path_flag = 'Center'
		self.avoidancepar = 1
		self.current_yaw = 0
		self.target_yaw = 0
		self.avoidancepar=1
		self.dx=0
		self.dy=0
		self.building_param = [0, 0, 0]
		self.current_r =2

		self.target_y = 0
		self.target_x = 0

		self.avoiding_obstacle = 0
		self.stay_current_x = 0
		self.stay_current_y = 0

		self.avoid_x = 0
		self.avoid_y = 0
		self.flagcount=0
		self.righttemp=0
		self.lefttemp=0
		self.fartemp=0
		self.neartemp=0
		self.mappingdirection=1
		self.inverse_flag = 0

		self.indoor_hgt = 2.5

		self.enter_bldg_flag = 0
		self.exit_bldg_flag = 0

		####FOR TEST ONLY######
		self.tagflag =1
		##########################


		self.enter_bldg_right = np.zeros(90)
		self.enter_bldg_left = np.zeros(90)
		self.enter_bldg_right_min = min(self.enter_bldg_right)
		self.enter_bldg_left_min = min(self.enter_bldg_left)

		self.avoid_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
		self.rate = rospy.Rate(10.0)
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

	#---------------------------------RPLIDAR AVOIDANCE-----------------------------------#
	#######################################################################################
	def rangessCb(self, msg):
		if msg is not None:
			self.front = msg.ranges[145:204]#Grabs values as [350:359, 0:10] going CCW rotation
			#print(self.front)
			self.left = msg.ranges[255:284]
			#print(self.left)
			self.right = msg.ranges[75:104]
			#print(self.right)
			self.back = [msg.ranges[330:359],msg.ranges[0:29]]

			self.enter_bldg_right = msg.ranges[90:180]
			self.enter_bldg_left = msg.ranges[180:270]
			self.all360 = msg.ranges
	#######################################################################################

	###################################################################################################################################


	def setWayoints_and_Fence(self):
		# Converts all GPS locations to the local frame.

		# rospy.loginfo('Current lat')
		# rospy.loginfo(self.current_lat)

		# rospy.loginfo('Current lon')
		# rospy.loginfo(self.current_lon)

		# rospy.loginfo('Current alt')
		# rospy.loginfo(self.current_alt)

		# rospy.loginfo('Current x')
		# rospy.loginfo(self.current_local_x)

		# rospy.loginfo('Current y')
		# rospy.loginfo(self.current_local_y)

		# rospy.loginfo('Current z')
		# rospy.loginfo(self.current_local_z)

		# #UNCOMMENT THIS CODE FOR GPS USE
		# #####################################################################################################################################
		#zebo (self.home_lat, self.home_lon, _) = pm.enu2geodetic(0,0,0,self.current_lat,self.current_lon,0)

		# # self.x_FenceLimit_max, self.y_FenceLimit_max, _ = pm.geodetic2enu(self.lat_max, self.lon_max, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# # self.x_FenceLimit_min, self.y_FenceLimit_min, _ = pm.geodetic2enu(self.lat_min, self.lon_min, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# # rospy.loginfo('Successfully set fence limit maxes and mins')

		# # self.x_fence_max_warn = self.x_FenceLimit_max - 2
		# # self.x_fence_min_warn = self.x_FenceLimit_min + 2
		# # self.y_fence_max_warn = self.y_FenceLimit_max - 2
		# # self.y_fence_min_warn = self.y_FenceLimit_min + 2
		# # self.z_limit_warn = self.z_limit - .5

		# ## Should be very close to (0,0) in local coordinate frame
		# self.landing_zone_x, self.landing_zone_y, _ = pm.geodetic2enu(self.landing_zone_lat, self.landing_zone_lon, self.z_limit, self.current_lat, self.current_lon, self.current_alt)

		self.waypoint1_x, self.waypoint1_y, _ = pm.geodetic2enu(self.waypoint1_lat, self.waypoint1_lon, self.takeoff_height, self.current_lat, self.current_lon, self.takeoff_height)

		# #####################################################################################################################################

		# rospy.loginfo('x Fence Max Warn')
		# rospy.loginfo(self.x_fence_max_warn)
		# rospy.loginfo('x Fence Min Warn')
		# rospy.loginfo(self.x_fence_min_warn)
		# rospy.loginfo('y Fence Max Warn')
		# rospy.loginfo(self.y_fence_max_warn)
		# rospy.loginfo('y Fence Min Warn')
		# rospy.loginfo(self.y_fence_min_warn)
		# rospy.loginfo('z Height Limit Warn')
		# rospy.loginfo(self.z_limit_warn)

		# rospy.logwarn('Waypoint x')
		# rospy.logwarn(self.waypoint1_x)
		# rospy.logwarn('Waypoint y')
		# rospy.logwarn(self.waypoint1_y)
		# rospy.logwarn('Waypoint z')
		# rospy.logwarn(self.waypoint1_z)
		# rospy.loginfo('Trying to test validity of waypoints')
		#self.isValidWaypoint(self.waypoint1_x,self.waypoint1_y,self.waypoint1_z) # Test whether waypoint 1 is within fence
		#self.isValidWaypoint(self.waypoint2_x,self.waypoint2_y,self.waypoint2_z) # Test whether waypoint 2 is within fence
		#self.isValidWaypoint(self.waypoint3_x,self.waypoint3_y,self.waypoint3_z) # Test whether waypoint 3 is within fence
		#self.isValidWaypoint(self.waypoint4_x,self.waypoint4_y,self.waypoint4_z) # Test whether waypoint 4 is within fence

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
		self.WAYPOINT4 = 0
		self.WAYPOINT5= 0
		self.BUILDING_CORNER1 = 0
		self.BUILDING_CORNER4_5 = 0
		self.BUILDING_CORNER4 = 0
		self.BUILDING_CORNER3 = 0
		self.BUILDING_CORNER2_5= 0
		self.BUILDING_CORNER2 = 0
		self.GOHOME = 0
		self.LAND = 0
		self.HOVER = 0

	def VehicleNavLog(self):
		# Drone navigation logger that activates either after each second, or if the
		# current state has changed. Also changes the name of the file to write to
		# after each start of the python script (unless using simulation because the
		# simulation resets time after each start whereas in actual testing, rospy.get_time()
		# is the amount of seconds passed from some date/time in 1970.)
		self.timer_sec = round(rospy.get_time() - self.start_time)
		if (self.previous_timer_sec != self.timer_sec) or (self.current_state != self.previous_cycle_state) or (self.current_signal != self.previous_cycle_signal):
			#name = "/home/risc/catkin_ws/src/object_localization/src/VehicleNavData_" + str(self.start_time) + ".txt"
			name = "/home/risc/logggg" + str(self.start_time) + ".txt"
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


			
	def Execute_Waypoint(self,waypoint_x,waypoint_y,waypoint_z,desired_yaw,t):

		done = 0
		yaw_condition = 0

		if desired_yaw == -1000: # desired_yaw was not specified. Set yaw towards next waypoint.
			desired_yaw = atan2((waypoint_y-self.current_local_y),(waypoint_x-self.current_local_x))

		self.positionSp.header.frame_id = 'local_origin'
		quaternion_yaw = quaternion_from_euler(0, 0, desired_yaw)
		self.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

		# Yaw condition depending on if desired yaw is close to 3.14 or -3.14 which would cause issues when it gets close because sign changes
		if abs(pi-abs(desired_yaw)) < 10*pi/180:
			yaw_condition = abs(abs(self.current_yaw)-desired_yaw)<.1
		else:
			yaw_condition = abs(self.current_yaw-desired_yaw)<.1

		if yaw_condition:
			self.positionSp.pose.position.x = waypoint_x
			self.positionSp.pose.position.y = waypoint_y
			self.positionSp.pose.position.z = waypoint_z

			if abs(self.current_local_x - waypoint_x)<= t and abs(self.current_local_y - waypoint_y) <= t and abs(self.current_local_z - waypoint_z) <= 0.5:
				rospy.loginfo("Current position close enough to desired waypoint")
				#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
				###########################################################################################
				self.current_signal = "Reached waypoint"
				###########################################################################################

				done = 1

		return done




def main():
	rospy.init_node('gps_setpoint_node', anonymous=True)
	rospy.logwarn("GPS setpoints node is started")

	K = Controller()
	# ff=fcuModes()
	# ff.setArm()
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

	####################################################################################################
	#--------------------------------------OUTDOOR AVOIDANCE-------------------------------------------#
	####################################################################################################
	
	# USE /scan FOR ACTUAL TESTING!!! USE /laser/scan FOR SIMULATION
	##############################################################################################################################################################################################################
	####################################################################################################

	########## Publishers ##########

	# Publisher: PositionTarget
	# avoid_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
	# rate = rospy.Rate(10.0)
	# Do initial checks
	while (K.current_lat*K.current_lon*K.current_alt) == 0 and not rospy.is_shutdown():

		rospy.loginfo('Waiting for current gps location to execute setWaypoints_and_FenceCb') # Initializes waypoints and fence in local x,y,z and checks to see if they make sense
		K.rate.sleep()

	K.setWayoints_and_Fence()

	#K.resetStates()

	# We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
	k=0
	while k<10:
		K.avoid_pub.publish(K.positionSp)
		K.rate.sleep()
		k = k+1

	# K.modes.setOffboardMode()
	# K.modes.setArm()

	# This allows you to start at a partiular state instead of always starting at LOADPACKAGE1

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
		#if (K.isTooCloseToFence()== True):
		#	rospy.logwarn('Changing to Hover mode because vehicle was too close to boundary')
		#	K.resetStates()
		#	K.HOVER = 1
			# PUT HOVER POINTS HERE SO THAT THEY DON'T CONSTANTLY GET UPDATED AND THE VEHICLE DRIFTS
		#	K.positionSp.pose.position.x = K.current_local_x 
		#	K.positionSp.pose.position.y = K.current_local_y 
		#	K.positionSp.pose.position.z = K.current_local_z
		####################################################	
	
    	

		if K.TAKEOFF1:
			print("IM HERE")
			K.current_state = 'TAKEOFF1'
			K.current_signal = 'Running'
			rospy.logwarn("Taking off")
			K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
			K.positionSp.pose.position.z = K.takeoff_height # Should be 1
			K.positionSp.pose.position.x = K.current_local_x
			K.positionSp.pose.position.y = K.current_local_y
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
			angle = atan2((K.waypoint1_y-K.current_local_y),(K.waypoint1_x-K.current_local_x))			
			
						
			waypoint1_is_done = K.Execute_Waypoint(K.waypoint1_x, K.waypoint1_y, K.takeoff_height, -1000,K.tolerance)

			if waypoint1_is_done:
				rospy.loginfo("Current position close enough to desired waypoint")
				rospy.loginfo("Reached waypoint 1")
				K.resetStates()
				K.HOVER = 1
				K.current_signal = 'Reached WP1'

		elif K.HOVER:
			K.current_state = 'HOVER'
			K.current_signal = 'Running'
			rospy.logwarn('Vehicle in Hover mode until something else happens')

			waypoint1_is_done = K.Execute_Waypoint(K.waypoint1_x, K.waypoint1_y, K.takeoff_height, -1000,K.tolerance)

			K.timerr = K.timerr + 1

			if K.timerr > K.timertolerence:
				rospy.loginfo("Current position close enough to desired waypoint")
				rospy.loginfo("Reached waypoint 1")
				K.resetStates()
				K.LAND = 1
				K.current_signal = 'Reached WP1'
				 

		elif K.LAND:
			K.current_state = 'LAND'
			K.current_signal = 'Running'
			rospy.loginfo('Vehicle is landing')
			K.modes.setAutoLandMode()
			if K.IS_LANDED:
				K.modes.setDisarm()
				K.resetStates()
				K.current_signal = 'Landed'


		K.VehicleNavLog()		
		rospy.loginfo("Heading to x")
		rospy.loginfo(K.positionSp.pose.position.x)
		rospy.loginfo("Heading to y")
		rospy.loginfo(K.positionSp.pose.position.y)
		K.avoid_pub.publish(K.positionSp)
		K.rate.sleep()	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
