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
class TEMPORARY_TAG_DETECT: # This was only for simulation
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
		# self.start_state = 'LOADPACKAGE1'
		# self.start_state = 'WAYPOINT1'
		# self.start_state = 'WAYPOINT2'
		# self.start_state = 'WAYPOINT3'
		# self.start_state = 'WAYPOINT4'
		# self.start_state = 'WORKER1SEARCH'
		# self.start_state = 'LOADPACKAGE2'
		# self.start_state = 'GOTOBUILDING'
		self.start_state = 'TAGSEARCH' # START IN THIS STATE
		# self.start_state = 'ENTERBUILDING'
		# self.start_state = 'WORKER2SEARCH'
		# self.start_state = 'MAPPING'

		#############################################################

		# GPS Fence
		self.lat_max = 22.312 #22.3076#22.3176 # Location of Testing Field at KAUST
		self.lat_min = 22.310 #22.3065#22.31725# Location of Testing Field at KAUST
		self.lon_max = 39.0955 #39.1055#39.0983 # Location of Testing Field at KAUST
		self.lon_min = 39.0949 #39.1045#39.0977 # Location of Testing Field at KAUST
		self.z_limit = 30
		
		#Waypoints GPS Coordiantes

		self.landing_zone_lat = 37.1999209 #22.3184053#47.397742 #Lat in simulation
		self.landing_zone_lon = -5.8811473 #39.1020018#8.5455939 #Lon in simulation

		self.waypoint1_lat = 37.200084        
		self.waypoint1_lon = -5.881067# Location of Testing Field at KAUST - a little ways down field
		self.waypoint1_alt = 2.8

		self.waypoint2_lat = 37.200102
		self.waypoint2_lon = -5.881273#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint2_alt = 2.8

		self.waypoint3_lat = 37.199807#22.3070405#22.317490
		self.waypoint3_lon = -5.881159#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint3_alt = 2.8

		self.waypoint4_lat = 37.199807
		self.waypoint4_lon = -5.881159
		self.waypoint4_alt = 2.8
		
		self.waypoint5_lat = 37.19972222 
		self.waypoint5_lon = -5.8812333332
		self.waypoint5_alt = 2.8

		self.waypoint6_lat = 37.199267
		self.waypoint6_lon = -5.881092
		self.waypoint6_alt = 2.8

		self.waypoint7_lat = 37.199374
		self.waypoint7_lon = -5.880975
		self.waypoint7_alt = 2.8

		self.waypoint8_lat = 37.19974167
		self.waypoint8_lon = -5.8809611088
		self.waypoint8_alt = 2.8
		
		self.worker1_search_WP1_lat = 37.1994761 # Simulation value
		self.worker1_search_WP1_lon = -5.8809624 # Simulation value
		self.worker1_search_WP2_lat =  37.19913333# Simulation value
		self.worker1_search_WP2_lon =  -5.880874997# Simulation value
		self.worker1_search_WP3_lat =  37.19912778# Simulation value
		self.worker1_search_WP3_lon =  -5.8809361086# Simulation value
		self.worker1_search_WP4_lat = 37.1994861 # Simulation value
		self.worker1_search_WP4_lon = -5.8810259 # Simulation value
		self.worker1_search_WP5_lat = 37.1994737 # Simulation value
		self.worker1_search_WP5_lon = -5.8810968 # Simulation value
		self.worker1_search_WP6_lat =  37.19911944# Simulation value
		self.worker1_search_WP6_lon =  -5.8810111092# Simulation value
		self.worker1_search_WP7_lat =  37.19911111# Simulation value
		self.worker1_search_WP7_lon =  -5.8810777764# Simulation value
		self.worker1_search_WP8_lat = 37.1994672 # Simulation value
		self.worker1_search_WP8_lon = -5.8811756 # Simulation value

		# ----THIS INFO IS ONLY IF WE DON'T WANT TO BASE SEARCH OFF OF BUILDING GPS CENTER POINT----#
		#############################################################################################
		self.bldg_search_corner1_lat = 37.19973746#37.1997257 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		self.bldg_search_corner1_lon = -5.88090335#-5.8809547 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
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
		self.bldg_search_corner4_lat = 37.1995311#37.1995062 #22.3185138# Simulation: 47.3976353 # Looking down at the play-field, this corner is the bottom right
		self.bldg_search_corner4_lon = -5.88092044#-5.8809181 #39.1017642# Simulation: 8.5454157 # Looking down at the play-field, this corner is the bottom right
		self.bldg_search_corner4_5_lat = 37.1996333331#37.1996296 #22.3184444# Simulation: 47.3976247 # Looking down at the play-field, this corner is the top right
		self.bldg_search_corner4_5_lon = -5.8809416642#-5.8809335 #39.1019761# Simulation: 8.5455773	# Looking down at the play-field, this corner is the top right
		#############################################################################################
		
		# # Outside Worker Search Variables in GPS
		self.building_center_lat = 37.1996174
		self.building_center_lon= -5.8810714           #39.0937654	# 39.0984 Roughly the center of the KAUST field

		self.enter_bldg_hgt = 1.5

		self.building_entr1_lat = 37.1997350 #22.3185652#47.3976886 # Simulation value
		self.building_entr1_lon = -5.8810680 #39.1020314#8.5455664 # Simulation value
		self.building_entr2_lat = 37.1997350 #22.3186021 # Simlulation value
		self.building_entr2_lon = -5.8810680 #39.1020595 # Simulation value
		self.building_entr3_lat = 37.1997075#22.3186403
		self.building_entr3_lon = -5.8811240#39.1020712 # Simulation value

		self.building_entr4_lat = 37.1997287
		self.building_entr4_lon = -5.8811944

		self.building_entr1_inside_lat = 37.1996912
		self.building_entr1_inside_lon = -5.8810747
		self.building_entr2_inside_lat = 37.1996912
		self.building_entr2_inside_lon = -5.8810747
		self.building_entr3_inside_lat = 37.1996721
		self.building_entr3_inside_lon = -5.8810939

		self.building_entr4_inside_lat = 37.199670
		self.building_entr4_inside_lon = -5.88117

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
		self.mapping = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/risc/catkin_ws/src/erl/my_package/launch/miniMission.launch"])
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
		self.TAKEOFF1 = 0
		self.WAYPOINT1 = 0
		self.WAYPOINT2 = 0
		self.WAYPOINT3 = 0
		self.WAYPOINT4 = 0
		self.WORKER1SEARCH = 0
		self.DELIVERAID1 = 0
		self.GOTOREFULE = 0
		self.REFULE = 0
		self.LOADPACKAGE2 = 0
		self.TAKEOFF2 = 0
		self.GOTOBUILDING = 0
		self.TAGSEARCH = 0
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
		self.HOLD.data = 75
		self.RELEASE.data = 45
		self.gripper_flag = True

		#----------------------Takeoff----------------------------#
		###########################################################
		self.takeoff_height = 1.5
		
		#-------------------Waypoint Variables--------------------#
		###########################################################
		self.waypoint1_x = 10
		self.waypoint1_y = 2
		self.waypoint1_z = 2

		self.waypoint2_x = 15
		self.waypoint2_y = 0
		self.waypoint2_z = 2

		self.waypoint3_x = 10
		self.waypoint3_y = 5
		self.waypoint3_z = 2

		self.waypoint4_x = 5
		self.waypoint4_y = 0
		self.waypoint4_z = 2

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

		self.deliver_aid1_z = 3

		self.worker1_found_flag = 0
		self.already_set_worker1_pose = 1
		self.ready2captureworker1 = 1

		self.worker1_search_WP_FLAG = -1

		self.worker1Sp = PoseStamped()

		#------------------WORKER2-------------------#
		self.worker2Sp = PoseStamped()

		self.worker2_found_flag = 0

		self.ready2captureworker2 = 1

		# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!THERE WILL BE MORE VARIABLES HERE AFTER NEW AVOIDANCE PACKAGE ADDED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		#-----------------GOTOREFULE VARIABLES------------------#
		###########################################################
		self.go_to_refule = 1
		self.closest_point_to_return = 0

		#---------------ENTRANCE SEARCH VARIABLES-----------------#
		###########################################################

		# Positive x is downfield, positive y is to the left
		self.entrance_search_corner1_x = 11.5#22#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner1_y = -11.5#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner1_5_x = 11.5#22#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner1_5_y = 0#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner2_x = 11.5#22#11.5 # Looking down at the play-field, this corner is the top left
		self.entrance_search_corner2_y = 11.5#12#11.5 # Looking down at the play-field, this corner is the top left
		self.entrance_search_corner2_5_x = 0#22#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner2_5_y = 11.5#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner3_x = -11.5#2#-11.5 # Looking down at the play-field, this corner is the bottom left
		self.entrance_search_corner3_y = 11.5#12#11.5 # Looking down at the play-field, this corner is the bottom left
		self.entrance_search_corner3_5_x = -11.5#22#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner3_5_y = 0#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner4_x = -11.5#2#-11.5 # Looking down at the play-field, this corner is the bottom right
		self.entrance_search_corner4_y = -11.5#-12#-11.5 # Looking down at the play-field, this corner is the bottom right
		self.entrance_search_corner4_5_x = 0#22#11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_corner4_5_y = -11.5#-12#-11.5 # Looking down at the play-field, this corner is the top right
		self.entrance_search_z = 3.3

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

	def WorkerPoseCb(self, msg):
		# Takes in possible worker location given from /detected_object_3d_pos topic and then sets the verifyPOI_flag to check if it was
		# actually a person. If it is, the verifyPOI_flag will set self.worker1_found_flag = 1 and then this function will update the
		# location of the person so that it can deliver the package accurately within ~2 meters of the mannequin

		if msg is not None and (self.WORKER1SEARCH or self.DELIVERAID1 or self.WAYPOINT1 or self.WAYPOINT2 or self.WAYPOINT3 or self.WAYPOINT4 or self.WORKER2SEARCH):
			if self.already_set_worker1_pose and (self.WORKER1SEARCH or self.WAYPOINT1 or self.WAYPOINT2 or self.WAYPOINT3 or self.WAYPOINT4):
				rospy.logwarn('Worker 1 Found!')             			
				self.worker1_found_flag = 1
				self.worker1Sp = msg
				self.worker1Sp.header.frame_id='local_origin'
				# self.positionSp.pose.position.z = self.worker1_search_z
				self.already_set_worker1_pose = 0
			elif (self.DELIVERAID1):
				rospy.logwarn('Worker 1 Found!')             			
				self.worker1_found_flag = 1
				self.worker1Sp = msg
				self.worker1Sp.header.frame_id='local_origin'
			elif (self.WORKER2SEARCH):
				rospy.logwarn('Worker 2 Found!')             			
				self.worker2_found_flag = 1
				self.worker2Sp = msg
				self.worker2Sp.header.frame_id='local_origin'
							
			
			# NOT SURE WHY WE DID THIS:
			# self.worker1Sp.pose.position.z = self.worker1_search_z

			(self.worker1_lat, self.worker1_lon, _) = pm.enu2geodetic(self.worker1Sp.pose.position.x,self.worker1Sp.pose.position.y,0,self.home_lat,self.home_lon,0)
			(self.worker2_lat, self.worker2_lon, _) = pm.enu2geodetic(self.worker2Sp.pose.position.x,self.worker2Sp.pose.position.y,0,self.home_lat,self.home_lon,0)
		# if msg is not None and self.worker1_found_flag:
		# 	self.worker1Sp = msg
		# 	self.worker1Sp.header.frame_id='local_origin'

		# 	self.positionSp.pose.position.z = self.worker1_search_z
		# 	# NOT SURE WHY WE DID THIS:
		# 	# self.worker1Sp.pose.position.z = self.worker1_search_z			

		# 	(self.worker1_lat, self.worker1_lon, _) = pm.enu2geodetic(self.worker1Sp.pose.position.x,self.worker1Sp.pose.position.y,0,self.home_lat,self.home_lon,0)

	################# THE SUBSCRIBER TOPIC FOR THESE FUNCTIONS STILL NEEDS TO BE PUBLISHED TO IN A DIFFERENT TAG_POSITION_LOCALIZATION
	def redtagPoseCb(self,msg):
		# Called if a red tag was thought to be seen. Then will hand off testing to verifyTag()
		if msg is not None:
			self.redtagSp = msg
			self.redtagSp.header.frame_id='local_origin'
			self.verifyTAG_flag = 1
			self.tag_color = 'Red'
			rospy.logwarn("Found Red Tag")

	def bluetagPoseCb(self,msg):
		# Called if a blue tag was thought to be seen. Then will hand off testing to verifyTag()
		if msg is not None:
			self.bluetagSp = msg
			self.bluetagSp.header.frame_id='local_origin'
			self.verifyTAG_flag = 1
			self.tag_color = 'Blue'

	def greentagPoseCb(self,msg):
		# Called if a green tag was thought to be seen. Then will hand off testing to verifyTag()
		if msg is not None:
			self.greentagSp = msg
			self.greentagSp.header.frame_id='local_origin'
			self.verifyTAG_flag = 1
			self.tag_color = 'Green'
	###################################################################################################################################

	def WorkerImageCb(self, msg):
		# This function will capture a ZED camera shot only if it is currently in a workersearch state and if a worker is found.
		# It will also log the information in the ObjectOfInterest...txt file (The file should change name each time this entire
		# script is run again)
		if msg is not None and ((self.ready2captureworker1*self.DELIVERAID1>0) or (self.ready2captureworker2*self.DELIVERAID2)>0):
			print("Received an image!")
			self.timer_sec = round(rospy.get_time() - self.start_time)
			try:
			# Convert your ROS Image message to OpenCV2
				cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			except CvBridgeError, e:
				print(e)
			else:
				# Save your OpenCV2 image as a jpeg
				logname = "/home/risc/catkin_ws/src/erl/Logging_Data/ObjectOfInterest_" + str(self.start_time) + ".txt"
				hs = open(logname,"a")
				# s = "\nTime Stamp (sec):\n"
				s = str(self.timer_sec) + "\n"
				if self.DELIVERAID1:
					picname = "/home/risc/catkin_ws/src/erl/Logging_Data/Worker1_" + str(self.start_time) + ".jpeg"
					cv2.imwrite(picname, cv2_img)
					# s = s + "\nTarget ID:\nOutside Worker\nTarget Position (Lat, Lon):\n"
					# s = s + "(" + str(self.worker1_lat) + ", " + str(self.worker1_lon) + ")\n"
					s = s + 'Target ID: Worker 1\n'
					s = s + str(self.worker1_lat) + "\n"
					s = s + str(self.worker1_lon) + "\n"
				elif self.DELIVERAID2:
					picname = "/home/risc/catkin_ws/src/erl/Logging_Data/Worker2_" + str(self.start_time) + ".jpeg"
					cv2.imwrite(picname, cv2_img)
					# s = s + "\nTarget ID:\nInside Worker\nTarget Position (Lat, Lon):\n"
					# s = s + "(" + str(self.worker2_lat) + ", " + str(self.worker2_lon) + ")\n"
					s = s + 'Target ID: Worker 2\n'
					s = s + str(self.worker2_lat) + "\n"
					s = s + str(self.worker2_lon) + "\n"
				hs.write(s)
				hs.close()

	def TagImageCb(self, msg):
		# Similar to WorkerImageCb. Will take an image of the tag and will log the tag as a particular color
		# depending on which flag (self.ready2capturered, self.ready2captureblue, or self.ready2capturegreen) 
		# was set by verifyTag(). Will only activate for blue and green tags if in TAGSEARCH state and one of the tags is set.
		# Will always work for red tag regardless of the state.
		if msg is not None and ((self.ready2capturered + self.ready2captureblue + self.ready2capturegreen)>0):
			print("Received an image!")
			self.timer_sec = round(rospy.get_time() - self.start_time)
			try:
			# Convert your ROS Image message to OpenCV2
				cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			except CvBridgeError, e:
				print(e)
			else:
				# Save your OpenCV2 image as a jpeg
				logname = "/home/risc/catkin_ws/src/erl/Logging_Data/ObjectOfInterest_" + str(self.start_time) + ".txt"
				hs = open(logname,"a")
				if self.ready2capturered:
					name = "/home/risc/catkin_ws/src/erl/Logging_Data/Red" + str(self.red_tag.total_num) + "_" + str(self.start_time) + ".jpeg"
					cv2.imwrite(name, cv2_img)
					# r = "\nTime Stamp (sec):\n"
					r = str(self.timer_sec) + "\n"
					# r = r + "\nTarget ID:\nRed Tag\nTarget Position (Lat, Lon):\n"
					# r = r + "(" + str(self.red_tag.lat) + ", " + str(self.red_tag.lon) + ")\n"
					r = r + 'Target ID: Red Tag\n'
					r = r + str(self.red_tag.lat) + "\n"
					r = r + str(self.red_tag.lon) + "\n"
					hs.write(r)
					self.ready2capturered = 0
				if self.ready2captureblue:
					name = "/home/risc/catkin_ws/src/erl/Logging_Data/Blue" + str(self.blue_tag.total_num) + "_" + str(self.start_time) + ".jpeg"
					cv2.imwrite(name, cv2_img)
					# b = "\nTime Stamp (sec):\n"
					b = str(self.timer_sec) + "\n"
					# b = b + "\nTarget ID:\nBlue Tag\nTarget Position (Lat, Lon):\n"
					# b = b + "(" + str(self.blue_tag.lat) + ", " + str(self.blue_tag.lon) + ")\n"
					b = b + 'Target ID: Blue Tag\n'
					b = b + str(self.blue_tag.lat) + "\n"
					b = b + str(self.blue_tag.lon) + "\n"
					hs.write(b)
					self.ready2captureblue = 0
				if self.ready2capturegreen:
					name = "/home/risc/catkin_ws/src/erl/Logging_Data/Green_" + str(self.start_time) + ".jpeg"
					cv2.imwrite(name, cv2_img)
					# g = "\nTime Stamp (sec):\n"
					g = str(self.timer_sec) + "\n"
					# g = g + "\nTarget ID:\nGreen Tag\nTarget Position (Lat, Lon):\n"
					# g = g + "(" + str(self.green_tag.lat) + ", " + str(self.green_tag.lon) + ")\n"
					g = g + 'Target ID: Green Tag\n'
					g = g + str(self.green_tag.lat) + "\n"
					g = g + str(self.green_tag.lon) + "\n"
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
			#name = "/home/risc/catkin_ws/src/object_localization/src/VehicleNavData_" + str(self.start_time) + ".txt"
			name = "/home/risc/catkin_ws/src/erl/Logging_Data/logging" + str(self.start_time) + ".txt"
			hs = open(name,"a")
			# s = "\nTime Stamp (sec):\n"
			s = str(self.timer_sec) + "\n"
			# s = s + "\nVehicle Location (Lat, Lon):\n"
			s = s + str(self.current_lat) + "\n"
			s = s + str(self.current_lon) + "\n"
			# s = s + "Current Heading (0=North, 90=East):\n"
			s = s + str(self.current_yaw_deg) + "\n"
			# s = s + "\nCurrent State:\n"
			s = s + str(self.current_local_z) + '\n'

			s = s + self.current_state + "\n"
			# s = s + "\nCurrent Signal:\n"
			s = s + self.current_signal + "\n"
			hs.write(s)
			hs.close()
			self.previous_timer_sec = round(rospy.get_time() - self.start_time)
			self.previous_cycle_state = self.current_state
			self.previous_cycle_signal = self.current_signal

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
		(self.home_lat, self.home_lon, _) = pm.enu2geodetic(0,0,0,self.current_lat,self.current_lon,0)

		# # self.x_FenceLimit_max, self.y_FenceLimit_max, _ = pm.geodetic2enu(self.lat_max, self.lon_max, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# # self.x_FenceLimit_min, self.y_FenceLimit_min, _ = pm.geodetic2enu(self.lat_min, self.lon_min, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		# # rospy.loginfo('Successfully set fence limit maxes and mins')

		# # self.x_fence_max_warn = self.x_FenceLimit_max - 2
		# # self.x_fence_min_warn = self.x_FenceLimit_min + 2
		# # self.y_fence_max_warn = self.y_FenceLimit_max - 2
		# # self.y_fence_min_warn = self.y_FenceLimit_min + 2
		# # self.z_limit_warn = self.z_limit - .5

		# ## Should be very close to (0,0) in local coordinate frame
		self.landing_zone_x, self.landing_zone_y, _ = pm.geodetic2enu(self.landing_zone_lat, self.landing_zone_lon, self.z_limit, self.current_lat, self.current_lon, self.current_alt)

		self.waypoint1_x, self.waypoint1_y, _ = pm.geodetic2enu(self.waypoint1_lat, self.waypoint1_lon, self.waypoint1_alt, self.current_lat, self.current_lon, self.current_alt)
		self.waypoint2_x, self.waypoint2_y, _ = pm.geodetic2enu(self.waypoint2_lat, self.waypoint2_lon, self.waypoint2_alt, self.current_lat, self.current_lon, self.current_alt)
		self.waypoint3_x, self.waypoint3_y, _ = pm.geodetic2enu(self.waypoint3_lat, self.waypoint3_lon, self.waypoint3_alt, self.current_lat, self.current_lon, self.current_alt)
		self.waypoint4_x, self.waypoint4_y, _ = pm.geodetic2enu(self.waypoint4_lat, self.waypoint4_lon, self.waypoint4_alt, self.current_lat, self.current_lon, self.current_alt)
		# rospy.loginfo('Successfully set waypoints to x,y,z')
		self.worker1_search_WP1_x, self.worker1_search_WP1_y, _ = pm.geodetic2enu(self.worker1_search_WP1_lat, self.worker1_search_WP1_lon, self.worker1_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.worker1_search_WP2_x, self.worker1_search_WP2_y, _ = pm.geodetic2enu(self.worker1_search_WP2_lat, self.worker1_search_WP2_lon, self.worker1_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.worker1_search_WP3_x, self.worker1_search_WP3_y, _ = pm.geodetic2enu(self.worker1_search_WP3_lat, self.worker1_search_WP3_lon, self.worker1_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.worker1_search_WP4_x, self.worker1_search_WP4_y, _ = pm.geodetic2enu(self.worker1_search_WP4_lat, self.worker1_search_WP4_lon, self.worker1_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.worker1_search_WP5_x, self.worker1_search_WP5_y, _ = pm.geodetic2enu(self.worker1_search_WP5_lat, self.worker1_search_WP5_lon, self.worker1_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.worker1_search_WP6_x, self.worker1_search_WP6_y, _ = pm.geodetic2enu(self.worker1_search_WP6_lat, self.worker1_search_WP6_lon, self.worker1_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.worker1_search_WP7_x, self.worker1_search_WP7_y, _ = pm.geodetic2enu(self.worker1_search_WP7_lat, self.worker1_search_WP7_lon, self.worker1_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.worker1_search_WP8_x, self.worker1_search_WP8_y, _ = pm.geodetic2enu(self.worker1_search_WP8_lat, self.worker1_search_WP8_lon, self.worker1_search_z, self.current_lat, self.current_lon, self.current_alt)

		self.building_center_x, self.building_center_y, _ = pm.geodetic2enu(self.building_center_lat, self.building_center_lon, self.z_limit, self.current_lat, self.current_lon, self.current_alt)
		
		# # #----- UNCOMMENT THIS ONLY IF YOU WANT TO SPECIFY LOCAL ENTRANCE SEARCH PATH BASED ON GPS LOCATIONS OF CORNERs OF SQUARE PATH INSTEAD OF BUILDING CENTER POINT ------ #
		self.entrance_search_corner1_x, self.entrance_search_corner1_y, _ = pm.geodetic2enu(self.bldg_search_corner1_lat, self.bldg_search_corner1_lon, self.entrance_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.entrance_search_corner1_5_x, self.entrance_search_corner1_5_y, _ = pm.geodetic2enu(self.bldg_search_corner1_5_lat, self.bldg_search_corner1_5_lon, self.entrance_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.entrance_search_corner2_x, self.entrance_search_corner2_y, _ = pm.geodetic2enu(self.bldg_search_corner2_lat, self.bldg_search_corner2_lon, self.entrance_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.entrance_search_corner2_5_x, self.entrance_search_corner2_5_y, _ = pm.geodetic2enu(self.bldg_search_corner2_5_lat, self.bldg_search_corner2_5_lon, self.entrance_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.entrance_search_corner3_x, self.entrance_search_corner3_y, _ = pm.geodetic2enu(self.bldg_search_corner3_lat, self.bldg_search_corner3_lon, self.entrance_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.entrance_search_corner3_5_x, self.entrance_search_corner3_5_y, _ = pm.geodetic2enu(self.bldg_search_corner3_5_lat, self.bldg_search_corner3_5_lon, self.entrance_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.entrance_search_corner4_x, self.entrance_search_corner4_y, _ = pm.geodetic2enu(self.bldg_search_corner4_lat, self.bldg_search_corner4_lon, self.entrance_search_z, self.current_lat, self.current_lon, self.current_alt)
		self.entrance_search_corner4_5_x, self.entrance_search_corner4_5_y, _ = pm.geodetic2enu(self.bldg_search_corner4_5_lat, self.bldg_search_corner4_5_lon, self.entrance_search_z, self.current_lat, self.current_lon, self.current_alt)

		self.yaw_top_z = atan2((self.entrance_search_corner4_y-self.entrance_search_corner1_y),(self.entrance_search_corner4_x-self.entrance_search_corner1_x))
		self.yaw_right_z = atan2((self.entrance_search_corner2_y-self.entrance_search_corner1_y),(self.entrance_search_corner2_x-self.entrance_search_corner1_x))
		self.yaw_bottom_z = atan2((self.entrance_search_corner1_y-self.entrance_search_corner4_y),(self.entrance_search_corner1_x-self.entrance_search_corner4_x))
		self.yaw_left_z = atan2((self.entrance_search_corner1_y-self.entrance_search_corner2_y),(self.entrance_search_corner1_x-self.entrance_search_corner2_x))
		# #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------#

		self.building_entr1_x, self.building_entr1_y, _ = pm.geodetic2enu(self.building_entr1_lat, self.building_entr1_lon, self.enter_bldg_hgt, self.current_lat, self.current_lon, self.current_alt)
		self.building_entr2_x, self.building_entr2_y, _ = pm.geodetic2enu(self.building_entr2_lat, self.building_entr2_lon, self.enter_bldg_hgt, self.current_lat, self.current_lon, self.current_alt)
		self.building_entr3_x, self.building_entr3_y, _ = pm.geodetic2enu(self.building_entr3_lat, self.building_entr3_lon, self.enter_bldg_hgt, self.current_lat, self.current_lon, self.current_alt)

		self.building_entr4_x, self.building_entr4_y, _ = pm.geodetic2enu(self.building_entr4_lat, self.building_entr4_lon, self.enter_bldg_hgt, self.current_lat, self.current_lon, self.current_alt)

		self.building_entr1_inside_x, self.building_entr1_inside_y, _ = pm.geodetic2enu(self.building_entr1_inside_lat, self.building_entr1_inside_lon, self.enter_bldg_hgt, self.current_lat, self.current_lon, self.current_alt)
		self.building_entr2_inside_x, self.building_entr2_inside_y, _ = pm.geodetic2enu(self.building_entr2_inside_lat, self.building_entr2_inside_lon, self.enter_bldg_hgt, self.current_lat, self.current_lon, self.current_alt)
		self.building_entr3_inside_x, self.building_entr3_inside_y, _ = pm.geodetic2enu(self.building_entr3_inside_lat, self.building_entr3_inside_lon, self.enter_bldg_hgt, self.current_lat, self.current_lon, self.current_alt)

		self.building_entr4_inside_x, self.building_entr4_inside_y, _ = pm.geodetic2enu(self.building_entr4_inside_lat, self.building_entr4_inside_lon, self.enter_bldg_hgt, self.current_lat, self.current_lon, self.current_alt)

		# # # THESE WILL NEED TO BE UPDATED DEPENDING ON WHERE THE ENTRANCES ARE
		self.yaw_building_entr1 = self.yaw_top_z
		self.yaw_building_entr2 = self.yaw_top_z
		self.yaw_building_entr3 = self.yaw_top_z
		self.yaw_building_entr4 = self.yaw_top_z

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
		self.WORKER1SEARCH = 0
		self.DELIVERAID1 = 0
		self.GOTOREFULE = 0
		self.REFULE = 0
		self.LOADPACKAGE2 = 0
		self.TAKEOFF2 = 0
		self.GOTOBUILDING = 0
		self.TAGSEARCH = 0
		self.ENTERBUILDING = 0
		self.WORKER2SEARCH = 0
		self.DELIVERAID2 = 0
		self.MAPPING = 0
		self.EXITBUILDING = 0
		self.GOHOME = 0
		self.LAND = 0
		self.HOVER = 0

	# def verifyPOI(self):
	# 	# Gives vehicle time (500*0.01 seconds) to verify the correct identification of
	# 	# a worker and that it wasn't a random false detection.
	# 	if self.verifyPOI_flag:
	# 		counter = 0
	# 		for i in range(500):

	# 			previous_counter = self.counterCb # self.counterCb is constantly getting updated every time objectPoseCb is called
	# 			rospy.sleep(.01) # Give self.counterCb chance to update if objectPoseCb is called again
	# 			if self.counterCb is not previous_counter:
	# 				counter = counter + 1
	# 		if counter >= 5:
	# 			if (self.WORKER1SEARCH):
	# 				self.worker1_found_flag = 1
	# 				rospy.loginfo("Outside worker found!")
	# 			elif (self.WORKER2SEARCH):
	# 				self.worker2_found_flag = 1
	# 				rospy.loginfo("Inside worker found!")

	# 		elif counter < 5:
	# 			rospy.loginfo("Detected false positive. Continuing search.")
		
	# 	self.verifyPOI_flag = 0
	# 	self.counterCb = 0

	def transformationoffcu(self, msg):
		br = tf.TransformBroadcaster()
		br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),rospy.Time.now(),"fcu","local_origin")

	def check_and_avoid(self,angle):

		self.front=list(self.front)
		self.right=list(self.right)
		self.left=list(self.left)

		i=0
		for x in self.front:
			if x < .3:
				self.front[i] = 1000
			i=i+1

		self.left=list(self.left)
		i=0;
		for x in self.left:	
			if x < .3:
				self.left[i] = 1000
			i=i+1

			
		self.right=list(self.right)
		i=0;
		for x in self.right:
			if x < .3:
				self.right[i] = 1000
			i=i+1
		
		self.minfront=min(self.front)
		self.minleft=min(self.left)
		self.minright=min(self.right)
		self.minback=min(self.back)
		print(self.minfront)
		print(self.minleft)
		print(self.minright)
		
		print('tried_left')
		print(self.tried_left)
		print('tried_right')
		print(self.tried_right)
		
		if (self.minfront < 7 and self.minfront>.3):


			if (self.minleft >= self.minright) and (self.minleft > 7 or self.minleft<0.3) and (self.tried_right ==0):
				if (self.minfront < 7 ):
					rospy.loginfo("Going Left")
					dx=0
					dy=-5
			# rospy.sleep(2.0)
					self.positionSp.pose.position.x=dx*math.cos(angle)-dy*math.sin(angle)+self.current_local_x
					self.positionSp.pose.position.y=dx*math.sin(angle)+dy*math.cos(angle)+self.current_local_y
			

					
			elif (self.minright >= self.minleft) and (self.minright > 7  or self.minright<0.3):

				if (self.minfront < 7 ):
					rospy.loginfo("Going Right")
					dx=0
					dy=5
			
					self.positionSp.pose.position.x=dx*math.cos(angle)-dy*math.sin(angle)+self.current_local_x
					self.positionSp.pose.position.y=dx*math.sin(angle)+dy*math.cos(angle)+self.current_local_y
					self.positionSp.pose.position.z=self.positionSp.pose.position.z
					#self.avoid_pub.publish(self.positionSp)
					self.tried_right = 1
		else: 
			self.tried_right = 0
			
	def Execute_Waypoint(self,waypoint_x,waypoint_y,waypoint_z,desired_yaw):

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

			if abs(self.current_local_x - waypoint_x)<= 2.5 and abs(self.current_local_y - waypoint_y) <= 2.5 and abs(self.current_local_z - waypoint_z) <= 1:
				rospy.loginfo("Current position close enough to desired waypoint")
				#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
				###########################################################################################
				self.current_signal = "Reached waypoint"
				###########################################################################################

				done = 1

		return done

	def BuildingScan(self):

		# This function generally follows the same type of path as the worker search except
		# that it always faces towards the building so that it can identify colored tags.

		# USING FOUR CORNERS FOR BUILDING SCAN (UNCOMMENTED THE APPROPRIATE LINES in setWaypoints_and_Fence):
		# User enters GPS locations of all search corners, and then 
		# setWaypoints_and_Fence converts them to local x/y to give the four corner coordinates.
		
		rospy.logwarn(self.building_scan_WP_FLAG)
		self.front=list(self.front)
		i=0
		for x in self.front:
			if x < .3:
				self.front[i] = 1000
			i=i+1
		self.minfront=min(self.front)
		print(self.minfront)

		too_far_off_path = self.stay_on_search_path(self.current_side, self.minfront)


		if too_far_off_path != 1:

			if self.building_scan_WP_FLAG == 1.5:
				self.current_side = 'Top'
				rospy.loginfo("Searching Midpoint")
				# # -------------------------------FOR SIMULATION PURPOSES ONLY----------------------------------#
				# ################################################################################################
				# self.verifyTAG_flag = 1
				# self.tag_color = 'Green'
				# self.green_tag.x = 10
				# self.green_tag.y = -6
				# self.greentagSp.pose.position.x = 10
				# self.greentagSp.pose.position.y = -9
				# ################################################################################################

				reached_entrance_search_WP2_Midpoint = self.Execute_Waypoint(self.entrance_search_corner1_5_x, self.entrance_search_corner1_5_y, self.entrance_search_z, self.yaw_top_z)
				if reached_entrance_search_WP2_Midpoint:
					self.building_scan_WP_FLAG = 2

			elif self.building_scan_WP_FLAG == 2:

				rospy.loginfo("Search top side of building (looking down on map) for tags")
				reached_entrance_search_WP2 = self.Execute_Waypoint(self.entrance_search_corner2_x, self.entrance_search_corner2_y, self.entrance_search_z, self.yaw_top_z)

				if reached_entrance_search_WP2:
					rospy.sleep(1)
					self.building_scan_WP_FLAG = 2.5

					#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
					###########################################################################################
					self.current_signal = "Reached entrance search waypoint TOP/LEFT"
					###########################################################################################
		
			elif self.building_scan_WP_FLAG == 2.5:
				self.current_side = 'Left'
				rospy.loginfo("Searching Midpoint")
				reached_entrance_search_WP3_Midpoint = self.Execute_Waypoint(self.entrance_search_corner2_5_x, self.entrance_search_corner2_5_y, self.entrance_search_z, self.yaw_left_z)
				if reached_entrance_search_WP3_Midpoint:
					self.building_scan_WP_FLAG = 3

			elif self.building_scan_WP_FLAG == 3:

				rospy.loginfo("Search left side of building (looking down on map) for tags")
				reached_entrance_search_WP3 = self.Execute_Waypoint(self.entrance_search_corner3_x, self.entrance_search_corner3_y, self.entrance_search_z, self.yaw_left_z)

				if reached_entrance_search_WP3:
					rospy.sleep(1)
					self.building_scan_WP_FLAG = 3.5

					#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
					###########################################################################################
					self.current_signal = "Reached entrance search waypoint BOTTOM/LEFT"
					###########################################################################################

			elif self.building_scan_WP_FLAG == 3.5:
				self.current_side = 'Bottom'
				rospy.loginfo("Searching Midpoint")
				reached_entrance_search_WP4_Midpoint = self.Execute_Waypoint(self.entrance_search_corner3_5_x, self.entrance_search_corner3_5_y, self.entrance_search_z, self.yaw_bottom_z)
				if reached_entrance_search_WP4_Midpoint:
					self.building_scan_WP_FLAG = 4
		
			elif self.building_scan_WP_FLAG == 4:

				rospy.loginfo("Search bottom side of building (looking down on map) for tags")
				reached_entrance_search_WP4 = self.Execute_Waypoint(self.entrance_search_corner4_x, self.entrance_search_corner4_y, self.entrance_search_z, self.yaw_bottom_z)

				#-----------------------------------FOR SIMULATION PURPOSES ONLY------------------------------#
				###############################################################################################
				# self.verifyTAG_flag = 1
				# self.tag_info.color = 'Red'
				# self.tag_info.x = self.building_center_x - 3
				# self.tag_info.y = self.building_center_y - 5
				###############################################################################################

				if reached_entrance_search_WP4:
					rospy.sleep(1)
					self.building_scan_WP_FLAG = 0.5

					#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
					###########################################################################################
					self.current_signal = "Reached entrance search waypoint BOTTOM/RIGHT"
					###########################################################################################

			elif self.building_scan_WP_FLAG == 0.5:
				self.current_side = 'Right'
				rospy.loginfo("Searching Midpoint")
				reached_entrance_search_WP1_Midpoint = self.Execute_Waypoint(self.entrance_search_corner4_5_x, self.entrance_search_corner4_5_y, self.entrance_search_z, self.yaw_right_z)
				
				if reached_entrance_search_WP1_Midpoint:
					self.building_scan_WP_FLAG = 1

			elif self.building_scan_WP_FLAG == 1:

				rospy.loginfo("Search right side of building (looking down on map) for tags")
				reached_entrance_search_WP1 = self.Execute_Waypoint(self.entrance_search_corner1_x, self.entrance_search_corner1_y, self.entrance_search_z, self.yaw_right_z)

				if reached_entrance_search_WP1:
					self.building_scan_WP_FLAG = 0 # END BUILDING SCAN

					#--------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE---------#
					###########################################################################################
					self.current_signal = "Reached entrance search waypoint TOP/RIGHT"
					###########################################################################################

			else:
				rospy.loginfo('Completed full revolution')
				rospy.loginfo('Number of red tags found:')
				rospy.loginfo(self.red_tag.total_num)
				self.completed_full_rev = 1
				self.building_scan_WP_FLAG = 1.5

	def stay_on_search_path(self,current_side, minfront):
		# AS WRITTEN, THIS CODE IS DEPENDANT ON THE LOCAL REFERENCE FRAME BEING ALIGNED WITH THE FIELD
		rospy.logwarn(current_side)
		rospy.logwarn("minfront")
		rospy.logwarn(minfront)
		too_far_off_path = 0
		backoff_distance = 2
		# if current_side == 'Top':

		if (minfront < backoff_distance and minfront>.3):
			print("VERY CLOSE STAY AWAY")
			too_far_off_path = 1
			self.dx=-20
			self.dy=0
			if current_side == 'Top':
				self.target_yaw = self.yaw_top_z
			elif current_side == 'Left':
				self.target_yaw = self.yaw_left_z
			elif current_side == 'Bottom':
				self.target_yaw = self.yaw_bottom_z
			elif current_side == 'Right':
				self.target_yaw = self.yaw_right_z

			self.positionSp.pose.position.x=self.dx*math.cos(self.target_yaw)-self.dy*math.sin(self.target_yaw)+self.current_local_x
			self.positionSp.pose.position.y=self.dx*math.sin(self.target_yaw)+self.dy*math.cos(self.target_yaw)+self.current_local_y





		# 		self.positionSp.pose.position.x = self.current_local_x + 0.5
		# 		rospy.logwarn("Backing off from top")
		# 		too_far_off_path = 1
		# 	elif (self.current_local_x > (self.entrance_search_corner2_x + 0.5)):
		# 		self.positionSp.pose.position.x = self.current_local_x - 0.5
		# 		rospy.logwarn("Going closer to top")
		# 		too_far_off_path = 1
		# elif current_side == 'Left':

		# 	if (minfront < backoff_distance and minfront>.3):
		# 		self.positionSp.pose.position.y = self.current_local_y + 0.5
		# 		rospy.logwarn("Backing off from left")	
		# 		too_far_off_path = 1
		# 	elif (self.current_local_y > (self.entrance_search_corner3_y + 0.5)):
		# 		self.positionSp.pose.position.y = self.current_local_y - 0.5
		# 		rospy.logwarn("Going closer to left")
		# 		too_far_off_path = 1
		# elif current_side == 'Bottom':

		# 	if (minfront < backoff_distance and minfront>.3):
		# 		self.positionSp.pose.position.x = self.current_local_x - 0.5
		# 		rospy.logwarn("Backing off from bottom")
		# 		too_far_off_path = 1
		# 	elif (self.current_local_x < (self.entrance_search_corner4_x - 0.5)):
		# 		self.positionSp.pose.position.x = self.current_local_x + 0.5
		# 		rospy.logwarn("Going closer to bottom")
		# 		too_far_off_path = 1
		# elif current_side == 'Right':

		# 	if (minfront < backoff_distance and minfront>.3):
		# 		self.positionSp.pose.position.y = self.current_local_y - 0.5
		# 		rospy.logwarn("Backing off from right")
		# 		too_far_off_path = 1
		# 	elif (self.current_local_y < (self.entrance_search_corner1_y - 0.5)):
		# 		self.positionSp.pose.position.y = self.current_local_y + 0.5
		# 		rospy.logwarn("Going closer to right")
		# 		too_far_off_path = 1

		rospy.logwarn('too_far_off_path')
		rospy.logwarn(too_far_off_path)
		return too_far_off_path

	def EntranceSearch(self,entrance_x,entrance_y,entrance_view_angle):
		# This function executes if green tag was not found during BuildingScan, after a complete
		# revolution. This code should make the drone go to about 10 meters, go to a known location
		# of a possible entrance, and then go to 1.5 meters yawing so that it is facing the possible
		# entrance.

		entrance_search_counter = 0
		self.current_side = 'Top'
		self.front=list(self.front)
		i=0
		for x in self.front:
			if x < .3:
				self.front[i] = 1000
			i=i+1
		self.minfront=min(self.front)
		print(self.minfront)

		too_far_off_path = self.stay_on_search_path(self.current_side, self.minfront)


		if too_far_off_path != 1:

			if (self.entrance_search_WPS_FLAG == 0):

				head_to_entrance = self.Execute_Waypoint(entrance_x,entrance_y,self.entrance_search_z,self.yaw_top_z)
				
				if head_to_entrance:
					
					self.entrance_search_WPS_FLAG = 1

			elif (self.entrance_search_WPS_FLAG == 1):
					head_to_entrance = self.Execute_Waypoint(entrance_x,entrance_y,self.entrance_search_z,self.yaw_top_z)
					rospy.loginfo('Checking entrance to see what the color of the tag is')
					if self.entrance_search_counter > 45:
						self.entrance_search_WPS_FLAG = 2


						#----------THIS IS SO I COULD GET THE GPS LOCATION AT THIS CORNER FROM THE LOG FILE-------------#
						#################################################################################################
						# if self.entrance_number_search == 3 and abs(self.current_local_z-self.entrance_search_z)<.5:
						# 	self.verifyTAG_flag = 1
						# 	self.tag_info.color = 'Green'
						# 	self.tag_info.x = self.current_local_x
						# 	self.tag_info.y = self.current_local_y
						#################################################################################################
					else:
						self.verifyTag()
						self.entrance_search_counter = self.entrance_search_counter + 1

						#-------THIS IS SO I COULD GET THE GPS LOCATION AT THIS Location FROM THE LOG FILE--------#
						###########################################################################################
						self.current_signal = "Checking possible unblocked entrance"
						###########################################################################################

					
			else:
				rospy.loginfo('Could not find tag at current entrance')
				self.entrance_search_WPS_FLAG = 0
				self.entrance_search_counter = 0
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
					if abs(temp_x-self.red_tag.x[i]) < 3 and abs(temp_y-self.red_tag.y[i]) < 3:
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
					if (abs(temp_x - self.building_entr1_x) < 10000 and abs(temp_y - self.building_entr1_y) < 10000): # Set really big because we don't care any more what entrance it is at
						at_entrance = 1
					elif (abs(temp_x - self.building_entr2_x) < 10000 and abs(temp_y - self.building_entr2_y) < 10000):
						at_entrance = 2
					elif (abs(temp_x - self.building_entr3_x) < 10000 and abs(temp_y - self.building_entr3_y) < 10000):
						at_entrance = 3
					elif (abs(temp_x - self.building_entr4_x) < 10000 and abs(temp_y - self.building_entr4_y) < 10000):
						at_entrance = 4


					if at_entrance > 0:
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

				rospy.logwarn('!!!!!!!!!!!!!!!!!!!!!!Testing to see if tag is close enough to building!!!!!!!!!!!!!!!!!!!!!!\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
				rospy.logwarn(self.greentagSp.pose.position.x)
				rospy.logwarn(self.greentagSp.pose.position.y)

				if (abs(temp_x - self.building_entr1_x) <= 10000 and abs(temp_y - self.building_entr1_y) <= 10000): # Set really big because we don't care what entrance it is at
					rospy.logwarn("Entrance1")
					rospy.logwarn(self.building_entr1_x)
					rospy.logwarn(self.building_entr1_y)
					self.unblocked_entrance = 1
					self.unblocked_entrance_x = self.building_entr1_x
					self.unblocked_entrance_y = self.building_entr1_y
				elif (abs(temp_x - self.building_entr2_x) <= 10000 and abs(temp_y - self.building_entr2_y) <= 10000):
					rospy.logwarn("Entrance2")
					rospy.logwarn(self.building_entr2_x)
					rospy.logwarn(self.building_entr2_y)
					self.unblocked_entrance = 2
					self.unblocked_entrance_x = self.building_entr2_x
					self.unblocked_entrance_y = self.building_entr2_y
				elif (abs(temp_x - self.building_entr3_x) <= 10000 and abs(temp_y - self.building_entr3_y) <= 10000):
					rospy.logwarn("Entrance3")
					rospy.logwarn(self.building_entr3_x)
					rospy.logwarn(self.building_entr3_y)
					self.unblocked_entrance = 3
					self.unblocked_entrance_x = self.building_entr3_x
					self.unblocked_entrance_y = self.building_entr3_y
				elif (abs(temp_x - self.building_entr4_x) <= 10000 and abs(temp_y - self.building_entr4_y) <= 10000):
					rospy.logwarn("Entrance4")
					rospy.logwarn(self.building_entr4_x)
					rospy.logwarn(self.building_entr4_y)
					self.unblocked_entrance = 4
					self.unblocked_entrance_x = self.building_entr4_x
					self.unblocked_entrance_y = self.building_entr4_y

				if self.unblocked_entrance > 0:
					self.green_tag.x = temp_x
					self.green_tag.y = temp_y
					self.green_tag.found = 1

					rospy.loginfo('Green tag found!')

					self.ready2capturegreen = 1
					(self.green_tag.lat, self.green_tag.lon, _) = pm.enu2geodetic(temp_x,temp_y,0,self.home_lat,self.home_lon,0)
					self.tag_color = "None"
		
		self.verifyTAG_flag = 0
		self.counterTAGCb = 0

	def Execute_EnterBuilding(self,target_yaw):
		rospy.logwarn("Entered Execute_EnterBuilding function")
		r = 0.5
		self.enter_bldg_right=list(self.enter_bldg_right)
		i=0
		for x in self.enter_bldg_right:
			if x < .3:
				self.enter_bldg_right[i] = 1000
			i=i+1
		self.enter_bldg_right_min=min(self.enter_bldg_right)
		print(self.enter_bldg_right_min)

		self.enter_bldg_left=list(self.enter_bldg_left)
		i=0
		for x in self.enter_bldg_left:
			if x < .3:
				self.enter_bldg_left[i] = 1000
			i=i+1
		self.enter_bldg_left_min=min(self.enter_bldg_left)
		print(self.enter_bldg_left_min)

		if self.enter_bldg_right_min > 100:
			self.enter_bldg_right_min = 100
		if self.enter_bldg_left_min > 100:
			self.enter_bldg_left_min = 100

		if (self.enter_bldg_right_min - self.enter_bldg_left_min) < -.5 and self.enter_bldg_right_min < 2:
			# Heading left
			# self.positionSp.pose.position.x = self.current_local_x + r*cos(pi/2 + target_yaw)
			# self.positionSp.pose.position.y = self.current_local_y + r*sin(pi/2 + target_yaw)
			dx=0
			dy=r
			# rospy.sleep(2.0)
			self.positionSp.pose.position.x=dx*math.cos(self.target_yaw)-dy*math.sin(self.target_yaw)+self.current_local_x
			self.positionSp.pose.position.y=dx*math.sin(self.target_yaw)+dy*math.cos(self.target_yaw)+self.current_local_y

		elif (self.enter_bldg_right_min - self.enter_bldg_left_min) > .5 and self.enter_bldg_left_min < 2:
			# Heading right
			# self.positionSp.pose.position.x = self.current_local_x - r*cos(pi/2 + target_yaw)
			# self.positionSp.pose.position.y = self.current_local_y - r*sin(pi/2 + target_yaw)
			dx=0
			dy=-r
			# rospy.sleep(2.0)
			self.positionSp.pose.position.x=dx*math.cos(self.target_yaw)-dy*math.sin(self.target_yaw)+self.current_local_x
			self.positionSp.pose.position.y=dx*math.sin(self.target_yaw)+dy*math.cos(self.target_yaw)+self.current_local_y

		else:

			# self.positionSp.pose.position.x = self.current_local_x + r*cos(target_yaw)
			# self.positionSp.pose.position.y = self.current_local_y + r*sin(target_yaw)
			if (self.enter_bldg_flag == 1):

				if self.unblocked_entrance == 1:
					self.positionSp.pose.position.x = self.building_entr1_inside_x
					self.positionSp.pose.position.y = self.building_entr1_inside_y
					self.positionSp.pose.position.z=self.enter_bldg_hgt
					self.target_yaw = math.atan2((self.positionSp.pose.position.y-self.current_local_y),(self.positionSp.pose.position.x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					if abs(self.current_local_x-self.building_entr1_inside_x) <.3 and abs(self.current_local_y-self.building_entr1_inside_y)<.3:
						self.enter_bldg_flag=2 

				elif self.unblocked_entrance == 2:
					self.positionSp.pose.position.x = self.building_entr2_inside_x
					self.positionSp.pose.position.y = self.building_entr2_inside_y
					self.positionSp.pose.position.z=self.enter_bldg_hgt
					self.target_yaw = math.atan2((self.positionSp.pose.position.y-self.current_local_y),(self.positionSp.pose.position.x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					if abs(self.current_local_x-self.building_entr2_inside_x) <.3 and abs(self.current_local_y-self.building_entr2_inside_y)<.3:
						self.enter_bldg_flag=2 
									 

				elif self.unblocked_entrance == 3:
					self.positionSp.pose.position.x = self.building_entr3_inside_x
					self.positionSp.pose.position.y = self.building_entr3_inside_y
					self.positionSp.pose.position.z=self.enter_bldg_hgt
					self.target_yaw = math.atan2((self.positionSp.pose.position.y-self.current_local_y),(self.positionSp.pose.position.x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					if abs(self.current_local_x-self.building_entr3_inside_x) <.3 and abs(self.current_local_y-self.building_entr3_inside_y)<.3:
						self.enter_bldg_flag=2

				elif self.unblocked_entrance == 4:
					self.positionSp.pose.position.x = self.building_entr4_inside_x
					self.positionSp.pose.position.y = self.building_entr4_inside_y
					self.positionSp.pose.position.z=self.enter_bldg_hgt
					self.target_yaw = math.atan2((self.positionSp.pose.position.y-self.current_local_y),(self.positionSp.pose.position.x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					if abs(self.current_local_x-self.building_entr4_inside_x) <.3 and abs(self.current_local_y-self.building_entr4_inside_y)<.3:
						self.enter_bldg_flag=2

			if (self.exit_bldg_flag == 1):

				if self.unblocked_entrance == 1:
					self.positionSp.pose.position.x = self.building_entr1_x
					self.positionSp.pose.position.y = self.building_entr1_y
					self.positionSp.pose.position.z=self.enter_bldg_hgt
					self.target_yaw = math.atan2((self.positionSp.pose.position.y-self.current_local_y),(self.positionSp.pose.position.x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					if abs(self.current_local_x-self.building_entr1_x) <.3 and abs(self.current_local_y-self.building_entr1_y)<.3:
						self.exit_bldg_flag=2 

				elif self.unblocked_entrance == 2:
					self.positionSp.pose.position.x = self.building_entr2_x
					self.positionSp.pose.position.y = self.building_entr2_y
					self.positionSp.pose.position.z=self.enter_bldg_hgt
					self.target_yaw = math.atan2((self.positionSp.pose.position.y-self.current_local_y),(self.positionSp.pose.position.x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					if abs(self.current_local_x-self.building_entr2_x) <.3 and abs(self.current_local_y-self.building_entr2_y)<.3:
						self.exit_bldg_flag=2 
									 

				elif self.unblocked_entrance == 3:
					self.positionSp.pose.position.x = self.building_entr3_x
					self.positionSp.pose.position.y = self.building_entr3_y
					self.positionSp.pose.position.z=self.enter_bldg_hgt
					self.target_yaw = math.atan2((self.positionSp.pose.position.y-self.current_local_y),(self.positionSp.pose.position.x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					if abs(self.current_local_x-self.building_entr3_x) <.3 and abs(self.current_local_y-self.building_entr3_y)<.3:
						self.exit_bldg_flag=2

				elif self.unblocked_entrance == 4:
					self.positionSp.pose.position.x = self.building_entr4_x
					self.positionSp.pose.position.y = self.building_entr4_y
					self.positionSp.pose.position.z=self.enter_bldg_hgt
					self.target_yaw = math.atan2((self.positionSp.pose.position.y-self.current_local_y),(self.positionSp.pose.position.x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					if abs(self.current_local_x-self.building_entr4_x) <.3 and abs(self.current_local_y-self.building_entr4_y)<.3:
						self.exit_bldg_flag=2


#-----------------------INDOOR-------------------------------------------------------------------------------------#
	def obstacle_avoid_decision(self):
		rospy.logwarn('Entered obstacle_avoid_decision function')
		all360 = self.all360
		all360 = list(all360)
		object_far_enough_dist = 4 #3
		i=0
		for x in all360:
			if x < .5:
				all360[i] = 1000
			i=i+1
		list_1080 = np.concatenate((all360,all360), axis=0) # This simplifies taking a 60 degree swath from the data so that we don't have to worry about trying to combine edges of data (x > 0 with x < 360)
		potential_path = []
		potential_path_index = [] # Not sure why I had to add this. For some reason, if I get rid of it and set a temp=potential_path, anything that happens to temp also happens to potential_path like they are connected somehow
		scan_for_opening = np.zeros(120)
		i=180
		azimuth = 180
		for x in range(360):
			scan_for_opening = list_1080[i-60:i+60]
			scan_for_opening = min(scan_for_opening)
			if scan_for_opening > object_far_enough_dist:
				potential_path.append(azimuth)
				potential_path_index.append(azimuth)
			if azimuth == 359:
				azimuth = 0
			elif azimuth != 359:
				azimuth = azimuth + 1
			i = i + 1
		if potential_path_index == []:
			print "Error! All read values are lower than minimum object distance! Try again!"
			best_azimuth = 180
			print "Still heading forward"
		else:
			target_heading= math.degrees(math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x)))
			potential_path_index[:] = [abs(i - 180) for i in potential_path_index] # Subtract 180 from every value in potential_path
			best_azimuth_index = potential_path_index.index(min(potential_path_index))
			best_azimuth = potential_path[best_azimuth_index]
			rospy.logwarn("Best azimuth")
			rospy.logwarn(best_azimuth)

		best_azimuth = (best_azimuth-180)*pi/180.0 # Convert the azimuth from [0,360] to [-pi,pi]
		rospy.logwarn("Best azimuth")
		rospy.logwarn(best_azimuth)
		rospy.logwarn("EXITING obstacle_avoid_decision function")
		return best_azimuth

	def indoor_check_and_avoid(self):
		rospy.logwarn('Entered check_and_avoid function')
		min_dist_to_obstacle = 3 #2
		r = 3.5 #2.5 Distance to travel to avoid obstacle
		front = self.front
		front=list(front)
		i=0
		for x in front:
			if x < .5:
				front[i] = 1000
			i=i+1
		self.minfront=min(front)
		print('minfront : ')
		print(self.minfront)

		if (self.minfront < min_dist_to_obstacle) or (self.avoiding_obstacle == 1):
			self.avoiding_obstacle =1 
			if (self.avoidancepar == 1):
				self.stay_current_x =self.current_local_x
				self.stay_current_y = self.current_local_y
				yaw_add = self.obstacle_avoid_decision()
				self.target_yaw = self.current_yaw +yaw_add
				quaternion = quaternion_from_euler(0, 0, self.target_yaw)
				self.positionSp.pose.orientation = Quaternion(*quaternion)
				self.avoidancepar=2
				self.positionSp.pose.position.x = self.stay_current_x
				self.positionSp.pose.position.y = self.stay_current_y
				self.positionSp.pose.position.z = self.indoor_hgt
				self.avoid_pub.publish(self.positionSp)

			if (self.avoidancepar ==2):
				self.positionSp.pose.position.x = self.stay_current_x
				self.positionSp.pose.position.y = self.stay_current_y
				self.positionSp.pose.position.z = self.indoor_hgt
				if (abs(self.current_yaw-self.target_yaw) < .3) or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3) or (abs(self.current_yaw - (self.target_yaw+2*pi)) < .3):
					rospy.logwarn("CONGRATULATIONS!!! YOU HAVE CHANGED YOUR YAW")
					if(self.minfront < min_dist_to_obstacle):
						print("Ops! YOU STILL SEE STH AFTER TURNING, WE WILL CHANGE PAR TO 1")
						self.avoidancepar=1
					else:
						self.avoidancepar = 3
						rospy.logwarn('Amount to add to x')
						rospy.logwarn(self.dx*math.cos(self.target_yaw)-self.dy*math.sin(self.target_yaw))
						rospy.logwarn('Amount to add to y')
						rospy.logwarn(self.dx*math.sin(self.target_yaw)+self.dy*math.cos(self.target_yaw))
						self.dx=r
						self.dy=0
						self.avoid_x=self.dx*math.cos(self.target_yaw)-self.dy*math.sin(self.target_yaw)+self.current_local_x
						self.avoid_y=self.dx*math.sin(self.target_yaw)+self.dy*math.cos(self.target_yaw)+self.current_local_y
			if (self.avoidancepar ==3):
				if (abs(self.avoid_x) <= 8+self.building_center_x and abs(self.avoid_y) <= 8+self.building_center_y):
					rospy.loginfo("Avoiding position is within bounds")
					self.positionSp.pose.position.x = self.avoid_x
					self.positionSp.pose.position.y = self.avoid_y
					self.positionSp.pose.position.z = self.indoor_hgt
					self.positionSp.header.frame_id = 'local_origin'
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)
					self.avoidancepar = 4
				elif (abs(self.avoid_x) > 8+self.building_center_x or abs(self.avoid_y) > 8+self.building_center_y):
					rospy.logwarn("Avoiding position is out of bounds")

					if abs(self.avoid_x) > 8+self.building_center_x or abs(self.avoid_y) > 8+self.building_center_y:
						rospy.logwarn("At the corner") 
						self.avoiding_obstacle = 0
						self.building_param = [0, 0, 0]
						self.avoidancepar = 1
						if self.path_flag == 'Center':
							self.path_flag = 'None'
							print ("Flag changed in check and avoid")
							self.target_x=self.building_center_x-self.current_r
							self.target_y=self.building_center_y-self.current_r
							self.positionSp.pose.position.x=self.current_local_x
							self.positionSp.pose.position.y=self.current_local_y
							self.positionSp.pose.position.z = self.indoor_hgt
							self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
							quaternion = quaternion_from_euler(0, 0, self.target_yaw)
							self.positionSp.pose.orientation = Quaternion(*quaternion)
						elif self.path_flag == 'None':
							self.path_flag = 'Right'
							print ("Flag changed in check and avoid in corner")
							self.target_x=self.building_center_x+self.current_r
							self.target_y=self.building_center_y-self.current_r
							self.positionSp.pose.position.x=self.current_local_x
							self.positionSp.pose.position.y=self.current_local_y
							self.positionSp.pose.position.z = self.indoor_hgt
							self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
							quaternion = quaternion_from_euler(0, 0, self.target_yaw)
							self.positionSp.pose.orientation = Quaternion(*quaternion)  
						elif self.path_flag == 'Right':
							if (self.current_r ==2):
								if (self.flagcount ==0):
									self.righttemp=2
									self.flagcount=self.flagcount+1
								else: 
									self.path_flag= 'Far'
									self.flagcount= 0 
							elif(self.current_r < 9):
								if (self.flagcount ==0):
									self.righttemp=-self.current_r+4
									self.flagcount =self.flagcount+1
								else: 
									self.righttemp=self.righttemp+4
									self.flagcount= self.flagcount +1
								if (abs(self.righttemp)>abs(self.current_r)):
									self.righttemp=0
									self.flagcount=0
									if (self.mappingdirection == 1):

										self.path_flag = 'Far'
									elif (self.mappingdirection == -1):
										self.path_flag = 'Near'
										if (self.inverse_flag > 0):
											self.current_r = self.current_r+2

							if (self.path_flag == 'Right'):
								print('Right Temp')
								print(self.righttemp)
								if (self.mappingdirection ==1):
									self.target_x=self.building_center_x+self.righttemp
									self.target_y=self.building_center_y-self.current_r
								elif(self.mappingdirection == -1):
									self.target_y=self.building_center_y+self.righttemp
									self.target_x=self.building_center_x-self.current_r

								self.stay_current_x_waypoint=self.current_local_x
								self.stay_current_y_waypoint=self.current_local_y
								self.positionSp.pose.position.x=self.current_local_x
								self.positionSp.pose.position.y=self.current_local_y
								self.positionSp.pose.position.z = self.indoor_hgt
								self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
								quaternion = quaternion_from_euler(0, 0, self.target_yaw)
								self.positionSp.pose.orientation = Quaternion(*quaternion)  
								self.building_param[0] = 1

						elif self.path_flag == 'Far':
							if (self.current_r ==2):
								if (self.flagcount ==0):
									self.fartemp=2
									self.flagcount=self.flagcount+1
								else: 
									self.path_flag= 'Left'
									self.flagcount = 0 
							elif (self.current_r<9):
								if (self.flagcount ==0):
									self.fartemp=-self.current_r+4
									self.flagcount =self.flagcount+1
								else: 
									self.fartemp=self.fartemp+4
									self.flagcount= self.flagcount +1
								if (abs(self.fartemp)>abs(self.current_r)):
									self.fartemp=0
									self.flagcount=0
									if(self.mappingdirection==1):
										self.path_flag = 'Left'
									elif(self.mappingdirection == -1):
										self.path_flag = 'Right'
										self.inverse_flag = self.inverse_flag+1
							if (self.path_flag == 'Far'):
								print('FAR Temp')
								print(self.fartemp)
								if (self.mappingdirection ==1):
									self.target_x=self.building_center_x+self.current_r
									self.target_y=self.building_center_y+self.fartemp
								elif(self.mappingdirection == -1):
									self.target_y=self.building_center_y-self.current_r
									self.target_x=self.building_center_x-self.fartemp
								self.stay_current_x_waypoint=self.current_local_x
								self.stay_current_y_waypoint=self.current_local_y
								self.positionSp.pose.position.x=self.current_local_x
								self.positionSp.pose.position.y=self.current_local_y
								self.positionSp.pose.position.z = self.indoor_hgt
								self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
								quaternion = quaternion_from_euler(0, 0, self.target_yaw)
								self.positionSp.pose.orientation = Quaternion(*quaternion)  
								self.building_param[0] = 1

				  		elif self.path_flag == 'Left':
				  			if self.building_param[0] == 0:
								if (self.current_r ==2):
									if (self.flagcount ==0):
										self.lefttemp=2
										self.flagcount=self.flagcount+1
									else: 
										self.path_flag= 'Near'
										self.flagcount = 0 
								elif (self.current_r < 9):
									if (self.flagcount ==0):
										self.lefttemp=-self.current_r+4
										self.flagcount =self.flagcount+1
									else: 
										self.lefttemp=self.lefttemp+4
										self.flagcount= self.flagcount +1
									if (abs(self.lefttemp)>abs(self.current_r)):
										self.lefttemp=0
										self.flagcount=0
										if(self.mappingdirection == 1):
											self.path_flag = 'Near'
										elif (self.mappingdirection == -1):
											self.path_flag = 'Far'	
								if (self.path_flag == 'Left'):
									print('left Temp')
									print(self.fartemp)
									if (self.mappingdirection ==1):
										self.target_x=self.building_center_x-self.lefttemp
										self.target_y=self.building_center_y+self.current_r
									elif (self.mappingdirection == -1):
										self.target_y=self.building_center_y-self.lefttemp
										self.target_x=self.building_center_x+self.current_r

									self.stay_current_x_waypoint=self.current_local_x
									self.stay_current_y_waypoint=self.current_local_y
									self.positionSp.pose.position.x=self.current_local_x
									self.positionSp.pose.position.y=self.current_local_y
									self.positionSp.pose.position.z = self.indoor_hgt
									self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
									quaternion = quaternion_from_euler(0, 0, self.target_yaw)
									self.positionSp.pose.orientation = Quaternion(*quaternion)  
									self.building_param[0] = 1

						elif self.path_flag == 'Near':
							if (self.current_r ==2):
								if (self.flagcount ==0):
									self.neartemp=2
									self.flagcount=self.flagcount+1
								else: 
									self.path_flag= 'Right'
									self.flagcount = 0 
									self.current_r=self.current_r+2
							elif(self.current_r <9):
								if (self.flagcount ==0):
									self.neartemp=-self.current_r+4
									self.flagcount =self.flagcount+1
								else: 
									self.neartemp=self.neartemp+4
									self.flagcount= self.flagcount +1
								if (abs(self.neartemp)>abs(self.current_r)):
									self.neartemp=0
									self.flagcount=0
									if(self.mappingdirection == 1):
										self.path_flag = 'Right'
										self.current_r=self.current_r+2
									elif(self.mappingdirection == -1):
										self.path_flag = 'Left'
							if (self.path_flag == 'Near'):
								print('Near Temp')
								print(self.neartemp)
								if (self.mappingdirection ==1):
									self.target_x=self.building_center_x-self.current_r
									self.target_y=self.building_center_y-self.neartemp
								elif(self.mappingdirection ==-1):
									self.target_y=self.building_center_y+self.current_r
									self.target_x=self.building_center_x+self.neartemp

								self.stay_current_x_waypoint=self.current_local_x
								self.stay_current_y_waypoint=self.current_local_y
								self.positionSp.pose.position.x=self.current_local_x
								self.positionSp.pose.position.y=self.current_local_y
								self.positionSp.pose.position.z = self.indoor_hgt
								self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
								quaternion = quaternion_from_euler(0, 0, self.target_yaw)
								self.positionSp.pose.orientation = Quaternion(*quaternion)  
								self.building_param[0] = 1

						if(self.EXITBUILDING):
							reached_unblocked_entrance=1

			if (self.avoidancepar == 4 ):
				if (self.minfront < min_dist_to_obstacle):
					self.avoidancepar =1
				elif abs(self.current_local_x - self.avoid_x) < .7 and (abs(self.current_local_y - self.avoid_y) < .7):
					rospy.loginfo("Attempted to avoid obstacle. Now will check to see if on path")
					self.avoiding_obstacle = 0
					self.avoidancepar = 1
					self.building_param = [0, 0, 0]
			if (abs(self.current_local_x-self.target_x)<3) and (abs(self.current_local_y-self.target_y)<3) :
				self.avoiding_obstacle = 0
				self.building_param = [0, 0, 0]
				self.avoidancepar = 1
				if(self.EXITBUILDING):
					reached_unblocked_entrance=1
				if self.path_flag == 'Center':
					self.path_flag = 'None'
					print ("Flag changed in check and avoid")
					self.target_x=self.building_center_x-self.current_r
					self.target_y=self.building_center_y-self.current_r
					self.positionSp.pose.position.x=self.current_local_x
					self.positionSp.pose.position.y=self.current_local_y
					self.positionSp.pose.position.z = self.indoor_hgt
					self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)
				elif self.path_flag == 'None':
					self.path_flag = 'Right'
					print ("Flag changed in check and avoid")
					self.target_x=self.building_center_x+self.current_r
					self.target_y=self.building_center_y-self.current_r
					self.positionSp.pose.position.x=self.current_local_x
					self.positionSp.pose.position.y=self.current_local_y
					self.positionSp.pose.position.z = self.indoor_hgt
					self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)
				elif self.path_flag == 'Right':
					if (self.current_r ==2):
						if (self.flagcount ==0):
							self.righttemp=2
							self.flagcount=self.flagcount+1
						else: 
							self.path_flag= 'Far'
							self.flagcount= 0 
					elif(self.current_r < 9):
						if (self.flagcount ==0):
							self.righttemp=-self.current_r+4
							self.flagcount =self.flagcount+1
						else: 
							self.righttemp=self.righttemp+4
							self.flagcount= self.flagcount +1
						if (abs(self.righttemp)>abs(self.current_r)):
							self.righttemp=0
							self.flagcount=0
							if (self.mappingdirection == 1):

								self.path_flag = 'Far'
							elif (self.mappingdirection == -1):
								self.path_flag = 'Near'
								if (self.inverse_flag > 0):
									self.current_r = self.current_r+2

					if (self.path_flag == 'Right'):
						print('Right Temp')
						print(self.righttemp)
						if (self.mappingdirection ==1):
							self.target_x=self.building_center_x+self.righttemp
							self.target_y=self.building_center_y-self.current_r
						elif(self.mappingdirection == -1):
							self.target_y=self.building_center_y+self.righttemp
							self.target_x=self.building_center_x-self.current_r

						self.stay_current_x_waypoint=self.current_local_x
						self.stay_current_y_waypoint=self.current_local_y
						self.positionSp.pose.position.x=self.current_local_x
						self.positionSp.pose.position.y=self.current_local_y
						self.positionSp.pose.position.z = self.indoor_hgt
						self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
						quaternion = quaternion_from_euler(0, 0, self.target_yaw)
						self.positionSp.pose.orientation = Quaternion(*quaternion)  
						self.building_param[0] = 1

				elif self.path_flag == 'Far':
					if (self.current_r ==2):
						if (self.flagcount ==0):
							self.fartemp=2
							self.flagcount=self.flagcount+1
						else: 
							self.path_flag= 'Left'
							self.flagcount = 0 
					elif (self.current_r<9):
						if (self.flagcount ==0):
							self.fartemp=-self.current_r+4
							self.flagcount =self.flagcount+1
						else: 
							self.fartemp=self.fartemp+4
							self.flagcount= self.flagcount +1
						if (abs(self.fartemp)>abs(self.current_r)):
							self.fartemp=0
							self.flagcount=0
							if(self.mappingdirection==1):
								self.path_flag = 'Left'
							elif(self.mappingdirection == -1):
								self.path_flag = 'Right'
								self.inverse_flag = self.inverse_flag+1
					if (self.path_flag == 'Far'):
						print('FAR Temp')
						print(self.fartemp)
						if (self.mappingdirection ==1):
							self.target_x=self.building_center_x+self.current_r
							self.target_y=self.building_center_y+self.fartemp
						elif(self.mappingdirection == -1):
							self.target_y=self.building_center_y-self.current_r
							self.target_x=self.building_center_x-self.fartemp
						self.stay_current_x_waypoint=self.current_local_x
						self.stay_current_y_waypoint=self.current_local_y
						self.positionSp.pose.position.x=self.current_local_x
						self.positionSp.pose.position.y=self.current_local_y
						self.positionSp.pose.position.z = self.indoor_hgt
						self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
						quaternion = quaternion_from_euler(0, 0, self.target_yaw)
						self.positionSp.pose.orientation = Quaternion(*quaternion)  
						self.building_param[0] = 1

				elif self.path_flag == 'Left':
					if self.building_param[0] == 0:
						if (self.current_r ==2):
							if (self.flagcount ==0):
								self.lefttemp=2
								self.flagcount=self.flagcount+1
							else: 
								self.path_flag= 'Near'
								self.flagcount = 0 
						elif (self.current_r < 9):
							if (self.flagcount ==0):
								self.lefttemp=-self.current_r+4
								self.flagcount =self.flagcount+1
							else: 
								self.lefttemp=self.lefttemp+4
								self.flagcount= self.flagcount +1
							if (abs(self.lefttemp)>abs(self.current_r)):
								self.lefttemp=0
								self.flagcount=0
								if(self.mappingdirection == 1):
									self.path_flag = 'Near'
								elif (self.mappingdirection == -1):
									self.path_flag = 'Far'	
						if (self.path_flag == 'Left'):
							print('left Temp')
							print(self.fartemp)
							if (self.mappingdirection ==1):
								self.target_x=self.building_center_x-self.lefttemp
								self.target_y=self.building_center_y+self.current_r
							elif (self.mappingdirection == -1):
								self.target_y=self.building_center_y-self.lefttemp
								self.target_x=self.building_center_x+self.current_r

							self.stay_current_x_waypoint=self.current_local_x
							self.stay_current_y_waypoint=self.current_local_y
							self.positionSp.pose.position.x=self.current_local_x
							self.positionSp.pose.position.y=self.current_local_y
							self.positionSp.pose.position.z = self.indoor_hgt
							self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
							quaternion = quaternion_from_euler(0, 0, self.target_yaw)
							self.positionSp.pose.orientation = Quaternion(*quaternion)  
							self.building_param[0] = 1
		
				elif self.path_flag == 'Near':
					if (self.current_r ==2):
						if (self.flagcount ==0):
							self.neartemp=2
							self.flagcount=self.flagcount+1
						else: 
							self.path_flag= 'Right'
							self.flagcount = 0 
							self.current_r=self.current_r+2
					elif(self.current_r <9):
						if (self.flagcount ==0):
							self.neartemp=-self.current_r+4
							self.flagcount =self.flagcount+1
						else: 
							self.neartemp=self.neartemp+4
							self.flagcount= self.flagcount +1
						if (abs(self.neartemp)>abs(self.current_r)):
							self.neartemp=0
							self.flagcount=0
							if(self.mappingdirection == 1):
								self.path_flag = 'Right'
								self.current_r=self.current_r+2
							elif(self.mappingdirection == -1):
								self.path_flag = 'Left'
					if (self.path_flag == 'Near'):
						print('Near Temp')
						print(self.neartemp)
						if (self.mappingdirection ==1):
							self.target_x=self.building_center_x-self.current_r
							self.target_y=self.building_center_y-self.neartemp
						elif(self.mappingdirection ==-1):
							self.target_y=self.building_center_y+self.current_r
							self.target_x=self.building_center_x+self.neartemp

						self.stay_current_x_waypoint=self.current_local_x
						self.stay_current_y_waypoint=self.current_local_y
						self.positionSp.pose.position.x=self.current_local_x
						self.positionSp.pose.position.y=self.current_local_y
						self.positionSp.pose.position.z = self.indoor_hgt
						
						self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
						quaternion = quaternion_from_euler(0, 0, self.target_yaw)
						self.positionSp.pose.orientation = Quaternion(*quaternion)  
						self.building_param[0] = 1




		rospy.logwarn("EXITING check_and_avoid function")
		return self.avoiding_obstacle


	def building_search_path(self):
    # This function sets up the path for the drone to take inside the building when it is trying to find the missing worker as well as for mapping.
    # After completing one full loop around the building, it decreases search radius by 2 (radius is actually width of square)
		currently_avoiding_obstacle = 0
		obstacle_to_avoid_immediately = 0
		go_back_to_path = 0
		currently_avoiding_obstacle = self.indoor_check_and_avoid()
		rospy.logwarn("Output of check_and_avoid function")
		rospy.logwarn(currently_avoiding_obstacle)
		if currently_avoiding_obstacle != 1 :
			if (self.path_flag == 'Center'):
				if self.building_param[0] == 0:
					rospy.logwarn("Building Parameter 1. Setting yaw")
					self.target_x=self.building_center_x
					self.target_y=self.building_center_y
					self.stay_current_x_waypoint=self.current_local_x
					self.stay_current_y_waypoint=self.current_local_y
					self.positionSp.pose.position.x=self.current_local_x
					self.positionSp.pose.position.y=self.current_local_y
					self.positionSp.pose.position.z = self.indoor_hgt
					self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					self.building_param[0] = 1
				elif self.building_param[1] == 0:
					rospy.logwarn("Building Parameter 2. Waiting on yaw")
					self.positionSp.pose.position.x=self.stay_current_x_waypoint
					self.positionSp.pose.position.y=self.stay_current_y_waypoint
					self.positionSp.pose.position.z = self.indoor_hgt
					if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
						self.positionSp.pose.position.x=self.target_x
						self.positionSp.pose.position.y=self.target_y
						self.positionSp.pose.position.z = self.indoor_hgt
						self.building_param[1] = 1
				elif self.building_param[2] == 0:
					rospy.logwarn("Building Parameter 3. Waiting on location")
					if (abs(self.current_local_x-self.target_x)<3) and (abs(self.current_local_y-self.target_y)<3):
						self.path_flag = 'None'
						self.flagcount=0
						print ("Flag changed in build search")
						rospy.logwarn("I reached Center waypoint")
						self.building_param = [0, 0, 0]

			if (self.path_flag == 'None'):
				if self.building_param[0] == 0:
					rospy.logwarn("Building Parameter 1. Setting yaw")
					self.target_x=self.building_center_x-self.current_r
					self.target_y=self.building_center_y-self.current_r
					self.stay_current_x_waypoint=self.current_local_x
					self.stay_current_y_waypoint=self.current_local_y
					self.positionSp.pose.position.x=self.current_local_x
					self.positionSp.pose.position.y=self.current_local_y
					self.positionSp.pose.position.z = self.indoor_hgt
					self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
					quaternion = quaternion_from_euler(0, 0, self.target_yaw)
					self.positionSp.pose.orientation = Quaternion(*quaternion)  
					self.building_param[0] = 1
				elif self.building_param[1] == 0:
					rospy.logwarn("Building Parameter 2. Waiting on yaw")
					self.positionSp.pose.position.x=self.stay_current_x_waypoint
					self.positionSp.pose.position.y=self.stay_current_y_waypoint
					self.positionSp.pose.position.z = self.indoor_hgt
					if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
						self.positionSp.pose.position.x=self.target_x
						self.positionSp.pose.position.y=self.target_y
						self.positionSp.pose.position.z = self.indoor_hgt
						self.building_param[1] = 1
				elif self.building_param[2] == 0:
					rospy.logwarn("Building Parameter 3. Waiting on location")
					if (abs(self.current_local_x-self.target_x)<3) and (abs(self.current_local_y-self.target_y)<3):
						self.path_flag = 'Right'
						self.flagcount=0
						print ("Flag changed in build search")
						rospy.logwarn("I reached close/right waypoint")
						self.building_param = [0, 0, 0]
			elif (self.path_flag == 'Right'):
				if self.building_param[0] == 0:
					if (self.current_r ==2):
						if (self.flagcount ==0):
							self.righttemp=2
							self.flagcount=self.flagcount+1
						else: 
							self.path_flag= 'Far'
							self.flagcount= 0 
					elif(self.current_r < 9):
						if (self.flagcount ==0):
							self.righttemp=-self.current_r+4
							self.flagcount =self.flagcount+1
						else: 
							self.righttemp=self.righttemp+4
							self.flagcount= self.flagcount +1
						if (abs(self.righttemp)>abs(self.current_r)):
							self.righttemp=0
							self.flagcount=0
							if (self.mappingdirection == 1):

								self.path_flag = 'Far'
							elif (self.mappingdirection == -1):
								self.path_flag = 'Near'
								if (self.inverse_flag > 0):
									self.current_r = self.current_r+2

					if (self.path_flag == 'Right'):
						print('Right Temp')
						print(self.righttemp)
						if (self.mappingdirection ==1):
							self.target_x=self.building_center_x+self.righttemp
							self.target_y=self.building_center_y-self.current_r
						elif(self.mappingdirection == -1):
							self.target_y=self.building_center_y+self.righttemp
							self.target_x=self.building_center_x-self.current_r

						self.stay_current_x_waypoint=self.current_local_x
						self.stay_current_y_waypoint=self.current_local_y
						self.positionSp.pose.position.x=self.current_local_x
						self.positionSp.pose.position.y=self.current_local_y
						self.positionSp.pose.position.z = self.indoor_hgt
						
						self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
						quaternion = quaternion_from_euler(0, 0, self.target_yaw)
						self.positionSp.pose.orientation = Quaternion(*quaternion)  
						self.building_param[0] = 1
				elif self.building_param[1] == 0:
					if (self.path_flag == 'Right'):
						self.positionSp.pose.position.x=self.stay_current_x_waypoint
						self.positionSp.pose.position.y=self.stay_current_y_waypoint
						self.positionSp.pose.position.z = self.indoor_hgt
						if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
							self.positionSp.pose.position.x=self.target_x
							self.positionSp.pose.position.y=self.target_y
							self.positionSp.pose.position.z = self.indoor_hgt
							self.building_param[1] = 1
				elif self.building_param[2] == 0:
					if (self.path_flag == 'Right'):
						if (abs(self.current_local_x-self.target_x)<2.5) and (abs(self.current_local_y-self.target_y)<2.5):
							print ("Flag changed in build search")
							rospy.logwarn("I reached far/right waypoint")
							self.building_param = [0, 0, 0]
			elif (self.path_flag == 'Far'):
				if self.building_param[0] == 0:
					if (self.current_r ==2):
						if (self.flagcount ==0):
							self.fartemp=2
							self.flagcount=self.flagcount+1
						else: 
							self.path_flag= 'Left'
							self.flagcount = 0 
					elif (self.current_r<9):
						if (self.flagcount ==0):
							self.fartemp=-self.current_r+4
							self.flagcount =self.flagcount+1
						else: 
							self.fartemp=self.fartemp+4
							self.flagcount= self.flagcount +1
						if (abs(self.fartemp)>abs(self.current_r)):
							self.fartemp=0
							self.flagcount=0
							if(self.mappingdirection==1):
								self.path_flag = 'Left'
							elif(self.mappingdirection == -1):
								self.path_flag = 'Right'
								self.inverse_flag = self.inverse_flag+1
					if (self.path_flag == 'Far'):
						print('FAR Temp')
						print(self.fartemp)
						if (self.mappingdirection ==1):
							self.target_x=self.building_center_x+self.current_r
							self.target_y=self.building_center_y+self.fartemp
						elif(self.mappingdirection == -1):
							self.target_y=self.building_center_y-self.current_r
							self.target_x=self.building_center_x-self.fartemp
						self.stay_current_x_waypoint=self.current_local_x
						self.stay_current_y_waypoint=self.current_local_y
						self.positionSp.pose.position.x=self.current_local_x
						self.positionSp.pose.position.y=self.current_local_y
						self.positionSp.pose.position.z = self.indoor_hgt
						
						self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
						quaternion = quaternion_from_euler(0, 0, self.target_yaw)
						self.positionSp.pose.orientation = Quaternion(*quaternion)  
						self.building_param[0] = 1
				elif self.building_param[1] == 0:
					self.positionSp.pose.position.x=self.stay_current_x_waypoint
					self.positionSp.pose.position.y=self.stay_current_y_waypoint
					self.positionSp.pose.position.z = self.indoor_hgt
					if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
						self.positionSp.pose.position.x=self.target_x
						self.positionSp.pose.position.y=self.target_y
						self.positionSp.pose.position.z = self.indoor_hgt
						self.building_param[1] = 1
				elif self.building_param[2] == 0:
					if (abs(self.current_local_x-self.target_x)<2.5) and (abs(self.current_local_y-self.target_y)<2.5):
						print ("Flag changed in build search")
						rospy.logwarn("I reached far/left waypoint")
						self.building_param = [0, 0, 0]
			elif (self.path_flag == 'Left'):

				if self.building_param[0] == 0:
					if self.building_param[0] == 0:
						if (self.current_r ==2):
							if (self.flagcount ==0):
								self.lefttemp=2
								self.flagcount=self.flagcount+1
							else: 
								self.path_flag= 'Near'
								self.flagcount = 0 
						elif (self.current_r < 9):
							if (self.flagcount ==0):
								self.lefttemp=-self.current_r+4
								self.flagcount =self.flagcount+1
							else: 
								self.lefttemp=self.lefttemp+4
								self.flagcount= self.flagcount +1
							if (abs(self.lefttemp)>abs(self.current_r)):
								self.lefttemp=0
								self.flagcount=0
								if(self.mappingdirection == 1):
									self.path_flag = 'Near'
								elif (self.mappingdirection == -1):
									self.path_flag = 'Far'	
						if (self.path_flag == 'Left'):
							print('left Temp')
							print(self.fartemp)
							if (self.mappingdirection ==1):
								self.target_x=self.building_center_x-self.lefttemp
								self.target_y=self.building_center_y+self.current_r
							elif (self.mappingdirection == -1):
								self.target_y=self.building_center_y-self.lefttemp
								self.target_x=self.building_center_x+self.current_r

							self.stay_current_x_waypoint=self.current_local_x
							self.stay_current_y_waypoint=self.current_local_y
							self.positionSp.pose.position.x=self.current_local_x
							self.positionSp.pose.position.y=self.current_local_y
							self.positionSp.pose.position.z = self.indoor_hgt
							
							self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
							quaternion = quaternion_from_euler(0, 0, self.target_yaw)
							self.positionSp.pose.orientation = Quaternion(*quaternion)  
							self.building_param[0] = 1
				elif self.building_param[1] == 0:
					self.positionSp.pose.position.x=self.stay_current_x_waypoint
					self.positionSp.pose.position.y=self.stay_current_y_waypoint
					self.positionSp.pose.position.z = self.indoor_hgt
					if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
						self.positionSp.pose.position.x=self.target_x
						self.positionSp.pose.position.y=self.target_y
						self.positionSp.pose.position.z = self.indoor_hgt
						self.building_param[1] = 1
				elif self.building_param[2] == 0:
					if (abs(self.current_local_x-self.target_x)<2.5) and (abs(self.current_local_y-self.target_y)<2.5):
						print ("Flag changed in build search")
						rospy.logwarn("I reached close/left waypoint")
						self.building_param = [0, 0, 0]
			elif (self.path_flag == 'Near'):
				if self.building_param[0] == 0:
					if (self.current_r ==2):
						if (self.flagcount ==0):
							self.neartemp=2
							self.flagcount=self.flagcount+1
						else: 
							self.path_flag= 'Right'
							self.flagcount = 0 
							self.current_r=self.current_r+2
					elif(self.current_r <9):
						if (self.flagcount ==0):
							self.neartemp=-self.current_r+4
							self.flagcount =self.flagcount+1
						else: 
							self.neartemp=self.neartemp+4
							self.flagcount= self.flagcount +1
						if (abs(self.neartemp)>abs(self.current_r)):
							self.neartemp=0
							self.flagcount=0
							if(self.mappingdirection == 1):
								self.path_flag = 'Right'
								self.current_r=self.current_r+2
							elif(self.mappingdirection == -1):
								self.path_flag = 'Left'
					if (self.path_flag == 'Near'):
						print('Near Temp')
						print(self.neartemp)
						if (self.mappingdirection ==1):
							self.target_x=self.building_center_x-self.current_r
							self.target_y=self.building_center_y-self.neartemp
						elif(self.mappingdirection ==-1):
							self.target_y=self.building_center_y+self.current_r
							self.target_x=self.building_center_x+self.neartemp

						self.stay_current_x_waypoint=self.current_local_x
						self.stay_current_y_waypoint=self.current_local_y
						self.positionSp.pose.position.x=self.current_local_x
						self.positionSp.pose.position.y=self.current_local_y
						self.positionSp.pose.position.z = self.indoor_hgt
						
						self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
						quaternion = quaternion_from_euler(0, 0, self.target_yaw)
						self.positionSp.pose.orientation = Quaternion(*quaternion)  
						self.building_param[0] = 1
				elif self.building_param[1] == 0:
					self.positionSp.pose.position.x=self.stay_current_x_waypoint
					self.positionSp.pose.position.y=self.stay_current_y_waypoint
					self.positionSp.pose.position.z = self.indoor_hgt
					if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
						self.positionSp.pose.position.x=self.target_x
						self.positionSp.pose.position.y=self.target_y
						self.positionSp.pose.position.z = self.indoor_hgt
						self.building_param[1] = 1

				elif self.building_param[2] == 0:
					if (abs(self.current_local_x-self.target_x)<2.5) and (abs(self.current_local_y-self.target_y)<2.5):
						print ("Flag changed in build search")
						rospy.logwarn("I reached close/right waypoint again. Resetting with a closer radius")
						self.building_param = [0, 0, 0]
			
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

	#Subscriber: tag_localization setpoints
	rospy.Subscriber("/detected_red_tag", PoseStamped, K.redtagPoseCb)
	rospy.Subscriber("/detected_blue_tag", PoseStamped, K.bluetagPoseCb)
	rospy.Subscriber("/detected_green_tag", PoseStamped, K.greentagPoseCb)
	# Subscriber: Worker Images
	rospy.Subscriber("/down/camera/color/image_raw", Image, K.WorkerImageCb)
	# Subscriber: Color Tag Images:
	rospy.Subscriber("/front/camera/color/image_raw", Image, K.TagImageCb)

	####################################################################################################
	#--------------------------------------OUTDOOR AVOIDANCE-------------------------------------------#
	####################################################################################################
	rospy.Subscriber("mavros/local_position/pose", PoseStamped, K.transformationoffcu)
	# FOR RPLIDAR
	# USE /scan FOR ACTUAL TESTING!!! USE /laser/scan FOR SIMULATION
	rospy.Subscriber("/scan", LaserScan, K.rangessCb)#################################################################################################################################################################################################################
	####################################################################################################

	########## Publishers ##########

	# Publisher: PositionTarget
	# avoid_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
	# rate = rospy.Rate(10.0)

	servo_pub = rospy.Publisher("/servo", UInt16, queue_size=10)

	# Do initial checks
	while (K.current_lat*K.current_lon*K.current_alt) == 0 and not rospy.is_shutdown():

		rospy.loginfo('Waiting for current gps location to execute setWaypoints_and_FenceCb') # Initializes waypoints and fence in local x,y,z and checks to see if they make sense
		K.rate.sleep()

	K.setWayoints_and_Fence()

	K.resetStates()

	# We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
	k=0
	while k<10:
		K.avoid_pub.publish(K.positionSp)
		K.rate.sleep()
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
	elif K.start_state == 'WAYPOINT4':
		K.WAYPOINT4 = 1
	elif K.start_state == 'WORKER1SEARCH':
		K.WORKER1SEARCH = 1
	elif K.start_state == 'LOADPACKAGE2':
		K.LOADPACKAGE2 = 1
	elif K.start_state == 'GOTOBUILDING':
		K.GOTOBUILDING = 1
	elif K.start_state == 'TAGSEARCH':
		K.TAGSEARCH = 1
	elif K.start_state == 'ENTERBUILDING':
		K.ENTERBUILDING = 1
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
		#if (K.isTooCloseToFence()== True):
		#	rospy.logwarn('Changing to Hover mode because vehicle was too close to boundary')
		#	K.resetStates()
		#	K.HOVER = 1
			# PUT HOVER POINTS HERE SO THAT THEY DON'T CONSTANTLY GET UPDATED AND THE VEHICLE DRIFTS
		#	K.positionSp.pose.position.x = K.current_local_x 
		#	K.positionSp.pose.position.y = K.current_local_y 
		#	K.positionSp.pose.position.z = K.current_local_z
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
			K.verifyTag()
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
			K.verifyTag()
			angle = atan2((K.waypoint1_y-K.current_local_y),(K.waypoint1_x-K.current_local_x))			
			K.check_and_avoid(angle)
			
			if (K.minfront>7):
				waypoint1_is_done = K.Execute_Waypoint(K.waypoint1_x, K.waypoint1_y, K.waypoint1_z, -1000)

				if waypoint1_is_done:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached waypoint 1")
					K.resetStates()
					K.WAYPOINT2 = 1
					K.current_signal = 'Reached WP1'

		elif K.WAYPOINT2:
			K.current_state = 'WAYPOINT2'
			K.current_signal = 'Running'
			rospy.loginfo("Heading to WP2")
			angle = atan2((K.waypoint2_y-K.current_local_y),(K.waypoint2_x-K.current_local_x))
			K.check_and_avoid(angle)
			K.verifyTag()

			if (K.minfront>7):

				waypoint2_is_done = K.Execute_Waypoint(K.waypoint2_x, K.waypoint2_y, K.waypoint2_z, -1000)

				if waypoint2_is_done:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached waypoint 2")
					K.resetStates()
					K.WAYPOINT3 = 1
					K.current_signal = 'Reached WP2'

		elif K.WAYPOINT3:
			K.current_state = 'WAYPOINT3'
			K.current_signal = 'Running'
			rospy.loginfo("Heading to WP3")
			angle = atan2((K.waypoint3_y-K.current_local_y),(K.waypoint3_x-K.current_local_x))
			K.check_and_avoid(angle)
			K.verifyTag()

			if (K.minfront>7 ):

				waypoint3_is_done = K.Execute_Waypoint(K.waypoint3_x, K.waypoint3_y, K.waypoint3_z, -1000)

				if waypoint3_is_done:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached waypoint 3")
					K.resetStates()
					K.WAYPOINT4 = 1
					K.current_signal = 'Reached WP3'

		elif K.WAYPOINT4:
			K.current_state = 'WAYPOINT4'
			K.current_signal = 'Running'
			rospy.loginfo("Heading to WP4")
			angle = atan2((K.waypoint4_y-K.current_local_y),(K.waypoint4_x-K.current_local_x))
			K.check_and_avoid(angle)
			K.verifyTag()
			if (K.minfront>7 ):

				waypoint4_is_done = K.Execute_Waypoint(K.waypoint4_x, K.waypoint4_y, K.waypoint4_z, -1000)

				if waypoint4_is_done:
					rospy.loginfo("Current position close enough to desired waypoint")
					rospy.loginfo("Reached waypoint 4")
					K.positionSp.header.frame_id = "local_origin"
					K.positionSp.pose.position.z = K.worker1_search_z
					K.resetStates()
					K.WORKER1SEARCH = 1
					K.current_signal = 'Reached WP4'

		elif K.WORKER1SEARCH:
			# Search algorithm is as follows: Drone follows a square pattern starting at the 
			# closest right corner, goes down field on right side, turn left at the end of 
			# the field, and comes back on left side. The search path side closest to the 
			# control station is basically at the same distance downfield as the building. 
			# Since the field is 30 meters wide, in order to span the entire field, the drone
			# should search at about 8 meters high and 7.5 meters inward from the field boundaries.
			K.current_state = 'WORKER1SEARCH'
			K.current_signal = 'Running'
			rospy.loginfo("Searching for outside worker")
			K.verifyTag()
			if K.worker1_found_flag:
				K.current_signal = 'Outside Worker Found'
				rospy.loginfo('Setting worker position to go to')
				K.positionSp.header.frame_id = 'local_origin'
				K.positionSp.pose.position.x = K.worker1Sp.pose.position.x
				K.positionSp.pose.position.y = K.worker1Sp.pose.position.y
				K.positionSp.pose.position.z = K.worker1_search_z

				desired_yaw = atan2((K.worker1Sp.pose.position.y-K.current_local_y),(K.worker1Sp.pose.position.x-K.current_local_x))
				quaternion_yaw = quaternion_from_euler(0, 0, desired_yaw)
				K.positionSp.pose.orientation = Quaternion(*quaternion_yaw)

				K.resetStates()
				K.DELIVERAID1 = 1

			else:
				# EXECUTE WORKER SEARCH WAYPOINTS. IF CAN'T FIND ANYTHING, DO IT OVER AGAIN

				# First make sure that drone is high enough so that it won't run into the building
				if K.worker1_search_WP_FLAG == -1:
					angle = atan2((K.entrance_search_corner1_y-K.current_local_y),(K.entrance_search_corner1_x-K.current_local_x))			
					K.check_and_avoid(angle)
			
					if (K.minfront>7):
						head_to_avoid_building1 = K.Execute_Waypoint(K.entrance_search_corner1_x,K.entrance_search_corner1_y, K.worker1_search_z, -1000)

						if head_to_avoid_building1:
							K.worker1_search_WP_FLAG = 0

				if K.worker1_search_WP_FLAG == 0:
					head_to_avoid_building2 = K.Execute_Waypoint(K.entrance_search_corner4_x,K.entrance_search_corner4_y, K.worker1_search_z, -1000)					

					if head_to_avoid_building2:
						K.worker1_search_WP_FLAG = 1

				if K.worker1_search_WP_FLAG == 1:
					waypoint1_is_done = K.Execute_Waypoint(K.worker1_search_WP1_x, K.worker1_search_WP1_y, K.worker1_search_z, -1000)

					if waypoint1_is_done:
						rospy.loginfo("Current position close enough to worker search WP1.\nHeading to worker search WP2")
						K.worker1_search_WP_FLAG = 2

						# #---- For finding GPS location of this point -- Simulation only
						# K.current_signal = 'Reached worker search waypoint 1'
						# ###############################################################

				if K.worker1_search_WP_FLAG == 2:
					waypoint2_is_done = K.Execute_Waypoint(K.worker1_search_WP2_x, K.worker1_search_WP2_y, K.worker1_search_z, -1000)

					if waypoint2_is_done:
						rospy.loginfo("Current position close enough to worker search WP2.\nHeading to worker search WP3")
						K.worker1_search_WP_FLAG = 3 

						# #---- For finding GPS location of this point -- Simulation only
						# K.current_signal = 'Reached worker search waypoint 2'
						# ###############################################################

				if K.worker1_search_WP_FLAG == 3:
					waypoint3_is_done = K.Execute_Waypoint(K.worker1_search_WP3_x, K.worker1_search_WP3_y, K.worker1_search_z, -1000)

					if waypoint3_is_done:
						rospy.loginfo("Current position close enough to worker search WP3.\nHeading to worker search WP4")
						K.worker1_search_WP_FLAG = 4 

						# #---- For finding GPS location of this point -- Simulation only
						# K.current_signal = 'Reached worker search waypoint 3'
						# ###############################################################

				if K.worker1_search_WP_FLAG == 4:
					waypoint4_is_done = K.Execute_Waypoint(K.worker1_search_WP4_x, K.worker1_search_WP4_y, K.worker1_search_z, -1000)

					if waypoint4_is_done:
						rospy.loginfo("Current position close enough to worker search WP4.\nHeading to worker search WP5")
						K.worker1_search_WP_FLAG = 5 

						# #---- For finding GPS location of this point -- Simulation only
						# K.current_signal = 'Reached worker search waypoint 4'
						# ###############################################################

				if K.worker1_search_WP_FLAG == 5:
					waypoint5_is_done = K.Execute_Waypoint(K.worker1_search_WP5_x, K.worker1_search_WP5_y, K.worker1_search_z, -1000)

					if waypoint5_is_done:
						rospy.loginfo("Current position close enough to worker search WP5.\nHeading to worker search WP6")
						K.worker1_search_WP_FLAG = 6 

						# #---- For finding GPS location of this point -- Simulation only
						# K.current_signal = 'Reached worker search waypoint 5'
						# ###############################################################

				if K.worker1_search_WP_FLAG == 6:
					waypoint6_is_done = K.Execute_Waypoint(K.worker1_search_WP6_x, K.worker1_search_WP6_y, K.worker1_search_z, -1000)

					if waypoint6_is_done:
						rospy.loginfo("Current position close enough to worker search WP6.\nHeading to worker search WP7")
						K.worker1_search_WP_FLAG = 7 

						# #---- For finding GPS location of this point -- Simulation only
						# K.current_signal = 'Reached worker search waypoint 6'
						# ###############################################################

				if K.worker1_search_WP_FLAG == 7:
					waypoint7_is_done = K.Execute_Waypoint(K.worker1_search_WP7_x, K.worker1_search_WP7_y, K.worker1_search_z, -1000)

					if waypoint7_is_done:
						rospy.loginfo("Current position close enough to worker search WP7.\nHeading to worker search WP8")
						K.worker1_search_WP_FLAG = 8 

						# #---- For finding GPS location of this point -- Simulation only
						# K.current_signal = 'Reached worker search waypoint 7'
						# ###############################################################

				if K.worker1_search_WP_FLAG == 8:
					waypoint8_is_done = K.Execute_Waypoint(K.worker1_search_WP8_x, K.worker1_search_WP8_y, K.worker1_search_z, -1000)

					if waypoint8_is_done:
						rospy.loginfo("Current position close enough to worker search WP8.\nHeading to worker search WP1")
						K.worker1_search_WP_FLAG = 1 
						rospy.logwarn("Could not find worker. Starting over again")

						# #---- For finding GPS location of this point -- Simulation only
						# K.current_signal = 'Reached worker search waypoint 8'
						# ###############################################################

						# #----------THIS IS FOR SIMULATION PURPOSES ONLY TO SPOOF WORKER FOUND-------------------------#
						# ###############################################################################################
						# K.verifyPOI_flag = 1
						# K.worker1_found_flag = 1
						# K.worker1Sp.pose.position.x = K.current_local_x - 2
						# K.worker1Sp.pose.position.y = K.current_local_y - 5
						# K.worker1Sp.pose.position.z = 0
						# ###############################################################################################

		elif K.DELIVERAID1:
			K.current_state = 'DELIVERAID1'
			K.current_signal = 'Running'
			rospy.loginfo('Trying to reach outside worker')

			# angle = atan2((K.waypoint1_y-K.current_local_y),(K.waypoint1_x-K.current_local_x))			
			# K.check_and_avoid(angle)
			angle = atan2((K.worker1Sp.pose.position.y-K.current_local_y),(K.worker1Sp.pose.position.x-K.current_local_x))
			K.check_and_avoid(angle)
			K.verifyTag()
			if (K.minfront>7 ):
				K.positionSp.pose.position.x = K.worker1Sp.pose.position.x
				K.positionSp.pose.position.y = K.worker1Sp.pose.position.y
				K.positionSp.pose.position.z = K.deliver_aid1_z
				if (abs(K.current_local_x - K.worker1Sp.pose.position.x)<.1 and abs(K.current_local_y - K.worker1Sp.pose.position.y)<.1):
					servo_pub.publish(K.RELEASE)
					rospy.sleep(1)
					K.ready2captureworker1 = 0
					rospy.loginfo("Aid Dropped")
					K.current_signal = 'Aid Delivered'
					K.resetStates()
					#K.TAGSEARCH = 1
					K.GOTOREFULE = 1
					# Find closest point to go to
					distance2one = sqrt((K.current_local_x-K.entrance_search_corner1_x) ** 2 + (K.current_local_y-K.entrance_search_corner1_y) ** 2)
					distance2four = sqrt((K.current_local_x-K.entrance_search_corner4_x) ** 2 + (K.current_local_y-K.entrance_search_corner4_y) ** 2)
					if distance2one < distance2four:
						K.closest_point_to_return = 1
					else:
						K.closest_point_to_return = 4

		elif K.GOTOREFULE:
			K.current_state = 'GOTOREFULE'
			K.current_signal = 'Running'
			rospy.loginfo('Headed to landing zone to refule')
			K.verifyTag()
			if K.closest_point_to_return == 1: 

				angle = atan2((K.landing_zone_y-K.current_local_y),(K.landing_zone_x-K.current_local_x))			
				K.check_and_avoid(angle)

				if (K.minfront>7):
					heading_to_landing = K.Execute_Waypoint(K.landing_zone_x,K.landing_zone_y,K.worker1_search_z, -1000)
				
					if heading_to_landing:
						K.resetStates()
						K.current_signal = 'Reached Landing Zone'
						K.REFULE = 1

			elif K.closest_point_to_return == 4: 
				if K.go_to_refule == 1:

					heading_to_right_side = K.Execute_Waypoint(K.entrance_search_corner4_x,K.entrance_search_corner4_y,K.worker1_search_z, -1000)
					
					if heading_to_right_side:
						K.go_to_refule = 2
				
				elif K.go_to_refule == 2:

					heading_to_corner1 = K.Execute_Waypoint(K.entrance_search_corner1_x,K.entrance_search_corner1_y,K.worker1_search_z, -1000)					
					if heading_to_corner1:
						K.go_to_refule = 3

				elif K.go_to_refule == 3:
					angle = atan2((K.landing_zone_y-K.current_local_y),(K.landing_zone_x-K.current_local_x))			
					K.check_and_avoid(angle)

					if (K.minfront>7):
						heading_to_landing = K.Execute_Waypoint(K.landing_zone_x,K.landing_zone_y,K.worker1_search_z, -1000)
						if heading_to_landing:
							K.resetStates()
							K.current_signal = 'Reached Landing Zone'
							K.REFULE = 1

		elif K.REFULE:
			K.current_state = 'REFULE'
			K.current_signal = 'Running'
			rospy.loginfo('Vehicle is landing to change battery and load next aid kit')
			K.modes.setAutoLandMode()
			K.verifyTag()
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
			K.verifyTag()
			for K.n.data in range(K.RELEASE.data,K.HOLD.data):
				if(rospy.is_shutdown()):
					continue
				servo_pub.publish(K.n)
				rospy.sleep(0.1)
			K.resetStates()
			K.TAKEOFF2 = 1
			K.current_signal = 'Package Loaded'

		# !!!!!!!!!!!!!!!!!!!!! SAFETY PILOT Will ARM AND PUT BACK INTO OFFBOARD MODE AGAIN HERE !!!!!!!!!!!!!!!!!!!!!!!!!!
		# !!!!!!!!!!!!!!!!!!!!! MAY ALSO WANT TO CHANGE MAX_XY_VEL HERE TOO FROM 1.5 TO 1.0 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		elif K.TAKEOFF2:
			K.current_state = 'TAKEOFF2'
			K.current_signal = 'Running'
			rospy.logwarn('Vehicle is taking off again')
			K.verifyTag()
			K.positionSp.header.frame_id = 'local_origin' # IS THIS NEEDED?
			K.positionSp.pose.position.x = K.current_local_x 
			K.positionSp.pose.position.y = K.current_local_y
			K.positionSp.pose.position.z = K.entrance_search_z # Should be 1
			K.positionSp.pose.orientation.w = 1.0	# IS THIS NEEDED?

			quaternion_yaw = quaternion_from_euler(0, 0, K.yaw_top_z)
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
			K.verifyTag()
			reached_building = K.Execute_Waypoint(K.entrance_search_corner1_x, K.entrance_search_corner1_y, K.entrance_search_z, -1000)
			# reached_building = K.Execute_Waypoint(K.entrance_search_corner1_x, K.entrance_search_corner1_y, 7, -1000)
			if reached_building:
				K.current_signal = 'Reached Building'
				K.resetStates()
				K.TAGSEARCH = 1

		elif K.TAGSEARCH:
			K.current_state = 'TAGSEARCH'
			K.current_signal = 'Searching for tags'
			K.verifyTag()
			rospy.loginfo('Searching for entrance')

			rospy.logwarn('Found green tag?')
			rospy.logwarn(K.green_tag.found)
			# rospy.logwarn('Completed full rev?')
			# rospy.logwarn(K.completed_full_rev)
			if (K.green_tag.found == 1): # and (K.completed_full_rev == 1):
				rospy.loginfo('Unblocked entrance successfully found!')
				K.resetStates()
				# K.ENTERBUILDING = 1
				K.WORKER2SEARCH = 1 # Bypass enter building state
				K.current_signal = 'Found Green Tag'

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

			elif (K.green_tag.found == 0): # and (K.completed_full_rev == 1):
				if K.entrance_number_search == 1:	# This variable is set in the EntranceSearch code
					K.EntranceSearch(K.building_entr1_x,K.building_entr1_y,K.yaw_building_entr1)
					# rospy.loginfo('Checking first entrance')
					rospy.loginfo('Checking for tags')
					# K.current_signal = 'Searching first entrance'
					# -------------------------------FOR SIMULATION PURPOSES ONLY----------------------------------#
					################################################################################################
					# K.verifyTAG_flag = 1
					# K.tag_color = 'Green'
					# K.green_tag.x = 10
					# K.green_tag.y = -6
					# K.greentagSp.pose.position.x = 10
					# K.greentagSp.pose.position.y = -7
					################################################################################################

				elif K.entrance_number_search == 2:
					K.EntranceSearch(K.building_entr2_x,K.building_entr2_y,K.yaw_building_entr2)
					rospy.loginfo('Checking second entrance')
					# K.current_signal = 'Searching second entrance'
				elif K.entrance_number_search == 3:
					rospy.loginfo('Checking third entrance')
					# K.current_signal = 'Searching third entrance'
					K.EntranceSearch(K.building_entr3_x,K.building_entr3_y,K.yaw_building_entr3)
				elif K.entrance_number_search == 4:
					rospy.loginfo('Checking fourth entrance')
					# K.current_signal = 'Searching fourth entrance'
					K.EntranceSearch(K.building_entr4_x,K.building_entr4_y,K.yaw_building_entr4)
				else:
					K.entrance_number_search = 1 # So that vehicle will check first entrance again
					K.completed_full_rev = 1
					rospy.loginfo('Still cannot find unblocked entrance. Try again at first entrance')
			# else:
			# 	# Execute building scan until full revolution
			# 	K.BuildingScan()

		elif K.ENTERBUILDING:
			K.current_state = 'ENTERBUILDING'
			K.current_signal = 'Running'
			rospy.loginfo("Attempting to enter building")
			rospy.logwarn("Correct Entrance to Enter")
			rospy.logwarn(K.enter_bldg_flag)
			K.verifyTag()
			# AS CODE IS WRITTEN, IT WILL GO TO WHERE THE DRONE WAS (WITH SAME ORIENTATION) WHEN IT DETECTED THE GREEN TAG, NOT THE DETECTED LOCATION OF THE GREEN TAG
			
			# ---------------------------- FOR SIMULATION ONLY -----------------------------------#
			#######################################################################################
			# K.unblocked_entrance_x = K.building_entr1_x
			# K.unblocked_entrance_y = K.building_entr1_y
			#######################################################################################

			if K.enter_bldg_flag == 0:
				#added this yaw better than using building directions

				target_yaw = math.atan2((K.unblocked_entrance_y-K.current_local_y),(K.unblocked_entrance_x-K.current_local_x))
				reached_unblocked_entrance = K.Execute_Waypoint(K.unblocked_entrance_x,K.unblocked_entrance_y, K.enter_bldg_hgt, K.yaw_top_z)
				if reached_unblocked_entrance:
					K.enter_bldg_flag = 1

			elif K.enter_bldg_flag == 1:
				K.Execute_EnterBuilding(K.yaw_top_z)
			elif K.enter_bldg_flag ==2:
				K.resetStates()
				K.current_signal = 'Entered Building'
				K.WORKER2SEARCH = 1
				K.enter_bldg_flag = 0
				# K.mapping.start()

		elif K.WORKER2SEARCH:
			K.current_state = 'WORKER2SEARCH'
			K.current_signal = 'Running'
			# print("Building Center")
			# print(K.building_center_x)
			# print(K.building_center_y)
			# print("current radius")
			# print (K.current_r)
			# print ("current side")
			# print (K.path_flag)
			K.verifyTag()
			rospy.logwarn('Vehicle searching for missing worker inside')

			if K.worker2_found_flag:
				K.current_signal = 'Inside Worker Found'
				rospy.logwarn('Setting worker position to go to')

				K.positionSp.header.frame_id = 'local_origin'
				K.positionSp.pose.position.x = K.worker2Sp.pose.position.x
				K.positionSp.pose.position.y = K.worker2Sp.pose.position.y
				K.positionSp.pose.position.z = K.indoor_hgt
				K.resetStates()
				K.DELIVERAID2 = 1
				# K.mapping.shutdown()
			else:
				K.building_search_path()
				if K.current_r > 7:
					K.current_r = 2
					K.path_flag = 'Center'

		elif K.DELIVERAID2:
			K.current_state = 'DELIVERAID2'
			K.current_signal = 'Running'
			rospy.logwarn('Delivering the first aid kit to inside worker')
			#Execute DELIVER AID 2
			K.verifyTag()
			if (abs(K.current_local_x - K.positionSp.pose.position.x)<.1 and abs(K.current_local_y - K.positionSp.pose.position.y)<.1):
				servo_pub.publish(K.RELEASE)
				rospy.sleep(1)
				K.ready2captureworker2 = 0
				rospy.loginfo("Aid Dropped")
				K.current_signal = 'Aid Delivered'
				K.resetStates()
				# K.MAPPING = 1
				K.GOHOME = 1
				K.path_flag = 'Right'
				K.current_r = 6
				K.building_param = [0, 0, 0]

		elif K.MAPPING:
			K.verifyTag()
			############################## THESE ARE THE MAPPING VARIABLES ###########################################
			if(K.tagflag == 1):
				# K.mapping.start()
				K.tagflag =0 
			 # ------>>>>>>>>>> WHEN YOU WANT TO START MAPPING TYPE THIS
				rospy.loginfo("Mapping started")
			#  ----->>>>>>>>>> WHEN YOU WANT TO STOP MAPPING TYPE THIS
			##########################################################################################################
			#NODE FOR MAPPING HERE 
			K.current_state = 'MAPPING'
			K.current_signal = 'Running'
			rospy.loginfo('Finishing mapping of inside of tent')
	 		print(K.flagcount)
			if (K.tagflag == 2):
				if (K.current_r == 6):
					K.building_search_path()
					print('current side')
					print(K.path_flag)
					print ('radius')
					print(K.current_r)
					print('target x')
					print(K.positionSp.pose.position.x)
					print('target y')
					print(K.positionSp.pose.position.y)
					print('mapping direction')
					print(K.mappingdirection)
				elif (K.current_r != 6):
					K.tagflag = 3
					K.current_r=6
					K.path_flag='Right'
			elif (K.tagflag == 3):
				if (K.current_r == 6):
					K.building_search_path ()
					print('current side')
					print(K.path_flag)
					print ('radius')
					print(K.current_r)
					print('target x')
					print(K.target_x)
					print('target y')
					print(K.target_y)
					print('mapping direction')
					print(K.mappingdirection)
				elif (K.current_r > 6):
					K.tagflag = 4
					K.current_r=6
			elif (K.tagflag==4):
				K.tagflag = 5
				rospy.loginfo("FINISHED MApping")
				K.resetStates()
				K.EXITBUILDING = 1
				K.current_signal = 'Mapping Finished'
				K.mapping.shutdown()

			

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
			exiting_x=0
			exiting_y=0
			print(K.exit_bldg_flag)
			K.verifyTag()
			if K.exit_bldg_flag == 0:
				print('here')
				if(K.unblocked_entrance == 1):

					exiting_x=K.building_entr1_inside_x
					exiting_y=K.building_entr1_inside_y
					print(exiting_x)
					print(exiting_y)

				elif (K.unblocked_entrance == 2):
					exiting_x=K.building_entr2_inside_x
					exiting_y=K.building_entr2_inside_y

				elif (K.unblocked_entrance == 3):
					exiting_x=K.building_entr3_inside_x
					exiting_y=K.building_entr3_inside_y

				elif (K.unblocked_entrance == 4):
					exiting_x=K.building_entr4_inside_x
					exiting_y=K.building_entr4_inside_y

				target_yaw = math.atan2((exiting_y-K.current_local_y),(exiting_x-K.current_local_x))
				currently_avoiding_obstacle = K.indoor_check_and_avoid()
				rospy.logwarn("Output of check_and_avoid function")
				rospy.logwarn(currently_avoiding_obstacle)
				if currently_avoiding_obstacle != 1 :
					reached_unblocked_entrance = K.Execute_Waypoint(exiting_x,exiting_y, K.enter_bldg_hgt, target_yaw)

					if reached_unblocked_entrance:
						K.exit_bldg_flag = 1

			elif K.exit_bldg_flag == 1:
				K.Execute_EnterBuilding(target_yaw)
			elif K.exit_bldg_flag ==2:
				K.resetStates()
				K.current_signal = 'Exited Building'
				K.GOHOME = 1

		elif K.GOHOME:
			K.verifyTag()
			K.current_state = 'MAPPING AND THEN GOHOME'
			K.current_signal = 'Running'
			rospy.loginfo('MAPPING AND THEN GOING HOME')
			# K.modes.setReturnToHome() # Not sure if this works. It didn't work in simulation
			reached_home = K.Execute_Waypoint(K.landing_zone_x,K.landing_zone_y,K.takeoff_height, -1000)

			if reached_home:   # Rules give a 3m radius from goal
				rospy.loginfo('Reached home')
				K.current_signal = 'Reached Home'
				K.resetStates()
				K.LAND = 1

		elif K.LAND:
			K.verifyTag()
			K.current_state = 'LAND'
			K.current_signal = 'Running'
			rospy.loginfo('Vehicle is landing')
			K.modes.setAutoLandMode()
			if K.IS_LANDED:
				K.modes.setDisarm()
				K.resetStates()
				K.current_signal = 'Landed'

		elif K.HOVER:
			K.verifyTag()
			K.current_state = 'HOVER'
			K.current_signal = 'Running'
			rospy.logwarn('Vehicle in Hover mode until something else happens')
			# NEED TO FIGURE OUT HOW TO EXIT THIS STATE

		K.VehicleNavLog()
		rospy.loginfo("Heading to x")
		rospy.loginfo(K.positionSp.pose.position.x)
		rospy.loginfo("Heading to y")
		rospy.loginfo(K.positionSp.pose.position.y)
		rospy.loginfo("Current State")
		rospy.loginfo(K.current_state)
		if K.already_set_worker1_pose == 0:
			rospy.logwarn("Position of Outside Worker was set")
		K.avoid_pub.publish(K.positionSp)
		K.rate.sleep()	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
