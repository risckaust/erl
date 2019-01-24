#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion
import tf
import numpy as np
import math
from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import NavSatFix, LaserScan
from geometry_msgs.msg import Point, PoseStamped,  Quaternion
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
import tf 


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

    #Waypoints GPS Coordiantes

    # self.home_lat = ENTER_HOME_COORDINATES
    # self.home_lon = ENTER_HOME_COORDINATES # THIS WILL BE NEEDED EVENTUALLY BECAUSE THERE WILL BE TWO TAKEOFF/LANDING LOCATIONS

    self.waypoint1_lat = 22.3118631#22.3070405#22.317490
    self.waypoint1_lon = 39.0952322#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
    self.waypoint1_alt = 2.5

    self.waypoint2_lat = 22.3118631#22.3070405#22.317490
    self.waypoint2_lon = 39.0952322#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
    self.waypoint2_alt = 2.5

    self.waypoint3_lat = 22.3118631#22.3070405#22.317490
    self.waypoint3_lon = 39.0952322#39.1047228#39.097909 # Location of Testing Field at KAUST - a little ways down field
    self.waypoint3_alt = 2.5

    # Current local ENU coordinates
    self.current_local_x = 0.0
    self.current_local_y = 0.0
    self.current_local_z = 0.0
    #Waypoints ENU Coordinates
    self.home_x = 0
    self.home_y = 0
    self.home_z = 0


    self.waypoint1_x = 0
    self.waypoint1_y = 25
    self.waypoint1_z = 40

    self.waypoint2_x = 0
    self.waypoint2_y = 0
    self.waypoint2_z = 40

    self.waypoint3_x = 0
    self.waypoint3_y = 0
    self.waypoint3_z = 40

    self.takeoff_height = 2.5

    # States
    self.TAKEOFF = 0
    self.WAYPOINT1 = 0
    self.WAYPOINT2 = 0
    self.WAYPOINT3 = 0
    self.GOHOME = 0
    self.LAND = 0
    self.HOVER = 0
    self.TENT=0 
    ####################AVOIDANCE PARAMETERS+INDOOR PARAMETERS #####################################################

    self.current_yaw = 0
    self.target_yaw = 0
    self.avoidancepar=1

    self.dx=0
    self.dy=0
    
    self.path_flag = 'None'
    self.building_param = [0, 0, 0]
    self.current_r = 8

    self.tf_listener = tf.TransformListener()


    self.positionSp = PoseStamped()
    yaw_degrees = 0  # North
    yaw = math.radians(yaw_degrees)
    quaternion = quaternion_from_euler(0, 0, yaw)

    self.modes = fcuModes()

    self.front=np.zeros(60)
    self.left=np.zeros(30)
    self.right=np.zeros(30)
    self.back=np.zeros(60)

    self.minfront=min(abs(self.front))
    self.minleft=min(abs(self.left))
    self.minright=min(abs(self.right))
    self.minback=min(abs(self.back))
    self.avoid_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped , queue_size=1)
    self.avoid_radius = 5
    self.avoiding_obstacle = 0
    self.get_back_on_path = 0 

    self.building_center_x = 0
    self.building_center_y =0

    self.CRASH_WARNING = 0
    self.minsafety_all = np.zeros(360)

    self.previousSp_x = 0
    self.previousSp_y = 0

    self.target_x = 0
    self.target_y = 0

    self.avoid_x = 0
    self.avoid_y = 0

    self.stay_current_x = 0
    self.stay_current_y = 0


    self.stay_current_x_waypoint = 0
    self.stay_current_y_waypoint = 0

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

  # Land state callback
  def landStateCb(self, msg):
    if msg is not None:
      if msg.landed_state == 1:
        self.IS_LANDED = True
      else:
        self.IS_LANDED = False

  def rangessCb(self, msg):
    if msg is not None:
      self.front = msg.ranges[140:219]
      self.left = msg.ranges[255:284]
      self.right = msg.ranges[75:104]
      self.back=np.concatenate((list(msg.ranges[330:359]), list(msg.ranges[0:29])), axis=None)
      self.all360 = msg.ranges
      self.minsafety_all=min(msg.ranges)

  def transformationoffcuCb(self, msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),rospy.Time.now(),"fcu","local_origin")

  ############################################################################

  def resetStates(self):
    self.TAKEOFF = 0
    self.WAYPOINT1 = 0
    self.WAYPOINT2 = 0
    self.WAYPOINT3 = 0
    self.GOHOME = 0
    self.LAND = 0
    self.HOVER = 0

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

  
  def obstacle_avoid_decision(self):
    # Upon seeing an obstacle, this function will choose which direction is the best to go to avoid and stay in the same general direction.
    # It will take a 60 degree swath starting with azimuth at 180 degrees (which is forward) and ending at 179 degrees doing a full
    # check in all directions. Then it will compare to see which one is closest to 180 degrees that satisfies the condition {obstacle>4 meters from current position}
    # and will return that direction.

    ####################### IMPORTANT!!!!!!!!!!!!!! ###########################
    # object_far_enough_dist SHOULD BE SET LARGER (by about .5) THAN r!!!!!!!!!!!
    ###########################################################################

    rospy.logwarn('Entered obstacle_avoid_decision function')

    all360 = self.all360
    all360 = list(all360)
    object_far_enough_dist = 4 #3

    ####### Clean data a little bit before using ######
    i=0
    for x in all360:
      if x < .5:
        all360[i] = 1000
      i=i+1
    ###################################################
    # rospy.loginfo(all360)
    list_1080 = np.concatenate((all360,all360), axis=0) # This simplifies taking a 60 degree swath from the data so that we don't have to worry about trying to combine edges of data (x > 0 with x < 360)
    potential_path = []
    potential_path_index = [] # Not sure why I had to add this. For some reason, if I get rid of it and set a temp=potential_path, anything that happens to temp also happens to potential_path like they are connected somehow
    scan_for_opening = np.zeros(120)
    i=180
    azimuth = 180

    for x in range(360):
      scan_for_opening = list_1080[i-50:i+50]
      # rospy.logwarn(scan_for_opening)
      scan_for_opening = min(scan_for_opening)
      # rospy.logwarn(scan_for_opening)
      # rospy.sleep(1)
      if scan_for_opening > object_far_enough_dist:
        potential_path.append(azimuth)
        potential_path_index.append(azimuth)
      if azimuth == 359:
        azimuth = 0
      elif azimuth != 359:
        azimuth = azimuth + 1
      i = i + 1
    # rospy.loginfo("List of potential_path")
    # rospy.loginfo(potential_path)
    if potential_path_index == []:
      print "Error! All read values are lower than minimum object distance! Try again!"
      best_azimuth = 180
      print "Still heading forward"
    else:


      target_heading= math.degrees(math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x)))
      potential_path_index[:] = [abs(i - target_heading) for i in potential_path_index] # Subtract 180 from every value in potential_path
      best_azimuth_index = potential_path_index.index(min(potential_path_index))



        # potential_path_index[:] = [abs(i - 180) for i in potential_path_index] # Subtract 180 from every value in potential_path
        # best_azimuth_index = potential_path_index.index(min(potential_path_index))
      
      best_azimuth = potential_path[best_azimuth_index]
      rospy.logwarn("Best azimuth")
      rospy.logwarn(best_azimuth)

    best_azimuth = (best_azimuth-180)*pi/180.0 # Convert the azimuth from [0,360] to [-pi,pi]
    rospy.logwarn("Best azimuth")
    rospy.logwarn(best_azimuth)
    rospy.logwarn("EXITING obstacle_avoid_decision function")
    return best_azimuth

  def check_and_avoid(self):
    rospy.logwarn('Entered check_and_avoid function')

    min_dist_to_obstacle = 3 #2
    r = 3.5 #2.5 Distance to travel to avoid obstacle
    target_alt = 2.5
    front = self.front
    front=list(front)

    ####### Clean data a little bit before using ######
    i=0
    for x in front:
      if x < .5:
        front[i] = 1000
      i=i+1
    ###################################################

    self.minfront=min(front)
    
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
            self.avoid_pub.publish(self.positionSp)

        if (self.avoidancepar ==2):

            self.positionSp.pose.position.x = self.stay_current_x
            self.positionSp.pose.position.y = self.stay_current_y
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
          # rospy.sleep(2.0)
                    self.avoid_x=self.dx*math.cos(self.target_yaw)-self.dy*math.sin(self.target_yaw)+self.current_local_x
                    self.avoid_y=self.dx*math.sin(self.target_yaw)+self.dy*math.cos(self.target_yaw)+self.current_local_y

        if (self.avoidancepar ==3):
            if (abs(self.avoid_x) <= 8 and abs(self.avoid_y) <= 8):
                rospy.loginfo("Avoiding position is within bounds")
                self.positionSp.pose.position.x = self.avoid_x
                self.positionSp.pose.position.y = self.avoid_y
                self.positionSp.pose.position.z = target_alt
                self.positionSp.header.frame_id = 'local_origin'
                quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                self.positionSp.pose.orientation = Quaternion(*quaternion)
                self.avoidancepar = 4


            elif (abs(self.avoid_x) > 8 or abs(self.avoid_y) > 8):
                rospy.logwarn("Avoiding position is out of bounds")

                if self.avoid_x > 8 and abs(self.avoid_y) < 8:
                    self.avoid_x =7 
                    self.positionSp.pose.position.x = self.avoid_x
                    self.positionSp.pose.position.y = self.avoid_y
                    self.positionSp.pose.position.z = target_alt
                    self.positionSp.header.frame_id = 'local_origin'
                    quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                    self.positionSp.pose.orientation = Quaternion(*quaternion)
                    self.avoidancepar = 4



                if self.avoid_x < -8 and abs(self.avoid_y) < 8:
                    self.avoid_x =-7 
                    self.positionSp.pose.position.x = self.avoid_x
                    self.positionSp.pose.position.y = self.avoid_y
                    self.positionSp.pose.position.z = target_alt
                    self.positionSp.header.frame_id = 'local_origin'
                    quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                    self.positionSp.pose.orientation = Quaternion(*quaternion)
                    self.avoidancepar = 4

                if  abs(self.avoid_x) < 8 and self.avoid_y < -8:
                    self.avoid_y =-7 
                    self.positionSp.pose.position.x = self.avoid_x
                    self.positionSp.pose.position.y = self.avoid_y
                    self.positionSp.pose.position.z = target_alt
                    self.positionSp.header.frame_id = 'local_origin'
                    quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                    self.positionSp.pose.orientation = Quaternion(*quaternion)
                    self.avoidancepar = 4


                if abs(self.avoid_x) < 8 and self.avoid_y > 8:
                    self.avoid_y =7 
                    self.positionSp.pose.position.x = self.avoid_x
                    self.positionSp.pose.position.y = self.avoid_y
                    self.positionSp.pose.position.z = target_alt
                    self.positionSp.header.frame_id = 'local_origin'
                    quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                    self.positionSp.pose.orientation = Quaternion(*quaternion)
                    self.avoidancepar = 4



                elif abs(self.avoid_x) > 8 and abs(self.avoid_y) > 8:
                    rospy.logwarn("At the corner") 
                    self.avoiding_obstacle = 0
                    self.building_param = [0, 0, 0]
                    self.avoidancepar = 1
                    if self.path_flag == 'None':
                        self.path_flag = 'Right'
                        self.target_x=self.building_center_x+self.current_r
                        self.target_y=self.building_center_y-self.current_r

                        self.positionSp.pose.position.x=self.current_local_x
                        self.positionSp.pose.position.y=self.current_local_y

                        self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                        quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                        self.positionSp.pose.orientation = Quaternion(*quaternion)  
                    elif self.path_flag == 'Right':
                        self.path_flag = 'Far'
                        self.target_x=self.building_center_x+self.current_r
                        self.target_y=self.building_center_y+self.current_r

                        self.positionSp.pose.position.x=self.current_local_x
                        self.positionSp.pose.position.y=self.current_local_y

                        self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                        quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                        self.positionSp.pose.orientation = Quaternion(*quaternion)
                    elif self.path_flag == 'Far':
                        self.path_flag = 'Left'
                        self.target_x=self.building_center_x-self.current_r
                        self.target_y=self.building_center_y+self.current_r

                        self.positionSp.pose.position.x=self.current_local_x
                        self.positionSp.pose.position.y=self.current_local_y

                        self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                        quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                        self.positionSp.pose.orientation = Quaternion(*quaternion)
                    elif self.path_flag == 'Left':
                        self.path_flag = 'Near'
                        self.target_x=self.building_center_x-self.current_r
                        self.target_y=self.building_center_y-self.current_r

                        self.positionSp.pose.position.x=self.current_local_x
                        self.positionSp.pose.position.y=self.current_local_y

                        self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                        quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                        self.positionSp.pose.orientation = Quaternion(*quaternion)
                    elif self.path_flag == 'Near':
                        self.path_flag = 'Right'
                        self.current_r = self.current_r - 2
                        self.target_x=self.building_center_x+self.current_r
                        self.target_y=self.building_center_y-self.current_r

                        self.positionSp.pose.position.x=self.current_local_x
                        self.positionSp.pose.position.y=self.current_local_y

                        self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                        quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                        self.positionSp.pose.orientation = Quaternion(*quaternion)
        if (self.avoidancepar == 4 ):
            if (self.minfront < min_dist_to_obstacle):
                self.avoidancepar =1


            elif abs(self.current_local_x - self.avoid_x) < .7 and (abs(self.current_local_y - self.avoid_y) < .7):
                rospy.loginfo("Attempted to avoid obstacle. Now will check to see if on path")
                self.avoiding_obstacle = 0
                self.avoidancepar = 1
                self.building_param = [0, 0, 0]

        if (abs(self.current_local_x-self.target_x)<4) and (abs(self.current_local_y-self.target_y)<4) :
            self.avoiding_obstacle = 0
            self.building_param = [0, 0, 0]
            self.avoidancepar = 1
            if self.path_flag == 'None':
                self.path_flag = 'Right'
                self.target_x=self.building_center_x+self.current_r
                self.target_y=self.building_center_y-self.current_r

                self.positionSp.pose.position.x=self.current_local_x
                self.positionSp.pose.position.y=self.current_local_y

                self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                self.positionSp.pose.orientation = Quaternion(*quaternion)  
            elif self.path_flag == 'Right':
                self.path_flag = 'Far'
                self.target_x=self.building_center_x+self.current_r
                self.target_y=self.building_center_y+self.current_r

                self.positionSp.pose.position.x=self.current_local_x
                self.positionSp.pose.position.y=self.current_local_y

                self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                self.positionSp.pose.orientation = Quaternion(*quaternion)
            elif self.path_flag == 'Far':
                self.path_flag = 'Left'
                self.target_x=self.building_center_x-self.current_r
                self.target_y=self.building_center_y+self.current_r

                self.positionSp.pose.position.x=self.current_local_x
                self.positionSp.pose.position.y=self.current_local_y

                self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                self.positionSp.pose.orientation = Quaternion(*quaternion)
            elif self.path_flag == 'Left':
                self.path_flag = 'Near'
                self.target_x=self.building_center_x-self.current_r
                self.target_y=self.building_center_y-self.current_r

                self.positionSp.pose.position.x=self.current_local_x
                self.positionSp.pose.position.y=self.current_local_y

                self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                self.positionSp.pose.orientation = Quaternion(*quaternion)
            elif self.path_flag == 'Near':
                self.path_flag = 'Right'
                self.current_r = self.current_r - 2
                self.target_x=self.building_center_x+self.current_r
                self.target_y=self.building_center_y-self.current_r

                self.positionSp.pose.position.x=self.current_local_x
                self.positionSp.pose.position.y=self.current_local_y

                self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
                quaternion = quaternion_from_euler(0, 0, self.target_yaw)
                self.positionSp.pose.orientation = Quaternion(*quaternion)
      # rospy.sleep(2)




    rospy.logwarn("EXITING check_and_avoid function")
    return self.avoiding_obstacle

  
  def building_search_path (self):
    # This function sets up the path for the drone to take inside the building when it is trying to find the missing worker as well as for mapping.
    # After completing one full loop around the building, it decreases search radius by 2 (radius is actually width of square)
    mapping_altitude = 2.5
    currently_avoiding_obstacle = 0
    obstacle_to_avoid_immediately = 0
    go_back_to_path = 0

    # obstacle_to_avoid_immediately = self.safetyradius()
    if obstacle_to_avoid_immediately != 1:            # If there is not an obstacle to avoid immediately, check to see if currently attempting to avoid an obstacle
      currently_avoiding_obstacle = self.check_and_avoid()
      rospy.logwarn("Output of check_and_avoid function")
      rospy.logwarn(currently_avoiding_obstacle)

      if currently_avoiding_obstacle != 1 and self.minfront >2.5:
        # go_back_to_path = self.get_back_to_path(self.path_flag)
        rospy.logwarn("Output of get_back_to_path function")
        rospy.logwarn(go_back_to_path)

        if (go_back_to_path != 1):
    
          if (self.path_flag == 'None'):

            if self.building_param[0] == 0:
              rospy.logwarn("Building Parameter 1. Setting yaw")
              self.target_x=self.building_center_x-self.current_r
              self.target_y=self.building_center_y-self.current_r
              self.stay_current_x_waypoint=self.current_local_x
              self.stay_current_y_waypoint=self.current_local_y

              self.positionSp.pose.position.x=self.current_local_x
              self.positionSp.pose.position.y=self.current_local_y

              self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
              quaternion = quaternion_from_euler(0, 0, self.target_yaw)
              self.positionSp.pose.orientation = Quaternion(*quaternion)  
              self.building_param[0] = 1

            elif self.building_param[1] == 0:
              rospy.logwarn("Building Parameter 2. Waiting on yaw")
              self.positionSp.pose.position.x=self.stay_current_x_waypoint
              self.positionSp.pose.position.y=self.stay_current_y_waypoint
              if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
                self.positionSp.pose.position.x=self.target_x
                self.positionSp.pose.position.y=self.target_y
                self.building_param[1] = 1

            elif self.building_param[2] == 0:
              rospy.logwarn("Building Parameter 3. Waiting on location")
              if (abs(self.current_local_x-self.target_x)<3) and (abs(self.current_local_y-self.target_y)<3):
                self.path_flag = 'Right'
                rospy.logwarn("I reached close/right waypoint")
                self.building_param = [0, 0, 0]

          elif (self.path_flag == 'Right'):
            if self.building_param[0] == 0:
              self.target_x=self.building_center_x+self.current_r
              self.target_y=self.building_center_y-self.current_r
              self.stay_current_x_waypoint=self.current_local_x
              self.stay_current_y_waypoint=self.current_local_y

              self.positionSp.pose.position.x=self.current_local_x
              self.positionSp.pose.position.y=self.current_local_y

              self.positionSp.pose.position.x=self.current_local_x
              self.positionSp.pose.position.y=self.current_local_y

              self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
              quaternion = quaternion_from_euler(0, 0, self.target_yaw)
              self.positionSp.pose.orientation = Quaternion(*quaternion)  
              self.building_param[0] = 1

            elif self.building_param[1] == 0:
              self.positionSp.pose.position.x=self.stay_current_x_waypoint
              self.positionSp.pose.position.y=self.stay_current_y_waypoint
              if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
                self.positionSp.pose.position.x=self.target_x
                self.positionSp.pose.position.y=self.target_y
                self.building_param[1] = 1

            elif self.building_param[2] == 0:
              if (abs(self.current_local_x-self.target_x)<2.5) and (abs(self.current_local_y-self.target_y)<2.5):
                self.path_flag = 'Far'
                rospy.logwarn("I reached far/right waypoint")
                self.building_param = [0, 0, 0]

          elif (self.path_flag == 'Far'):
            if self.building_param[0] == 0:
              self.target_x=self.building_center_x+self.current_r
              self.target_y=self.building_center_y+self.current_r
              self.stay_current_x_waypoint=self.current_local_x
              self.stay_current_y_waypoint=self.current_local_y

              self.positionSp.pose.position.x=self.current_local_x
              self.positionSp.pose.position.y=self.current_local_y

              self.positionSp.pose.position.x=self.current_local_x
              self.positionSp.pose.position.y=self.current_local_y

              self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
              quaternion = quaternion_from_euler(0, 0, self.target_yaw)
              self.positionSp.pose.orientation = Quaternion(*quaternion)  
              self.building_param[0] = 1

            elif self.building_param[1] == 0:
              self.positionSp.pose.position.x=self.stay_current_x_waypoint
              self.positionSp.pose.position.y=self.stay_current_y_waypoint
              if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
                self.positionSp.pose.position.x=self.target_x
                self.positionSp.pose.position.y=self.target_y
                self.building_param[1] = 1

            elif self.building_param[2] == 0:
              if (abs(self.current_local_x-self.target_x)<2.5) and (abs(self.current_local_y-self.target_y)<2.5):
                self.path_flag = 'Left'
                rospy.logwarn("I reached far/left waypoint")
                self.building_param = [0, 0, 0]

          elif (self.path_flag == 'Left'):
            if self.building_param[0] == 0:
              self.target_x=self.building_center_x-self.current_r
              self.target_y=self.building_center_y+self.current_r
              self.stay_current_x_waypoint=self.current_local_x
              self.stay_current_y_waypoint=self.current_local_y

              self.positionSp.pose.position.x=self.current_local_x
              self.positionSp.pose.position.y=self.current_local_y

              self.positionSp.pose.position.x=self.current_local_x
              self.positionSp.pose.position.y=self.current_local_y

              self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
              quaternion = quaternion_from_euler(0, 0, self.target_yaw)
              self.positionSp.pose.orientation = Quaternion(*quaternion)  
              self.building_param[0] = 1

            elif self.building_param[1] == 0:
              self.positionSp.pose.position.x=self.stay_current_x_waypoint
              self.positionSp.pose.position.y=self.stay_current_y_waypoint
              if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
                self.positionSp.pose.position.x=self.target_x
                self.positionSp.pose.position.y=self.target_y
                self.building_param[1] = 1

            elif self.building_param[2] == 0:
              if (abs(self.current_local_x-self.target_x)<2.5) and (abs(self.current_local_y-self.target_y)<2.5):
                self.path_flag = 'Near'
                rospy.logwarn("I reached close/left waypoint")
                self.building_param = [0, 0, 0]

          elif (self.path_flag == 'Near'):
            if self.building_param[0] == 0:
              self.target_x=self.building_center_x-self.current_r
              self.target_y=self.building_center_y-self.current_r
              self.stay_current_x_waypoint=self.current_local_x
              self.stay_current_y_waypoint=self.current_local_y

              self.positionSp.pose.position.x=self.current_local_x
              self.positionSp.pose.position.y=self.current_local_y

              self.positionSp.pose.position.x=self.current_local_x
              self.positionSp.pose.position.y=self.current_local_y

              self.target_yaw = math.atan2((self.target_y-self.current_local_y),(self.target_x-self.current_local_x))
              quaternion = quaternion_from_euler(0, 0, self.target_yaw)
              self.positionSp.pose.orientation = Quaternion(*quaternion)  
              self.building_param[0] = 1

            elif self.building_param[1] == 0:
              self.positionSp.pose.position.x=self.stay_current_x_waypoint
              self.positionSp.pose.position.y=self.stay_current_y_waypoint
              if abs(self.current_yaw-self.target_yaw) < .2 or (abs(self.current_yaw - (self.target_yaw-2*pi)) < .3):
                self.positionSp.pose.position.x=self.target_x
                self.positionSp.pose.position.y=self.target_y
                self.building_param[1] = 1

            elif self.building_param[2] == 0:
              if (abs(self.current_local_x-self.target_x)<2.5) and (abs(self.current_local_y-self.target_y)<2.5):
                self.path_flag = 'Right'
                rospy.logwarn("I reached close/right waypoint again. Resetting with a closer radius")
                self.building_param = [0, 0, 0]
                self.current_r = self.current_r - 2

def main():
  rospy.init_node('obstacle_avoidance', anonymous=True)
  rospy.logwarn("GPS setpoints node is started")

  K = Controller()

  ########## Subscribers ##########

  # Subscriber: GPS
  rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, K.gpsCb)

  # Subscriber: Local pose
  rospy.Subscriber("mavros/local_position/pose", PoseStamped, K.localPoseCb)

  rospy.Subscriber("mavros/local_position/pose", PoseStamped, K.transformationoffcuCb)

  # Get landing state
  rospy.Subscriber("mavros/extended_state", ExtendedState, K.landStateCb)

  # FOR RPLIDAR
  rospy.Subscriber("laser/scan", LaserScan, K.rangessCb)

  ########## Publishers ##########
  
  # Publisher: PositionTarget
  # avoid_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
  rate = rospy.Rate(10.0)

  # Do initial checks
  while ((K.current_lat*K.current_lon*K.current_alt) == 0) and not rospy.is_shutdown():

    rospy.loginfo('Waiting for current gps location to execute setWaypoints_and_Fence') # Initializes waypoints and fence in local x,y,z and checks to see if they make sense
    rospy.sleep(0.1)

  K.setWayoints_and_Fence()

  K.resetStates()

  # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
  k=0
  while k<10:
    K.avoid_pub.publish(K.positionSp)
    rate.sleep()
    k = k+1
  K.positionSp.pose.position.x=K.current_local_x
  K.positionSp.pose.position.y=K.current_local_y
  K.positionSp.pose.position.z=K.takeoff_height

  K.modes.setOffboardMode()
  K.modes.setArm()

  ###################### TAKEOFF STUFF ####################
  K.TAKEOFF = 1

  #########################################################

  while not rospy.is_shutdown():

    if K.TAKEOFF:
      rospy.logwarn('Vehicle is taking off')
      rospy.loginfo('Takeoff Height')
      rospy.loginfo(K.positionSp.pose.position.z)
             
      if abs(K.current_local_z - K.takeoff_height) < .1:
        rospy.logwarn("Reached Takeoff Height")
        K.resetStates()
        K.TENT = 1

    if K.TENT:

      K.building_search_path()
      rospy.loginfo('Current side')
      rospy.loginfo(K.path_flag)

      print('Waypoint_x')
      print(K.positionSp.pose.position.x)
      print('Waypoint_y')
      print(K.positionSp.pose.position.y)

      print('Current_x')
      print(K.current_local_x)
      print('Current_y')
      print(K.current_local_y)

      print('Target_yaw')
      print(K.target_yaw)
      print('Current_yaw')
      print(K.current_yaw)

      print('Current radius')
      print(K.current_r)

      print('Building Parameter')
      print(K.building_param)

      print('AVOIDANCE PARAMETER')
      print(K.avoidancepar)


      if K.current_r < 2:
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
    K.avoid_pub.publish(K.positionSp)
  
    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

