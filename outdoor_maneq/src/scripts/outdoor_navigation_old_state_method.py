#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

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


class Controller:

	def __init__(self):

		# Current GPS Coordinates
		self.current_lat = 0.0
		self.current_lon = 0.0
		self.current_alt = 0.0

		# GPS Fence !!!!!!!!!!!!!!!!!!!!!!!!!! This will need to be changed so that it is a polygon, not a square
		self.lat_max = 22.309131#22.3176 # Location of Testing Field at KAUST
		self.lat_min = 22.307000#22.31725# Location of Testing Field at KAUST
		self.lon_max = 39.105090#39.0983 # Location of Testing Field at KAUST
		self.lon_min = 39.104000#39.0977 # Location of Testing Field at KAUST
		self.z_limit = 30
		
		#Waypoints GPS Coordiantes
		self.waypoint1_lat = 22.307048#22.317490
		self.waypoint1_lon = 39.104780#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint1_alt = 4

		self.waypoint2_lat = 22.307048#22.317490
		self.waypoint2_lon = 39.104780#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint2_alt = #4

		self.waypoint3_lat = 22.307048#22.317490
		self.waypoint3_lon = 39.104780#39.097909 # Location of Testing Field at KAUST - a little ways down field
		self.waypoint3_alt = 4

		# Current local ENU coordinates
		self.current_local_x = 0.0
		self.current_local_y = 0.0
		self.current_local_z = 0.0

		#Waypoints ENU Coordinates
		self.waypoint1_x = 0
		self.waypoint1_y = 0
		self.waypoint1_z = 0

		self.waypoint2_x = 0
		self.waypoint2_y = 0
		self.waypoint2_z = 0 

		self.waypoint3_x = 0
		self.waypoint3_y = 0
		self.waypoint3_z = 0 
		
		self.x_FenceLimit_max = 0
		self.y_FenceLimit_max = 0
		self.x_FenceLimit_min = 0
		self.y_FenceLimit_min = 0

		self.x_fence_max_warn = 0
		self.x_fence_min_warn = 0
		self.y_fence_max_warn = 0
		self.y_fence_min_warn = 0
		self.z_limit_warn = 0

		self.delta_x = 0
		self.delta_y = 0
		self.delta_z = 0

		self.current_state  = 'Idle'
		self.current_signal = ''
		self.resume_state = 'Hover'

		self.takeoff_height = 1

		# Instantiate a setpoint topic structure
		self.positionSp	= PoseStamped()
		#defining the modes
		self.modes = fcuModes()

		#self.avoid_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
		#self.rate = rospy.Rate(10.0)


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

	def setWayoints_and_Fence(self):

		rospy.logwarn('Current lat')
		rospy.logwarn(self.current_lat)

		rospy.logwarn('Current lon')
		rospy.logwarn(self.current_lat)

		rospy.logwarn('Current atl')
		rospy.logwarn(self.current_lat)

		self.x_FenceLimit_max, self.y_FenceLimit_max, self.z_limit = self.calculate_displacements(self.lat_max,self.lon_max,self.z_limit)
		self.x_FenceLimit_min, self.y_FenceLimit_min, self.z_limit = self.calculate_displacements(self.lat_min,self.lon_min,self.z_limit)

		self.waypoint1_x, self.waypoint1_y, self.waypoint1_z = self.calculate_displacements(self.waypoint1_lat,self.waypoint1_lon,self.waypoint1_alt)
		self.waypoint2_x, self.waypoint2_y, self.waypoint2_z = self.calculate_displacements(self.waypoint2_lat,self.waypoint2_lon,self.waypoint2_alt)
		self.waypoint3_x, self.waypoint3_y, self.waypoint3_z = self.calculate_displacements(self.waypoint3_lat,self.waypoint3_lon,self.waypoint3_alt)

		self.x_fence_max_warn = self.x_FenceLimit_max - 1
		self.x_fence_min_warn = self.x_FenceLimit_min + 1
		self.y_fence_max_warn = self.y_FenceLimit_max - 1
		self.y_fence_min_warn = self.y_FenceLimit_min + 1
		self.z_limit_warn = self.z_limit - .5

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

		self.isValidWaypoint(self.waypoint1_x,self.waypoint1_y,self.waypoint1_z) # Test whether waypoint 1 is within fence
		self.isValidWaypoint(self.waypoint2_x,self.waypoint2_y,self.waypoint2_z) # Test whether waypoint 2 is within fence
		self.isValidWaypoint(self.waypoint3_x,self.waypoint3_y,self.waypoint3_z) # Test whether waypoint 3 is within fence
    	
	def calculate_displacements(self,target_lat,target_lon,target_alt):
		#self.delta_x= target_lat#-self.current_local_x
		#self.delta_y= target_lon#-self.current_local_y
		#self.delta_z= target_alt

		delta_x,delta_y,delta_z = pm.geodetic2enu(target_lat, target_lon, target_alt, self.current_lat, self.current_lon, self.current_alt)

		return delta_x,delta_y,delta_z

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

	# State: start
	def execute_start(self):
		
		self.current_state = 'Start'
		self.current_signal = 'Running'

		# If target waypoints are out of bounds, exit code and put valid waypoints in "_init_(self)

		self.positionSp.header.frame_id = 'local_origin'
		self.positionSp.pose.position.x = self.current_local_x 
		self.positionSp.pose.position.y = self.current_local_y
		self.positionSp.pose.position.z = self.takeoff_height
		self.positionSp.pose.orientation.w = 1.0
		self.avoid_pub.publish(self.positionSp) # Hopefully publishing this before setting to OFFBOARD mode will take care of the weird takeoff altitudes

		rospy.logwarn('x target')
		rospy.logwarn(self.positionSp.pose.position.x)
		rospy.logwarn('y target')
		rospy.logwarn(self.positionSp.pose.position.y)
		rospy.logwarn('z target')
		rospy.logwarn(self.positionSp.pose.position.z)

		# Activated upon start command
		self.loginfo = 'The quadcopter is set to OFFBOARD...'
		self.modes.setOffboardMode()
		self.current_signal = 'Done'
		

	def takeoff(self):
		self.current_state  = 'TakeOff'
		self.current_signal = 'Running'

		self.modes.setArm()
		# VALUES THAT HAVE ALREADY BEEN SET
 	# 	self.positionSp.header.frame_id='local_origin'
		# self.positionSp.pose.position.x =  self.delta_x 
		# self.positionSp.pose.position.y =  self.delta_y 
		# self.positionSp.pose.position.z =  takeoff_height
		# self.positionSp.pose.orientation.w = 1.0
		while not rospy.is_shutdown() and (self.current_local_z<self.takeoff_height):
			if (self.isTooCloseToFence()== True):
				rospy.logwarn('Changing to Hover mode because vehicle was too close to boundary')
				self.resume_state = 'TakeOff'
				self.current_signal = 'Interrupted'
		
		rospy.logwarn("Reached Takeoff Height")		
		self.current_signal = 'Done'

	def execute_wayPoint1(self):
	    # Combined with obstacle avoidance package. HOW CAN WE COMBINE IT WITH AVOIDANCE PACKAGE????

	    rospy.logwarn("Heading to Waypoint1...")
	    self.current_state = 'Waypoint1'
	    self.current_signal = 'Running'

	    self.positionSp.header.frame_id='local_origin'
	    self.delta_x, self.delta_y,self.delta_z = self.calculate_displacements(self.waypoint1_x,self.waypoint1_y,self.waypoint1_z)  
	    self.positionSp.pose.position.x =  self.delta_x #+self.current_local_x #+   # change it when u meet abdulqade
	    self.positionSp.pose.position.y =  self.delta_y #+self.current_local_y #+ self.delta_y
	    self.positionSp.pose.position.z =  self.delta_z
	    self.positionSp.pose.orientation.w = 1.0

	    if (self.isTooCloseToFence()== True):
	    	rospy.logwarn('Changing to Hover mode because vehicle was too close to boundary')
	    	self.resume_state = 'Waypoint1'
	    	self.current_signal = 'Interrupted'

	    elif abs(self.delta_x -self.current_local_x)<= 0.1 and abs(self.delta_y-self.current_local_y) <= 0.1 :#and abs(self.delta_z-self.current_local_z) <= 3.0:   # Rules give a 3m radius from goal
	    	rospy.logwarn("Current position close enough to desired waypoint")
	    	self.current_signal = 'Done'
	    	rospy.logwarn("Reached waypoint 1")

	def execute_wayPoint2(self):
		# Combined with obstacle avoidance package. HOW CAN WE COMBINE IT WITH AVOIDANCE PACKAGE????

	    rospy.logwarn("Heading to Waypoint2...")
	    self.current_state = 'Waypoint2'
	    self.current_signal = 'Running'

	    self.positionSp.header.frame_id='local_origin'
	    self.delta_x, self.delta_y,self.delta_z = self.calculate_displacements(self.waypoint2_x,self.waypoint2_y,self.waypoint2_z)  
	    self.positionSp.pose.position.x =  self.delta_x #+self.current_local_x #+   # change it when u meet abdulqade
	    self.positionSp.pose.position.y =  self.delta_y #+self.current_local_y #+ self.delta_y
	    self.positionSp.pose.position.z =  self.delta_z
	    self.positionSp.pose.orientation.w = 1.0

	    if (self.isTooCloseToFence()== True):
	    	rospy.logwarn('Changing to Hover mode because vehicle was too close to boundary')
	    	self.resume_state = 'Waypoint2'
	    	self.current_signal = 'Interrupted'

	    elif abs(self.delta_x -self.current_local_x)<= 0.1 and abs(self.delta_y-self.current_local_y) <= 0.1 :#and abs(self.delta_z-self.current_local_z) <= 3.0:   # Rules give a 3m radius from goal
	    	rospy.logwarn("Current position close enough to desired waypoint")
	    	self.current_signal = 'Done'
	    	rospy.logwarn("Reached waypoint 2")

	def execute_wayPoint3(self):
		# Combined with obstacle avoidance package. HOW CAN WE COMBINE IT WITH AVOIDANCE PACKAGE????

	    rospy.logwarn("Heading to Waypoint3...")
	    self.current_state = 'Waypoint3'
	    self.current_signal = 'Running'

	    self.positionSp.header.frame_id='local_origin'
	    self.delta_x, self.delta_y,self.delta_z = self.calculate_displacements(self.waypoint3_x,self.waypoint3_y,self.waypoint3_z)  
	    self.positionSp.pose.position.x =  self.delta_x #+self.current_local_x #+   # change it when u meet abdulqade
	    self.positionSp.pose.position.y =  self.delta_y #+self.current_local_y #+ self.delta_y
	    self.positionSp.pose.position.z =  self.delta_z
	    self.positionSp.pose.orientation.w = 1.0

	    if (self.isTooCloseToFence()== True):
	    	rospy.logwarn('Changing to Hover mode because vehicle was too close to boundary')
	    	self.resume_state = 'Waypoint3'
	    	self.current_signal = 'Interrupted'

	    elif abs(self.delta_x -self.current_local_x)<= 0.1 and abs(self.delta_y-self.current_local_y) <= 0.1 :#and abs(self.delta_z-self.current_local_z) <= 3.0:   # Rules give a 3m radius from goal
	    	rospy.logwarn("Current position close enough to desired waypoint")
	    	self.current_signal = 'Done'
	    	rospy.logwarn("Reached waypoint 3")

	def land(self):
		self.current_state = 'Land'
		self.current_signal = 'Running'
		self.modes.setAutoLandMode()

	def hover(self):
		self.current_state = 'Hover'
		self.current_signal = 'Running'
		self.delta_x = self.current_local_x
		self.delta_y = self.current_local_y
		self.delta_z = self.current_local_z #This may cause some drift - consider changing

		self.positionSp.header.frame_id='local_origin'
		self.positionSp.pose.position.x =  self.delta_x # self.current_local_x +   # change it when u meet abdulqade
		self.positionSp.pose.position.y =  self.delta_y #self.current_local_x + self.delta_y
		self.positionSp.pose.position.z =  self.delta_z

		self.avoid_pub.publish(self.positionSp)

		#self.current_signal = 'Done'

	def update_state(self):

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

		# manage the transition between states
		state = self.current_state
		signal = self.current_signal
		
		if (state == 'Start' and signal != 'Done'):	# initial signal
			self.execute_start()
			rospy.logwarn("drone start")
		elif (state == 'Start' and signal == 'Done'):
			rospy.logwarn("Vehicle About to Takeoff")
			self.takeoff()
		elif (state == 'TakeOff' and signal == 'Running'):
			rospy.logwarn("Vehicle Currently Taking Off")
			self.takeoff()
		elif (state == 'Takeoff' and signal == 'Interrupted'):
			rospy.logwarn("Takeoff was interrupted! Entering Hover mode")
			self.hover()
		elif (state == 'TakeOff' and signal == 'Done'):
			rospy.logwarn("Takeoff Complete")
			self.execute_wayPoint1()
		elif (state == 'Waypoint1' and signal == 'Running'):
			rospy.logwarn('Heading to Waypoint1')
			self.execute_wayPoint1() 
		elif (state == 'Waypoint1' and signal == 'Interrupted'):
			rospy.logwarn('Waypoint1 Interrupted!')
			self.resume_state = 'Waypoint1'
			self.hover()
		elif (state == 'Waypoint1' and signal == 'Done'):
			rospy.logwarn("Waypoint1 complete.")

			self.hover()
			#rospy.logwarn("Moving to Waypoint2")
			#self.execute_wayPoint2()
			
		elif (state == 'Waypoint2' and signal == 'Running'):
			rospy.logwarn("Heading to Waypoint2")
			self.execute_wayPoint2()
		elif (state == 'Waypoint2' and signal == 'Interrupted'):
			rospy.logwarn('Waypoint2 Interrupted!')
			self.resume_state = 'Waypoint2'
			self.hover()
		elif (state == 'Waypoint2' and signal == 'Done'):
			rospy.logwarn("Waypoint2 complete")

			self.land()
			#rospy.logwarn("Moving to Waypoint3")
			#self.execute_wayPoint3()

		elif (state == 'Waypoint3' and signal == 'Running'):
			rospy.logwarn("Headed to Waypoint3")
			self.execute_wayPoint3()
		elif (state == 'Waypoint3' and signal == 'Interrupted'):
			rospy.logwarn('Waypoint3 Interrupted!')
			self.resume_state = 'Waypoint3'
			self.hover()
		elif (state == 'Waypoint3' and signal == 'Done'):
			rospy.logwarn("Waypoint3 complete.")
			
			self.land()
			#rospy.logwarn("Moving to Waypoint3")
			#self.execute_wayPoint3()

		elif(state == 'Land' and signal == 'Running'):
			rospy.logwarn("Mission Complete. Landing")
			self.land()

		elif (state == 'Hover' and signal == 'Done'):
			rospy.logwarn("Mission continuing where left off")
			self.current_state = self.resume_state
			self.current_signal = 'Running'

def main():
	rospy.init_node('gps_setpoint_node', anonymous=True)
	rospy.logwarn("GPS setpoints node is started")

	K = Controller()

	########## Subscribers ##########

	# Subscriber: GPS
	rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, K.gpsCb)

	# Subscriber: Local pose
	rospy.Subscriber("mavros/local_position/pose", PoseStamped, K.localPoseCb)

	########## Publishers ##########

	# Publisher: PositionTarget
	avoid_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
	rate = rospy.Rate(10.0)

	# Do initial checks
	K.setWayoints_and_Fence() # Initializes waypoints and fence and checks to see if they make sense

	K.current_state = 'Start'
	K.current_signal = 'Running'

	while not rospy.is_shutdown():
		K.update_state()
		K.avoid_pub.publish(K.positionSp)	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass