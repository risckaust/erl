#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion


from sensor_msgs.msg import NavSatFix,Joy
from geometry_msgs.msg import Point, PointStamped

from dji_sdk.srv import SDKControlAuthority,DroneTaskControl

class Controller:

	def __init__(self):

		# Current GPS Coordinates
		self.current_lat					= 0.0
		self.current_lon					= 0.0
		self.current_alt					= 0.0

		# Current local ENU coordinates
		self.current_local_x				= 0.0
		self.current_local_y				= 0.0
		self.current_local_z				= 0.0

                self.dji_on_land                                = True
		# Instantiate a setpoint topic structure
		self.dji_position_sp					=Joy()
		self.dji_position_sp.axes                               =[0, 0, 0, 0]
                
                self.dji_velocity_sp					=Joy()
		self.dji_velocity_sp.axes                               =[0, 0, 0, 0]
                 
                self.last_time                                 = 0
                self.current_time                              = 0

                self.step_size_x                               = 0
                self.step_size_y                               = 0


		######### Callbacks #########
        def enableSDK(self):
                rospy.wait_for_service('dji_sdk/sdk_control_authority')        
                try:
                   s = rospy.ServiceProxy('dji_sdk/sdk_control_authority',SDKControlAuthority)
                  
                   response=s(control_enable=1)

                   if response.result:
                      print "The DJI is enabled"

                   else:
                      print "Obtain Control Failed"


                except rospy.ServiceException, e:
                    print "Service call failed : %s"%e
     
        def takeOff(self):
                rospy.wait_for_service('dji_sdk/drone_task_control')        
                try:
                   s = rospy.ServiceProxy('dji_sdk/drone_task_control',DroneTaskControl)
                   
                   response=s( task = 4)
                 

                   if response.result:
                      print "The DJI is Taking Off"
                      self.dji_on_land = False

                   else:
                      print "TakeOff Failed"


                except rospy.ServiceException, e:
                    print "Service call failed : %s"%e  
    
        def land(self):
                rospy.wait_for_service('dji_sdk/drone_task_control')        
                try:
                   s = rospy.ServiceProxy('dji_sdk/drone_task_control',DroneTaskControl)
                   
                   response=s( task = 6)
                 
                   self.dji_on_land = True

                   if response.result:
                      print "The DJI is landing"
                      
                   else:
                      print "Landing Failed"


                except rospy.ServiceException, e:
                    print "Service call failed : %s"%e      

	def gpsCb(self, msg):
		if msg is not None:
			self.current_lat = msg.latitude
			self.current_lon = msg.longitude
			self.current_alt = msg.altitude

	def localPoseCb(self, msg):
		if msg is not None:
			self.current_local_x = msg.point.x
			self.current_local_y = msg.point.y
			self.current_local_z = msg.point.z

	def gpsSpCb(self, msg):
		# lat = msg.x; lon = msg.y;
		# msg.z is the altitude setpoint in ENU
		if msg is not None:
			target_lat = msg.x
			target_lon = msg.y
			target_alt = msg.z # in ENU

			target_gps_alt = self.current_alt
			delta_x, delta_y, delta_z = pm.geodetic2enu(target_lat, target_lon, target_gps_alt, self.current_lat, self.current_lon, self.current_alt)
                        
                        print delta_x
                        print delta_y
                        
                        if abs(delta_x) > 0.1 and abs(delta_y) > 0.1 and self.dji_on_land:
                               self.takeOff()
                               rate=rospy.Rate(10)

			       i = 0
			       while not rospy.is_shutdown() and i  < 10:
					# Initialize setpoint
					self.dji_position_sp.axes[0] = 0
					self.dji_position_sp.axes[1] = 0
					self.dji_position_sp.axes[2] = 1

					i = i +1
					rate.sleep()
                               self.last_time = rospy.get_time()

                        if abs(delta_x) < 0.5 and abs(delta_y) < 0.5:
                               self.land()
                        
                        self.current_time=rospy.get_time()
                        dt = self.last_time-self.current_time
                                               
                        if delta_x > 1:
                              self.dji_velocity_sp.axes[0] = 2
                              self.step_size_x=self.dji_velocity_sp.axes[0]*dt
                        elif delta_x < -1:
                              self.dji_velocity_sp.axes[0] = -2
                              self.step_size_x=self.dji_velocity_sp.axes[0]*dt
                        else:
                              self.dji_velocity_sp.axes[0]=0
                              self.step_size_x=delta_x

                        if delta_y > 1:
                              self.dji_velocity_sp.axes[1] = 2
                              self.step_size_y=self.dji_velocity_sp.axes[1]*dt
                        elif delta_y < -1:
                              self.dji_velocity_sp.axes[1] = -2
                              self.step_size_y=self.dji_velocity_sp.axes[1]*dt
                        else:
                              self.dji_velocity_sp.axes[1] = 0
                              self.step_size_y=delta_y
                               
			#self.dji_position_sp.axes[0] = self.step_size_x
			#self.dji_position_sp.axes[1] = self.step_size_y
			#self.dji_position_sp.axes[2] =  target_alt

                        self.last_time = self.current_time

                else:
                        self.dji_position_sp.axes[0] = 0
                        self.dji_position_sp.axes[1] = 0
                        self.dji_position_so.axes[2] = 0


def main():
	rospy.init_node('gps_setpoint_node', anonymous=True)

	rospy.logwarn("GPS setpoints node is started")

	K = Controller()

	########## Subscribers ##########

	# Subscriber: GPS
	rospy.Subscriber("dji_sdk/gps_position", NavSatFix, K.gpsCb)

	# Subscriber: Local pose
	rospy.Subscriber("dji_sdk/local_position", PointStamped, K.localPoseCb)

	# User GPS setpoint
	rospy.Subscriber("dji_gps_setpoint", Point, K.gpsSpCb)

       
        

	########## Publishers ##########

	# Publisher: PositionTarget
	setp_pub = rospy.Publisher("dji_sdk/flight_control_setpoint_ENUposition_yaw", Joy, queue_size=10)        
        setv_pub = rospy.Publisher("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", Joy, queue_size=10) 

	rate = rospy.Rate(10.0)
        
        K.enableSDK()
        
        #calling TakeOff function
        K.takeOff()

	i = 0
	while not rospy.is_shutdown() and i  < 10:
		# Initialize setpoint
		K.dji_position_sp.axes[0] = 0
		K.dji_position_sp.axes[1] = 0
		K.dji_position_sp.axes[2] = 1

		i = i +1
		rate.sleep()

        K.last_time = rospy.get_time()

	while not rospy.is_shutdown():

                setv_pub.publish(K.dji_velocity_sp)
		#setp_pub.publish(K.dji_position_sp)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
