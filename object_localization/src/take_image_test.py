#!/usr/bin/env python

import rospy # ROS interface
import pymap3d as pm # coordinate conversion
import tf

from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PoseWithCovarianceStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

import cv2

# THIS CODE SUCCESSFULLY CAPTURES A PHOTO AND LOGS SOME INFORMATION
# LAUNCH FILES REQUIRED
# roslaunch zed_wrapper zed_tarek.launch
# roslaunch darknet_ros yolov3_tiny_tarek.launch
# rosrun object_localization take_image_test.py

class Controller:

	def __init__(self):

		# For camera, taking pictures
		self.bridge = CvBridge()
		self.worker1_found_flag = 1
		self.WORKER1SEARCH = 1
		self.worker2_found_flag = 0
		self.WORKER2SEARCH = 0
		self.timer_sec = 0
		self.start_time = 0
		self.worker_lat = 22.221313
		self.worker_lon = 39.091118
		self.positionSp = PoseStamped()


	def WorkerImageCb(self, msg):
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
				hs = open("ObjectOfInterest.txt","a")
				s = "\nTime Stamp (sec)\n"
				s = s + str(self.timer_sec)
				if self.WORKER1SEARCH:
					cv2.imwrite('Worker1.jpeg', cv2_img)
					s = s + "\nTarget ID:\nOutside Worker\nTarget Position (Lat, Lon)\n"
					s = s + "(" + str(self.worker_lat) + ", " + str(self.worker_lon) + ")\n"
				elif self.WORKER2SEARCH:
					cv2.imwrite('Worker2.jpeg', cv2_img)
					s = s + "\nTarget ID:\nInside Worker\nTarget Position (Lat, Lon)\n"
					s = s + "(" + str(self.worker_lat) + ", " + str(self.worker_lon) + ")\n"
				hs.write(s)
				hs.close()
				rospy.sleep(10)


def main():
    rospy.init_node('gps_setpoint_node', anonymous=True)
    rospy.logwarn("GPS setpoints node is started")
    K = Controller()
    K.start_time = rospy.get_time()
    rospy.sleep(1)
	# Subscriber: Images
    rospy.Subscriber("/zed/left/image_rect_color", Image, K.WorkerImageCb)

	# Publisher: PositionTarget
    avoid_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    rate = rospy.Rate(10.0)

	# We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        avoid_pub.publish(K.positionSp)
        rate.sleep()
        k = k+1

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
