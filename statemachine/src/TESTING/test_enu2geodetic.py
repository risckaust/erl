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

local_x = 5
local_y = 2
local_z = 0

target_lat = 0
target_lon = 0
target_alt = 0

home_lat = 22.3118631
home_lon = 39.0952322
home_alt = 0

(target_lat, target_lon, target_alt) = pm.enu2geodetic(local_x,local_y,local_z,home_lat,home_lon,home_alt)

(x, y, z) = pm.geodetic2enu(22.4118631,39.1952322,0,home_lat,home_lon,home_alt)

# IF target_lon > current_lon, this translates to a positive x value
# IF target_lat > current_lat, this translates to a positive y value

#print x
#print y
#print z

print target_lat
print target_lon
print target_alt
