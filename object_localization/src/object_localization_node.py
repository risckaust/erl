#! /usr/bin/env python

#Modules import
import rospy
import numpy as np
import tf
import datetime

from math import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from darknet_ros_msgs.msg import *

#==============================================================================
# 
#                               Constants
# 
#==============================================================================

# ZED M left camera matrix  (pixel units) - obtained from /zed/left/camera_info
# message type is sensor_msgs/CameraInfo
K = [677.0139770507812, 0.0, 635.8870239257812, 0.0, 677.0139770507812, 
     373.62017822265625, 0.0, 0.0, 1.0]
fx = K[0] #focal length along the x-axis
fy = K[4] #focal length along the y-axis
cx = K[2] #principal point - x coordinate
cy = K[5] #principal point - y coordinate

                         
#==============================================================================
# 
#                               Main loop
# 
#==============================================================================

def listener():
    rospy.init_node('listener', anonymous=True)
    box = bounding_boxes()    
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, box.cb_bounding_boxes)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, box.cb_pose)
    rospy.spin()


#==============================================================================
# 
#                               Library functions
# 
#==============================================================================
def pix2meters(x,y,Z):
#==============================================================================
#     x and y are the center coordinates of the bounding box
#     Z is the depth at x and y
#     This function returns x and y converted from pixels to meters
#                in the camera frame using the camera matrix
#==============================================================================   
    X = ((x-cx)*Z)/fx
    Y = ((y-cy)*Z)/fy
    return X,Y


#==============================================================================
#                   
#                               Callbacks
# 
#==============================================================================
class bounding_boxes(object):
    
    def __init__ (self):
        
        self.center = PoseStamped()
        self.pt_transformed = PoseStamped()        

        self.confidence = 0.0
        self.object_class = None
        
        self.tf_listener = tf.TransformListener()
        self.sp_pub = rospy.Publisher("/detected_object_3d_pos", PoseStamped, queue_size=10)
        
        #set default value for z: 1 meter
        self.center.pose.position.z = 1        
        
    def cb_bounding_boxes(self,msg):
              
        if not msg == None:# and msg.image_header.frame_id == "zed_left_camera_optical_frame":
            self.center.header.frame_id = "zed_left_camera_optical_frame"
            tmp_1 = sum(p.Class == "person" for p in msg.bounding_boxes)
            if tmp_1 > 0: #or tmp_3 > 0:
                tmp2 = max(msg.bounding_boxes, key=lambda x: x.probability)
                if tmp2.Class == "person": # or tmp2.Class == "kite":                
                    print "***************************************************"                
                    print "Class = ", self.object_class
                    print "Confidence = ", self.confidence
                    print "Found missing worker at time {}".format(rospy.Time.now())
                
                    self.center.pose.position.x = (tmp2.xmin+tmp2.xmax)/2
                    self.center.pose.position.y = (tmp2.ymin+tmp2.ymax)/2
                    self.confidence = tmp2.probability
                    self.object_class = tmp2.Class
                    
		    # below, use either default values or uncomment #self.center.pose.position.z if you want the actual height of the drone
                    XY = pix2meters(self.center.pose.position.x,self.center.pose.position.y,self.center.pose.position.z) #self.center.pose.position.z)
            
                    self.center.pose.position.x = XY[0]
                    self.center.pose.position.y = XY[1]

                
                    try:
                        self.tf_listener.waitForTransform("/zed_left_camera_optical_frame", "/local_origin", rospy.Time(), rospy.Duration(20.0))
                        self.pt_transformed = self.tf_listener.transformPose("/local_origin", self.center)
                        self.sp_pub.publish(self.pt_transformed)
                        print "center coordinates in /local_origin frame:"
                        print self.pt_transformed
                    except Exception, e:
                        print e                
                        print "fail"
	#elif not msg == None and msg.image_header.frame_id == "something_else":
		#pass
    
    def cb_pose(self,msg):
        if not msg == None:
            self.center.pose.position.z = msg.pose.position.z


if __name__ == '__main__':
    try:
        print "main"
        listener()
        
    except rospy.ROSInterruptException:
        print "exit"
        
        
