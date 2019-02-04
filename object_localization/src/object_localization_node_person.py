#! /usr/bin/env python

# Modules Import
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
from yolo_ros_vino.msg import *

# message type is sensor_msgs/CameraInfo
K = [919.3767700195312, 0.0, 648.4270629882812, 0.0, 919.3844604492188, 355.51776123046875, 0.0, 0.0, 1.0]
fx = K[0] #focal length along the x-axis
fy = K[4] #focal length along the y-axis
cx = K[2] #principal point - x coordinate
cy = K[5] #principal point - y coordinate

# Main loop
def listener():
    rospy.init_node('listener', anonymous=True)
    box = bounding_boxes()
    rospy.Subscriber("/person/yolo_ros_vino/bounding_boxes", BoundingBoxes, box.cb_bounding_boxes)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, box.cb_pose)
    rospy.spin()

# Library functions
def pix2meters(x,y,Z):
    X = ((x - cx) * Z) / fx
    Y = ((y - cy) * Z) / fy
    return X,Y

# Callbacks
class bounding_boxes(object):
    def __init__ (self):
        self.detections             =   {"person": 0, "Red": 0, "Green": 0, "Blue": 0}          # contains object names and counter of how many of each object were detected
        self.topic_name             =   {"person": "/detected_object_3d_pos", "Red": "/detected_red_tag", "Green": "/detected_green_tag", "Blue": "/detected_blue_tag"}
        self.publisher              =   {}
        for key in self.topic_name:
            self.publisher[key]     =   rospy.Publisher(self.topic_name[key], PoseStamped, queue_size=10)
        self.center                 =   PoseStamped()
        self.pt_transformed         =   PoseStamped()
        self.tf_listener            =   tf.TransformListener()
        self.center.pose.position.z = 1     # initlizes z to 1 meter in case mavros fails

    def cb_bounding_boxes(self,msg):
        if not msg == None:
            for key in self.detections:
                self.detections[key] = sum(p.Class == key for p in msg.bounding_boxes)    # counts the number of detections per object
                print "Detected", self.detections[key], "occurences of ", key

            for bounding_box in msg.bounding_boxes:
                # person is detected from down looking camera while color tags are detected from down looking camera
                if bounding_box.Class == "person":
                    self.zed_frame_id = "down_color_optical_frame"
                else:
                    self.zed_frame_id = "front_color_optical_frame"

                self.center.header.frame_id = self.zed_frame_id
                self.confidence = bounding_box.probability
                self.object_class = bounding_box.Class

                tmp_x = (bounding_box.xmin + bounding_box.xmax) / 2
                tmp_y = (bounding_box.ymin + bounding_box.ymax) / 2

                self.center.pose.position.x, self.center.pose.position.y = pix2meters(tmp_x, tmp_y, self.center.pose.position.z)

                print "*******************************************************"
                print "Class = ", self.object_class
                print "Confidence = ", self.confidence
                #print "Found missing worker at time {}".format(rospy.Time.now())

                try:
                    self.tf_listener.waitForTransform(self.zed_frame_id, "/local_origin", rospy.Time(), rospy.Duration(1.0))
                    self.pt_transformed = self.tf_listener.transformPose("/local_origin", self.center)
                    self.publisher[self.object_class].publish(self.pt_transformed)
                    print "Center coordinates in /local_origin frame:"
                    print self.pt_transformed

                except Exception, e:
                    print e
                    print "There was an error either in TF transform or publishing the setpoint"

    def cb_pose(self,msg):
        if not msg == None:
            self.center.pose.position.z = msg.pose.position.z

if __name__ == '__main__':
    try:
        print "main"
        listener()

    except rospy.ROSInterruptException:
        print "exit"
