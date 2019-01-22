#! /usr/bin/env python

import cv2
import baxter_interface
import rospy

from sensor_msgs.msg import Image

class Camera:
    def __init__(self, limb_name):
        self.limb_name = limb_name
        rospy.init_node(limb_name + '_camera', anonymous = True)
        display_pub = rospy.Publisher('/robot/xdisplay',Image)
        self.main() 

    def republish(self, msg):
        """
        Sends the camera image to baxter's display
        """             
        print(msg)
        display_pub.publish(msg)
    
    def main(self):
        head_camera = baxter_interface.CameraController("head_camera")
        head_camera.close()
        left_camera = baxter_interface.CameraController("left_hand_camera")
        left_camera.resolution =(960, 600)
        left_camera.open()
        camera_name = "left_hand_camera"
        sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image, self.republish,None,1)
        rospy.spin()


Camera('left')
