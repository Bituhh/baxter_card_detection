#! /usr/bin/env python

import numpy as np
import tf.transformations
import rospy
import baxter_interface
import cv2
import threading

from cards_utils import Cards
from arm import Arm
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class EyeInHand:
    def __init__(self, limb_name):
        self.limb_name = limb_name
        self.arm = Arm(limb_name)
        self.available_tread = True
        if limb_name == 'left':
            self.other_limb_name = 'right'
        else:
            self.other_limb_name = 'left'
        self.otherArm = Arm(self.other_limb_name)
        #self.otherArm.tuck_arm()
        self.cards = np.array([])
        self.bridge = CvBridge()
        self.cards = Cards()
        #rospy.init_node(limb_name, anonymous=True)
        self.display_pub = rospy.Publisher('/robot/xdisplay', Image)
        head_camera = baxter_interface.CameraController('head_camera')
        head_camera.close()
        self.other_camera = baxter_interface.CameraController(self.other_limb_name + '_hand_camera')
        self.other_camera.close()
        self.camera = baxter_interface.CameraController(self.limb_name + '_hand_camera')
        self.camera.open()
        self.arm.calibrate()
        resp  = 'n'
        while resp is not 'y':
            resp = str(raw_input('Please place the gripper arm at the center of the card holder, press "y" to continue: '))
        self.arm.go_home()
        if self.available_tread:
            cam_thread = threading.Thread(target=self.get_camera)
            choices_thread = threading.Thread(target=self.pick_choices)
            cam_thread.start()
            choices_thread.start()
        rospy.spin()
        cv2.destroyAllWindows()

    def get_camera(self):
        self.available_tread = False
        sub = rospy.Subscriber('/cameras/' + self.limb_name + '_hand_camera/image', Image, self.callback, None, 1)

    def callback(self, msg):
        self.cards.img = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow('main',self.cards.img)
        cv2.waitKey(1)
        self.corners, self.contours, self.hierarchy = self.cards.get_features((5,5))
        for iterations, corner in enumerate(self.corners):
            card_img = self.cards.extract_card(corner)
            self.cards.append(card_img)

    def pick_choices(self):
        while not rospy.is_shutdown():
            card_response = 100
            while (card_response < 0 or card_response > 3) and not rospy.is_shutdown():
                card_response = int(input('please pick a card index, where the left most corner is 0 and the right most corner is 3: '))
                self.arm.choose_card(card_response)
                if card_response < 0 or card_response > 3:
                    print('index out of range! please choose a number between 0 and 3!')



EyeInHand('left')
