#! /usr/bin/env python

import numpy as np
import tf.transformations
import rospy

#from tf.transformations import quaternion_from_euler
from arm import Arm


calibration_response = 'n'
card_response = 100
home = False

rightArm = Arm('right')
leftArm = Arm('left')

#while(True):
#    print(rightArm.get_current_joints())
#    print(leftArm.get_current_joints())


#rightArm.tuck_arm()

while calibration_response != 'y' or not home:
    calibration_response = str(raw_input('please place the gripper arm at the center of the left hand side tag, press "y" to continue: '))
    home = leftArm.calibrate()
    if not home:
        print('something went wrong somewhere, please try again in a different position!')

while card_response < 0 or card_response > 3:
    card_response = int(input('please pick a card index, where the left most corner is 0 and the right most corner is 3: '))
    if card_response < 0 or card_response > 3:
        print('index out of range! please choose a number between 0 and 3!')

leftArm.choose_card(card_response)
rospy.spin()