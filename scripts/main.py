#! /usr/bin/env python

import numpy as np

from Arm import *

def calibrate():
    pass

leftArm = Arm('right')
leftArm.tuck_arm()
#print(leftArm.get_current_pose())

#print(leftArm.limb.joint_angles())
#print(leftArm.get_current_pose())

#current = leftArm.get_current_pose()

#tuck_right_arm()
#leftArm.pose.position.x = current['position'].x
#leftArm.pose.position.y = current['position'].y - 0.02
#leftArm.pose.position.z = current['position'].z
#leftArm.set_pose()



