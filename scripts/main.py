#! /usr/bin/env python

import numpy as np
import tf.transformations
import rospy

from tf.transformations import quaternion_from_euler

from Arm import * 



rightArm = Arm('right')
rightArm.tuck_arm()

leftArm = Arm('left')

leftArm.calibrate()

print(leftArm.get_current_pose())

print(leftArm.get_current_joints())
#print(leftArm.get_current_pose())

#current = leftArm.get_current_pose()

#tuck_right_arm()
#leftArm.pose.position.x = current['position'].x
#leftArm.pose.position.y = current['position'].y - 0.02
#leftArm.pose.position.z = current['position'].z
#leftArm.set_pose()



