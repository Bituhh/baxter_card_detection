#!/usr/bin/env python

import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy
import struct

from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIKRequest, SolvePositionIK
from geometry_msgs.msg import Pose, PoseStamped


class Arm:
    def __init__(self, limb_name):
        rospy.init_node('rsdk_ik_service_client')
        baxter_enabler = baxter_interface.RobotEnable(CHECK_VERSION)
        baxter_enabler.enable()
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(self.limb_name)
        self.pose = Pose()
        current = self.get_current_pose()
        self.pose.position.x = current['position'].x
        self.pose.position.y = current['position'].y
        self.pose.position.z = current['position'].z
        self.pose.orientation.x = current['orientation'].x
        self.pose.orientation.y = current['orientation'].y
        self.pose.orientation.z = current['orientation'].z

    def set_pose(self):
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(PoseStamped(header=header, pose=self.pose))
        ik_namespace = 'ExternalTools/' + self.limb_name + '/PositionKinematicsNode/IKService'
        try:
            ik_service = rospy.ServiceProxy(ik_namespace, SolvePositionIK)
            ik_response = ik_service(ik_request)
            ik_validation = struct.unpack('<%dB' % len(ik_response.result_type), ik_response.result_type)[0]
            print(ik_validation)
            if (ik_validation != ik_response.RESULT_INVALID):
                joint_position = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
                print(joint_position)
                self.limb.move_to_joint_positions(joint_position)
            else:
                rospy.logerr('No Valid Pose Found!')
        except(rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr('Service call failed: %s' % (e,))

    def get_current_pose(self):
        return self.limb.endpoint_pose()

    def tuck_arm(self):
        if self.limb_name is 'left':
            joint_positions = {
                    'left_w0': -0.0003489641638001473,
                    'left_w1': 0.010915568977665657,
                    'left_w2': -0.0000802511382973492,
                    'left_e0': 2.6847146875244476,
                    'left_e1': 2.55128311639743,
                    'left_s0': -0.9388896568511722,
                    'left_s1': -1.2015999442359355}
	else:
            joint_positions = {
                    'right_s0': 0.926873588697747,
                    'right_s1': -1.2091710667545756, 
                    'right_w0': -0.0002909058219513483, 
                    'right_w1': 0.012275787814772343, 
                    'right_w2': -0.00004903657065913336, 
                    'right_e0': -2.6863739465112344, 
                    'right_e1': 2.551905038230334}
        self.limb.move_to_joint_positions(joint_positions)


if __name__ == '__main__':
    leftArm = Arm('left')
    current = leftArm.get_current_pose()
    print(leftArm.pose.position.x)
    leftArm.pose.position.x = 0.5
    print(leftArm.pose.position.x)
    leftArm.set_pose()













