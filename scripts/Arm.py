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
        
        # Enabling baxter
        baxter_enabler = baxter_interface.RobotEnable(CHECK_VERSION)
        if not baxter_enabler.state().enabled:
            baxter_enabler.enable()
        
        # Initialising limbs
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(self.limb_name)
        
        # Initialising poses
        self.pose = Pose()
        current = self.get_current_pose()
        self.pose.position.x = current['position'].x
        self.pose.position.y = current['position'].y
        self.pose.position.z = current['position'].z
        self.pose.orientation.x = current['orientation'].x
        self.pose.orientation.y = current['orientation'].y
        self.pose.orientation.z = current['orientation'].z
        self.pose.orientation.w = current['orientation'].w
        if self.limb_name is 'left':
            self.x_displacement = -0.2
            self.y_displacement = -0.15
            self.slot_a_displacement = 0.0
            self.slot_b_displacement = 0.0
            self.slot_c_dlsplacement = 0.0
            self.slot_d_displacement = 0.0
        else:
            self.x_displacement = -0.2
            self.y_displacement = 0.15
            self.slot_a_displacement = -0.0
            self.slot_b_displacement = -0.0
            self.slot_c_dlsplacement = -0.0
            self.slot_d_displacement = -0.0

        self._calibration_waypoint = False
        self.home_waypoint = False


    def calculate_joints(self):
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(PoseStamped(header=header, pose=self.pose))
        ik_namespace = 'ExternalTools/' + self.limb_name + '/PositionKinematicsNode/IKService'
        try:
            ik_service = rospy.ServiceProxy(ik_namespace, SolvePositionIK)
            ik_response = ik_service(ik_request)
            ik_validation = struct.unpack('<%dB' % len(ik_response.result_type), ik_response.result_type)[0]
            if (ik_validation != ik_response.RESULT_INVALID):
                return dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            else:
                rospy.logerr('No Valid Pose Found!')
                return False
        except(rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr('Service call failed: %s' % (e,))
            return False

    def set_pose(self):
        joint_positions = self.calculate_joints()
        print(joint_positions)
        if (joint_positions):
            self.set_joints(joint_positions)

    def set_joints(self, joints):
        return self.limb.move_to_joint_positions(joints)

    def get_current_pose(self):
        return self.limb.endpoint_pose()

    def get_current_joints(self):
        return self.limb.joint_angles()

    def calibrate(self):
        # user interface
        # get current joints
        self._calibration_waypoint = self.get_current_pose() 
        self.pose.position.x = self._calibration_waypoint['position'].x + self.x_displacement
        self.pose.position.y = self._calibration_waypoint['position'].y + self.y_displacement
        self.home_waypoint = arm.calculate_joints()
        if not self.home_waypoint:
            rospy.logerr('Inverse Kinematics failed - home waypoint not calibrate, please try again!')
            return False
        else:
            return self.home_waypoint
        self.set_pose()
)
    def pick_card(self, index):
        if index is 0:
            # pick from first slot
            self.pose.position.x = self._calibration_waypoint['position'].x
            self.pose.position.y = self._calibration_waypoint['position'].y + self.slot_a_displacement
            self.set_orientation('vertical')

        elif index is 1:
            # pick from second slot
            self.pose.position.x = self._calibration_waypoint['position'].x
            self.pose.position.y = self._calibration_waypoint['position'].y + self.slot_b_displacement
            self.set_orientation('vertical')
        elif index is 2:
            # pick from third slot
            self.pose.position.x = self._calibration_waypoint['position'].x
            self.pose.position.y = self._calibration_waypoint['position'].y + self.slot_c_displacement
            self.set_orientation('vertical')
        elif index is 3:
            # pick from fourth slot
            self.pose.position.x = self._calibration_waypoint['position'].x
            self.pose.position.y = self._calibration_waypoint['position'].y + self.slot_d_displacement
            self.set_orientation('vertical')
        else:
            rospy.logerr('Index out of range - Card index must be between 0 - 3!')
            return False
        self.set_pose()

    def go_home(self):
        pass

    def set_orientation(self, orientation = 'horizontal'):
        if orientation = 'vertical':
            # set to vertical
            pass
        elif orientation is 'horizontal':
            # set to horizontal
            pass
        else:
            rospy.logerr('Orientation not defined - please select between "horizontal" or "vertical"!')

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













