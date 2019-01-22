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
        rospy.init_node(limb_name+'_arm_client', anonymous=True)
        
        # Enabling baxter
        baxter_enabler = baxter_interface.RobotEnable(CHECK_VERSION)
        if not baxter_enabler.state().enabled:
            baxter_enabler.enable()
        
        # Initialising limbs
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.gripper = baxter_interface.Gripper(limb_name, CHECK_VERSION)
        
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
        self.pick_displacement = 0.1
        self.give_card_x = 0.0
        self.give_card_y = 0.0
        self.give_card_z = 0.0
        
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

    def set_pose(self, joint_positions=None):
        if joint_positions:
            self.set_joints(joint_positions)
        else:
            joint_positions = self.calculate_joints()
            if joint_positions:
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
            rospy.loginfo('Arm calibrated!')
            return self.home_waypoint
        self.set_pose()
)
    def choose_card(self, index):
        if index is 0:
            # pick from first slot
            pick_card(self.slot_a_displacement)

        elif index is 1:
            # pick from second slot
            pick_card(self.slot_b_displacement)
        elif index is 2:
            # pick from third slot
            pick_card(self.slot_c_displacement)

        elif index is 3:
            # pick from fourth slot
            pick_card(self.slot_d_displacement)
        else:
            rospy.logerr('Index out of range - Card index must be between 0 - 3!')

    def pick_card(self, displacement):
        self.pose.position.x = self._calibration_waypoint['position'].x
        self.pose.position.y = self._calibration_waypoint['position'].y + displacement
        self.pose.position.z = self._calibration_waypoint['position'].z + self.pick_displacement
        self.set_orientation('vertical')
        self.set_pose()
        self.grip_action('open')
        self.pose.position.z = self._calibration_waypoint['position'].z
        self.grip_action('close')
        self.give_card()
        self.go_home()

    def grip_action(self, action):
        if action == 'open':
            self.gripper.open()
        elif action == 'close':
            self.gripper.close()
        else:
            rospy.logerr('Unknown gripper action, please choose between "open" or "close"')

    def give_card(self)
        self.pose.position.x = self.give_card_x
        self.pose.position.y = self.give_card_y
        self.pose.position.z = self.give_card_z

    def go_home(self):
        self.pose.position.x = self.home_waypoint['position'].x
        self.pose.position.y = self.home_waypoint['position'].y
        self.pose.position.z = self.home_waypoint['position'].z
        self.set_orientation('horizontal')

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
    leftArm.tuck_arm()
    rightArm = Arm('right')
    rightArm.tuck_arm()
