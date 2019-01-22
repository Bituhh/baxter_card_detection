#!/usr/bin/env python

import baxter_interface
from tf.transformations import quaternion_from_euler
from baxter_interface import CHECK_VERSION
import rospy
import struct
from math import pi

from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIKRequest, SolvePositionIK
from geometry_msgs.msg import Pose, PoseStamped


class Arm:
    def __init__(self, limb_name, enable=True):
        rospy.init_node('rsdk_ik_service_client', anonymous=True)

        # Enabling baxter
        self.baxter_enabler = baxter_interface.RobotEnable(CHECK_VERSION)
        if not self.baxter_enabler.state().enabled and enable:
            self.baxter_enabler.enable()

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
            self.x_displacement = -0.1
            self.y_displacement = -0.15
            self.slot_a_displacement = -0.045
            self.slot_b_displacement = -0.110
            self.slot_c_dlsplacement = -0.165
            self.slot_d_displacement = -0.215
        else:
            self.x_displacement = 0.1
            self.y_displacement = 0.15
            self.slot_a_displacement = 0.045
            self.slot_b_displacement = 0.110
            self.slot_c_dlsplacement = 0.165
            self.slot_d_displacement = 0.215
        self.pick_displacement = 0.05
        self.give_card_x = 0.9
        self.give_card_y = 0.5
        self.give_card_z = 0.4

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
        # get current joints
    	if self.limb_name is 'left':
    		joint_positions = {
                        'left_w0': -0.018791264651596317,
                        'left_w1': -1.4737720419609113,
                        'left_w2': -0.08360195293975504,
                        'left_e0': 0.2059369207736168,
                        'left_e1': 1.097946748928985,
                        'left_s0': -0.6465729020937019,
                        'left_s1': 0.32482043183473636
                        }
    	else:
    		return False
    	self.set_joints(joint_positions)
        self._calibration_waypoint = self.get_current_pose()
        self.pose.position.x = self._calibration_waypoint['position'].x + self.x_displacement
        self.pose.position.y = self._calibration_waypoint['position'].y + self.y_displacement
        self.home_waypoint = self.get_current_pose()
        self.set_orientation('horizontal')
        self.set_pose()
        rospy.loginfo('Arm calibrated!')
        return self.home_waypoint

    def choose_card(self, index):
        if index is 0:
            # pick from first slot
            self.pick_card(self.slot_a_displacement)
        elif index is 1:
            # pick from second slot
            self.pick_card(self.slot_b_displacement)
        elif index is 2:
            # pick from third slot
            self.pick_card(self.slot_c_displacement)
        elif index is 3:
            # pick from fourth slot
            self.pick_card(self.slot_d_displacement)
        else:
            rospy.logerr('Index out of range - Card index must be between 0 - 3!')

    def pick_card(self, displacement):
        self.pose.position.x = self._calibration_waypoint['position'].x
        self.pose.position.y = self._calibration_waypoint['position'].y + displacement
        self.pose.position.z = self._calibration_waypoint['position'].z + self.pick_displacement
        self.set_orientation('horizontal')
        self.set_pose()
        self.grip_action('open')
        self.pose.position.z = self._calibration_waypoint['position'].z
        self.set_pose()
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

    def give_card(self):
        self.pose.position.x = self.give_card_x
        self.pose.position.y = self.give_card_y
        self.pose.position.z = self.give_card_z
        self.set_pose()

    def go_home(self):
        self.pose.position.x = self.home_waypoint['position'].x
        self.pose.position.y = self.home_waypoint['position'].y
        self.pose.position.z = self.home_waypoint['position'].z
        self.set_orientation('horizontal')
        self.set_pose()

    def set_orientation(self, orientation = 'horizontal'):
        if orientation is 'vertical':
            # set to vertical
            orientation_values = quaternion_from_euler(0, pi, 0)
        elif orientation is 'horizontal':
            # set to horizontal
            orientation_values = quaternion_from_euler(0, -pi/2, 0)
        else:
            rospy.logerr('Orientation not defined - please select between "horizontal" or "vertical"!')
        print(orientation_values)
        self.pose.orientation.x = orientation_values[0]
        self.pose.orientation.y = orientation_values[1]
        self.pose.orientation.z = orientation_values[2]
        self.pose.orientation.w = orientation_values[3]
        print(self.pose.orientation)

    def tuck_arm(self):
        if self.limb_name is 'left':
            joint_positions = {
                    'left_w0': 0.10776215034895031,
                    'left_w1': 0.09050486648523941,
                    'left_w2': -0.08360195293975504,
                    'left_e0': 2.9160974777701716,
                    'left_e1': 2.5448741271019015,
                    'left_s0': -0.8490583660945765,
                    'left_s1': -2.1794032043882017
                    }
	else:
            joint_positions = {
                    'right_s0': 0.8536603084582327,
                    'right_s1': -2.182087670767001,
                    'right_w0': -0.1169660350762628,
                    'right_w1': 0.3163835375013666,
                    'right_w2': 0.10929613113683573,
                    'right_e0': -2.8562722270426404,
                    'right_e1': 2.541039175132188
                    }
        self.limb.move_to_joint_positions(joint_positions)

    def set_enabler(self, state):
        if state:
            self.baxter_enabler.enable()
        else:
            self.baxter_enabler.disable()


if __name__ == '__main__':
    leftArm = Arm('left')
    leftArm.tuck_arm()
    rightArm = Arm('right')
    rightArm.tuck_arm()
