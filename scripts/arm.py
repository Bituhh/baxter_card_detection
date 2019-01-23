#!/usr/bin/env python

import baxter_interface
from tf.transformations import quaternion_from_euler
from baxter_interface import CHECK_VERSION
import rospy
import struct
from math import pi
import time

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

        # Initialising limb
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.gripper = baxter_interface.Gripper(limb_name, CHECK_VERSION)
        self.keys = [ 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2' ]
        self.home = [ -0.4759175394414496, 2.2227381616459647, -0.9579710020344409, -0.42836413501700177, -0.5614369683660614, -1.4898788402337082, 0.2822524649709161 ]
        self.slot_a = [ 0.768140879533621, 0.6699661091089545, -1.153553552489831, -0.05138835639416136, -0.8179952551398969, 0.9541360500647273, 1.5780827355371194 ]
        self.slot_a_offset = [0.7612379659881365, 0.6323835798057618, -1.1278593742927505, -0.12310195822780445, -0.7489661196850532, 1.100247720110813, 1.5358982638702705 ]
        self.slot_b = [ 0.8153107887610974, 0.47131559707779336, -1.1777137498990264, 0.02032524543948173, -0.8245146734884099, 1.050393344504537, 1.4365730078546899 ]
        self.slot_b_offset = [ 0.8149272935641261, 0.47668452983539233, -1.1731118075353701, -0.058674765136617076, -0.8030389424580141, 1.177330254702055, 1.498315734567078 ]
        self.slot_c = [ 0.8686166211401155, 0.3765922834258691, -1.2057088992779352, 0.06519418348513008, -0.9215389583221623, 1.075320532307675, 1.3249759055360262 ]
        self.slot_c_offset = [ 0.8532768132612614, 0.4479223900625408, -1.2264176399143882, -0.04180097646987752, -0.883189438625027, 1.12210694633818, 1.3721458147635026 ]
        self.slot_d = [ 0.9878836273982065, 0.6143593055481082, -1.4308205799001197, 0.045252433242619704, -1.017796252761972, 0.8532768132612614, 1.3146215352177997 ]
        self.slot_d_offset = [ 0.8862574002007978, 0.7186699991243164, -1.4281361135213202, -0.0782330201821561, -0.9387962421858732, 0.884339924215941, 1.4112623248545806 ]
        self.adjust = [ 0.3888641297289524, 0.9311263382464462, -1.4112623248545806, 0.35473305719850196, -0.8436894333369775, -1.07953897947436, 3.001233411497812 ]
        self.hand_card = [ 1.0316020798529408, 0.10085923680346595, -0.8440729285339489, 0.08590292412158317, -1.3652429012180183, -0.9330438142313029, 1.8388594694776397 ]
        self.safe_state = [ -0.18062623777350748, 1.8396264598715824, -0.6385195029573034, -1.1355292782321775, -0.1292378813793461, 0.8862574002007978, -0.05675728915176031 ]
        # Initialising poses - Inverse Kinematics
        self.pose = Pose()

    def set_joints(self, joints):
        print(joints)
        return self.limb.move_to_joint_positions(joints)

    def get_current_joints(self):
        return self.limb.joint_angles()

    def calibrate(self):
    	if self.limb_name is 'left':
            self.gripper.calibrate()
    		adjust = self.create_joint_position(self.keys, self.adjust)
    	else:
            # ToDo: implements right arms joints positions
    		return False

        self.set_joints(adjust)
        self.grip_action('open')
        rospy.loginfo('Arm calibrated!')

    def go_home(self):
        home = self.create_joint_position(self.keys, self.home)
        self.set_joints(home)

    def choose_card(self, index):
        self.set_joints(self.create_joint_position(self.keys, self.safe_state))
        if index is 0:
            self.grip_action('open')
            self.set_joints(self.create_joint_position(self.keys, self.slot_a_offset))
            self.set_joints(self.create_joint_position(self.keys, self.slot_a))
            self.grip_action('close')
            self.set_joints(self.create_joint_position(self.keys, self.slot_a_offset))
            print('sleeping')
            time.sleep(5)

        elif index is 1:
            self.grip_action('open')
            self.set_joints(self.create_joint_position(self.keys, self.slot_b_offset))
            self.set_joints(self.create_joint_position(self.keys, self.slot_b))
            self.grip_action('close')
            self.set_joints(self.create_joint_position(self.keys, self.slot_b_offset))
        elif index is 2:
            # pick from third slot
            self.grip_action('open')
            self.set_joints(self.create_joint_position(self.keys, self.slot_c_offset))
            self.set_joints(self.create_joint_position(self.keys, self.slot_c))
            self.grip_action('close')
            self.set_joints(self.create_joint_position(self.keys, self.slot_c_offset))
        elif index is 3:
            # pick from fourth slot
            self.grip_action('open')
            self.set_joints(self.create_joint_position(self.keys, self.slot_d_offset))
            self.set_joints(self.create_joint_position(self.keys, self.slot_d))
            self.grip_action('close')
            self.set_joints(self.create_joint_position(self.keys, self.slot_d_offset))
        else:
            rospy.logerr('Index out of range - Card index must be between 0 - 3!')
        self.give_card()

    def give_card(self):
        self.set_joints(self.create_joint_position(self.keys, self.hand_card))
        self.grip_action('open')
        self.go_home()

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

    def tuck_arm(self):
        if self.limb_name is 'left':
            joint_positions = {'left_w0': 0.10776215034895031,'left_w1': 0.09050486648523941,'left_w2': -0.08360195293975504,'left_e0': 2.9160974777701716,'left_e1': 2.5448741271019015,'left_s0': -0.8490583660945765,'left_s1': -2.1794032043882017 }
        else:
           joint_positions = {'right_s0': 0.8536603084582327,'right_s1': -2.182087670767001,'right_w0': -0.1169660350762628,'right_w1': 0.3163835375013666,'right_w2': 0.10929613113683573,'right_e0': -2.8562722270426404,'right_e1': 2.541039175132188 }
        self.set_joints(joint_positions)

    def set_enabler(self, state):
        if state:
            self.baxter_enabler.enable()
        else:
            self.baxter_enabler.disable()

    def create_joint_position(self, keys, joints):
        return dict(zip(keys, joints))

    def calculate_joints(self):
        # Inverse Kinematics, could not get the task to work with this, left for later projects
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
        # Inverse Kinematics, could not get the task to work with this, left for later projects
        joint_positions = self.calculate_joints()
        if joint_positions:
            self.set_joints(joint_positions)

    def get_current_pose(self):
            # Inverse Kinematics, could not get the task to work with this, left for later projects
            return self.limb.endpoint_pose()

    def set_orientation(self, orientation = 'horizontal'):
            # Inverse Kinematics, could not get the task to work with this, left for later projects
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


if __name__ == '__main__':
    leftArm = Arm('left')
    leftArm.tuck_arm()
    rightArm = Arm('right')
    rightArm.tuck_arm()
