import baxter_interface
import rospy
import struct

from std_msgs.msg import Header
from baxter_core_msgs.msg import SolvePositionIKRequest, SolvePositionIK
from geometry_msgs.msg import Pose, PoseStamped


class Arm:
    def __init__(self, limb):
        self.limb = baxter_interface.Limb(limb)
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
        il_request.pose_stamp.append(PoseStamped(header=header, pose=self.pose))
        ik_namespace = 'ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
        try:
            ik_service = rospy.ServiceProxy(ik_namespace, SolvePositionIK)
            ik_response = ik_service(ik_request)
            ik_validation = struct.unpack('<%dB' % len(ik_response.result_type), ik_response.result_type)[0]
            if (ik_validation != ik_response.RESULT_INVALID):
                joint_position = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
                self.limb.move_to_joint_positions(joint_position)
            else:
                rospy.logerr('No Valid Pose Found!')
        except(rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr('Service call failed: %s' % (e,))

    def get_current_pose(self)
        return self.limb.endpoint.pose()


if __name__ == '__main__':
    leftArm = Arm('left')
    current = leftArm.get_current_pose()
    leftArm.pose.position.x = current['position'].x + 10
    leftArm.set_pose()

