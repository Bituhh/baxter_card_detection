import baxter_interface
import rospy

from std_msgs.msg import Header
from baxter_core_msgs.msg import SolvePositionIKRequest
from geometry_msgs.msg import Pose, PoseStamped


class Arm:
    def __init__(self, limb):
        self.limb = baxter_interface.Limb(limb)
        self.pose = Pose()
        self.set_pose(self.pose)

    def set_pose(self):
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        request = SolvePositionIKRequest()
        request.pose_stamp.append(PoseStamped(header=header, pose=self.ik_pose))

    def get_current_pose(self)
        return self.limb.endpoint.pose()


leftPose = {
        pos_x: 0,
        pos_y: 0,
        pos_z: 0,
        rot_x: 0,
        rot_y: 0,
        rot_z: 0}
leftArm = Arm('left', leftPose)

rightPose = {
        pos_x: 0,
        pos_y: 0,
        pos_z: 0,
        rot_x: 0,
        rot_y: 0,
        rot_z: 0}
rightArm = Arm('right', rightPose)

leftArm.set_pose()


# Loop similar to arduino setup loop!
def setup():
    print('Im in setup')
    pass

def loop():
    print('Im in loop')
    #return True 




status = True
setup()
while (not loop()):
    pass


