import baxter_interface


from geometry_msgs.msg import Pose


class Arm:
    def __init__(self, limb, pose):
        self.limb = baxter_interface.Limb(limb)
        self.pose = pose
        
        self.set_pose(self.pose)

    def set_pose(self, pose = self.pose):
        
        pass

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


