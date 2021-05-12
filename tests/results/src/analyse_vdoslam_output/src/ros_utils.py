import numpy as np
from geometry_msgs.msg import Pose, Twist
from scipy.spatial.transform import Rotation as R



def pose_to_homogenous(pose: Pose):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    r = R.from_quat([pose.orientation.x,
                     pose.orientation.y,
                     pose.orientation.z,
                     pose.orientation.w])

    rot = r.as_matrix()

    homo = np.eye(4)
    homo[0:3, 0:3] = rot
    homo[0:3, 3] = [x, y, z]

    return np.array(homo)

def homogenous_to_pose(homo: np.array):
    assert(homo.shape == (4,4))
    translation = homo[0:3,3]
    rot = homo[0:3, 0:3]

    pose = Pose()
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]

    r = R.from_matrix(rot)
    quat = r.as_quat()

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose




if __name__ == "__main__":
    #for testing
    pose = Pose()
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    pose.position.x = 4
    pose.position.y = 5
    pose.position.z = 8

    homo = pose_to_homogenous(pose)
    print(homo)

    p = homogenous_to_pose(homo)
    print(p)

