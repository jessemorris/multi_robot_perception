from geometry_msgs.msg import Pose, Twist


class BoundingBox:
    
    def __init__(self, class_dict):
        self.x = class_dict["x"]
        self.y = class_dict["y"]
        self.width = class_dict["width"]
        self.height = class_dict["height"]

class VDOTime:
    
    def __init__(self, class_dict):
        self.sec = class_dict["sec"]
        self.nsec = class_dict["nsec"]

    def toSec(self):
        return self.sec + self.nsec / 1e9

    def toNsec(self):
        return self.sec * 1e9 + self.nsec



class Odometry:

    def __init__(self, class_dict):
        
        pose = Pose()
        pose.position.x = -class_dict["pose"]["translation"]["y"]
        pose.position.y = class_dict["pose"]["translation"]["x"]
        pose.position.z = class_dict["pose"]["translation"]["z"]

        pose.orientation.x = class_dict["pose"]["rotation"]["x"]
        pose.orientation.y = class_dict["pose"]["rotation"]["y"]
        pose.orientation.z = class_dict["pose"]["rotation"]["z"]
        pose.orientation.w = class_dict["pose"]["rotation"]["w"]

        twist = Twist()
        twist.linear.x = class_dict["twist"]["linear"]["x"]
        twist.linear.y = class_dict["twist"]["linear"]["y"]
        twist.linear.z = class_dict["twist"]["linear"]["z"]

        twist.angular.x = class_dict["twist"]["angular"]["x"]
        twist.angular.y = class_dict["twist"]["angular"]["y"]
        twist.angular.z = class_dict["twist"]["angular"]["z"]

        self.pose = pose
        self.twist = twist
        self.time = VDOTime(class_dict["time"])

class SceneObject:

    def __init__(self, class_dict):
        self.semantic_instance_index = class_dict["semantic_instance_index"]
        self.label = class_dict["label"]
        self.tracking_id = class_dict["tracking_id"]
        self.frame_id = class_dict["frame_id"]
        self.unique_id = class_dict["unique_id"]
        self.diff_time = class_dict["diff_time"]
        self.bounding_box = BoundingBox(class_dict["bounding_box"])
        self.scene_time = VDOTime(class_dict["scene_time"])


        pose = Pose()
        pose.position.x = class_dict["pose"]["translation"]["x"]
        pose.position.y = class_dict["pose"]["translation"]["y"]
        pose.position.z = class_dict["pose"]["translation"]["z"]

        pose.orientation.x = class_dict["pose"]["rotation"]["x"]
        pose.orientation.y = class_dict["pose"]["rotation"]["y"]
        pose.orientation.z = class_dict["pose"]["rotation"]["z"]
        pose.orientation.w = class_dict["pose"]["rotation"]["w"]

        twist = Twist()
        twist.linear.x = class_dict["twist"]["linear"]["x"]
        twist.linear.y = class_dict["twist"]["linear"]["y"]
        twist.linear.z = class_dict["twist"]["linear"]["z"]

        twist.angular.x = class_dict["twist"]["angular"]["x"]
        twist.angular.y = class_dict["twist"]["angular"]["y"]
        twist.angular.z = class_dict["twist"]["angular"]["z"]

        self.pose = pose
        self.twist = twist

class Scene:

    def __init__(self, class_dict):
        pose = Pose()
        pose.position.x = class_dict["camera_pose"]["translation"]["x"]
        pose.position.y = class_dict["camera_pose"]["translation"]["y"]
        pose.position.z = class_dict["camera_pose"]["translation"]["z"]

        pose.orientation.x = class_dict["camera_pose"]["rotation"]["x"]
        pose.orientation.y = class_dict["camera_pose"]["rotation"]["y"]
        pose.orientation.z = class_dict["camera_pose"]["rotation"]["z"]
        pose.orientation.w = class_dict["camera_pose"]["rotation"]["w"]

        twist = Twist()
        twist.linear.x = class_dict["camera_twist"]["linear"]["x"]
        twist.linear.y = class_dict["camera_twist"]["linear"]["y"]
        twist.linear.z = class_dict["camera_twist"]["linear"]["z"]

        twist.angular.x = class_dict["camera_twist"]["angular"]["x"]
        twist.angular.y = class_dict["camera_twist"]["angular"]["y"]
        twist.angular.z = class_dict["camera_twist"]["angular"]["z"]

        self.camera_pose = pose
        self.camera_twist = twist
        self.scene_time = VDOTime(class_dict["scene_time"])
        self.scene_objects = []

        if "objects" in class_dict:
            for obj in class_dict["objects"]:
                self.scene_objects.append(SceneObject(obj))






