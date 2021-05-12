from loader import OutputResultsLoader
from ros_utils import *
from vdo_slam_types import *
from typing import List
import matplotlib.pyplot as plt
import numpy as np
import os
import random
from colour import Color

_RESULTS_FOLDER = "/home/jesse/Code/src/ros/src/multi_robot_perception/tests/results/src/analyse_vdoslam_output/results_folder"

print(_RESULTS_FOLDER)



def load_scenes(scenes_dict) -> List[Scene]:
    scenes = []
    for scene_dict in scenes_dict["scenes"]:
        scenes.append(Scene(scene_dict))
    return scenes


def load_gt_odom(odom_dict) -> List[Odometry]:
    gt_odom = []
    for gt_odom_dict in odom_dict["odom_gt"]:
        gt_odom.append(Odometry(gt_odom_dict))

    return gt_odom

class PlottingManager:

    def __init__(self, r_loader: OutputResultsLoader):
        self.scenes = load_scenes(r_loader.getMapDict())
        self.odom_ft = load_gt_odom(r_loader.getOdomGtDict())

        assert(len(self.scenes) == len(self.odom_ft))
        #is homogenous matrix
        #set initial transform
        # pose_odom_init_0 = pose_to_homogenous(self.odom_ft[0].pose)
        # pose_odom_init_1 = pose_to_homogenous(self.odom_ft[1].pose)
        # self.pose_odom_R =  pose_odom_init_0 @ pose_odom_init_1
        self.pose_odom_R  = pose_to_homogenous(self.odom_ft[0].pose)
        self.trajectory_dict = self.construct_trajetories_all()

    def _transform_pose(self, pose):
        hom_pose = pose_to_homogenous(pose)
        # pose_in_r = np.linalg.inv(self.pose_vio_R) @ hom_pose
        pose_in_r = np.linalg.inv(hom_pose)  @ self.pose_odom_R @ hom_pose
        # pose_in_r[0:3, 3] +=  self.pose_odom_R[0:3, 3]
        return homogenous_to_pose(pose_in_r)
        # return pose

    def write_results_for_trajectory_eval(self):
        gt_file = open(_RESULTS_FOLDER + "/stamped_groundtruth.txt", "w")
        vio_file = open(_RESULTS_FOLDER + "/stamped_traj_estimate.txt", "w")
        print("Writing trajectories")

        for scene in self.scenes:
            pose_r = self._transform_pose(scene.camera_pose)
            timestamp = scene.scene_time.toSec()

            vio_file.write("{:.10e} {} {} {} {} {} {} {}\n".format(timestamp, 
                pose_r.position.x, pose_r.position.y, pose_r.position.z, 
                pose_r.orientation.x, pose_r.orientation.y, pose_r.orientation.z, pose_r.orientation.w))  



        for odom in self.odom_ft:
            pose_r = odom.pose
            timestamp = odom.time.toSec()

            gt_file.write("{:.10e} {} {} {} {} {} {} {}\n".format(timestamp, 
                pose_r.position.x, pose_r.position.y, pose_r.position.z, 
                pose_r.orientation.x, pose_r.orientation.y, pose_r.orientation.z, pose_r.orientation.w))  

        gt_file.close()
        vio_file.close()


        


    def get_xy_scenes(self):
        xy = []
        for scene in self.scenes:
            pose_r = self._transform_pose(scene.camera_pose)
            x = pose_r.position.x
            y = pose_r.position.y 

            # x  = scene.camera_pose.position.x
            # y = scene.camera_pose.position.y

            xy.append([x, y])

        return np.array(xy)

    def get_xy_odom(self):
        xy = []
        # self.odom_offset_x = self.odom_ft[0].pose.position.x
        # self.odom_offset_y = self.odom_ft[0].pose.position.y

        for odom in self.odom_ft:
            # pose_r = self._transform_pose(odom.pose)
            # x = pose_r.position.x
            # y = pose_r.position.y 
            
            x = odom.pose.position.x
            y = odom.pose.position.y

            xy.append([x, y])

        return np.array(xy)

    def construct_trajetories_all(self):
        #make n tracking_id's by occurances by 2 (xy)
        #tracking Id will start at 1, so index is tracking_id - 1
        trajectory_dict = {}


        for scene in self.scenes:
            for scene_object in scene.scene_objects:
                tracking_id = scene_object.tracking_id
                pose_r = self._transform_pose(scene_object.pose)
                # pose_r = scene_object.pose

                # print(trajectory_dict)
                # print(tracking_id in trajectory_dict)

                if tracking_id not in trajectory_dict:
                    trajectory_dict[tracking_id] = [[pose_r.position.x,pose_r.position.y]]
                else:
                    trajectory_dict[tracking_id].append([pose_r.position.x,pose_r.position.y])

        return trajectory_dict

    def get_trajectory(self, id):
        if id in self.trajectory_dict:
            return np.array(self.trajectory_dict[id])
        else:
            print("{} is not in trajectory dict".format(id))


    def plot_trajectories(self):
        traj_dict = self.construct_trajetories_all()
        for id, path in traj_dict.items():
            color = Color(pick_for=id)
            path = np.array(path)

            rgb = (color.get_red(), color.get_green(), color.get_blue())

            plt.scatter(path[:, 0], path[:, 1], color =[rgb])


if __name__ == "__main__":
    results_loader = OutputResultsLoader("/home/jesse/Code/src/ros/src/multi_robot_perception/VDO_SLAM/output_results")
    pl = PlottingManager(results_loader)

    xy_scenes = pl.get_xy_scenes()
    xy_odom = pl.get_xy_odom()

    # pl.plot_trajectories()

    # trajectory = pl.get_trajectory(10)

    # plt.scatter(xy_scenes[:, 0], xy_scenes[:, 1], color='g')
    # plt.scatter(xy_odom[:, 0], xy_odom[:, 1], color='r')
    # plt.show()
    pl.write_results_for_trajectory_eval()
    
    

    
