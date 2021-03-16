#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <mask_rcnn/SemanticObject.h>

#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <iostream>

#include <vision_msgs/BoundingBox2D.h>

// #incl
#include <mono_depth_2/MonoDepthInterface.hpp>
#include <mask_rcnn/MaskRcnnInterface.hpp>
#include <flow_net/FlowNetInterface.hpp>

#include <vdo_slam/vdo_slam.hpp>

using namespace std;

void LoadData(const string &strPathToSequence, vector<string> &vstrFilenamesRGB, vector<string> &vstrFilenamesDEP,
              vector<double> &vTimestamps, vector<cv::Mat> &vPoseGT, vector<vector<float> > &vObjPoseGT);



/**
 * @brief Runs the example kitti dataset using inferred flow and masking but gt (provided depth).
 * 
 * This allows me to test how mich the depth effects the algorithm and if depth is correct will the 
 * rest of the algorithm work with me pipeline.
 * 
 * Loading functions taken from vdo_slam_exmaple.cc
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "kitti_gt_depth_node");
    ros::NodeHandle n;

    std::string path_to_sequence, vdo_yaml_file;
    int sensor_type;

    n.getParam("/path_to_sequence", path_to_sequence);
    n.getParam("/vdo_yaml_file", vdo_yaml_file);
    n.getParam("/sensor_type", sensor_type);

    VDO_SLAM::eSensor sensor;
    if (sensor_type == 0) {
        sensor = VDO_SLAM::eSensor::MONOCULAR;
        ROS_INFO_STREAM("Using Monocular");
    }
    else if (sensor_type == 1) {
        sensor = VDO_SLAM::eSensor::STEREO;
        ROS_INFO_STREAM("Using Stereo");
    }
    else if (sensor_type == 2) {
        sensor = VDO_SLAM::eSensor::RGBD;
        ROS_INFO_STREAM("Using RGBD");
    }
    else {
        ROS_WARN_STREAM("Invalid sensor type");
    }

    ROS_INFO_STREAM(path_to_sequence);
    ROS_INFO_STREAM(vdo_yaml_file);

    ROS_INFO_STREAM(sensor);


    // Retrieve paths to images
    vector<string> vstrFilenamesRGB;
    vector<string> vstrFilenamesDEP;
    std::vector<cv::Mat> vPoseGT;
    vector<vector<float> > vObjPoseGT;
    vector<double> vTimestamps;

    LoadData(path_to_sequence, vstrFilenamesRGB, vstrFilenamesDEP,
                  vTimestamps, vPoseGT, vObjPoseGT);

    // save the id of object pose in each frame
    vector<vector<int> > vObjPoseID(vstrFilenamesRGB.size());
    for (int i = 0; i < vObjPoseGT.size(); ++i)
    {
        int f_id = vObjPoseGT[i][0];
        if (f_id>=vstrFilenamesRGB.size())
            break;
        vObjPoseID[f_id].push_back(i);
    }


    // Check consistency in the number of images, depth maps, segmentations and flow maps
    int nImages = vstrFilenamesRGB.size();

    mask_rcnn::MaskRcnnInterface mask_rcnn(n);
    flow_net::FlowNetInterface flow_net(n);

    VDO_SLAM::System slam(vdo_yaml_file,sensor);


    mask_rcnn.start_service();
    flow_net.start_service();
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    cout << endl << "--------------------------------------------------------------------------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // namedWindow( "Trajectory", cv::WINDOW_AUTOSIZE);
    cv::Mat imTraj = cv::Mat::zeros(800, 600, CV_8UC3);


    cv::Mat imRGB, imD, mTcw_gt, flow_mat, mask_mat, prev_rgb;
    cv::Mat tracked_mask_converted;
    bool is_first = true;
    for(int ni=0; ni<nImages; ni++) {
         // Read imreadmage and depthmap from file
        cout << vstrFilenamesRGB[ni] << endl;
        imRGB = cv::imread(vstrFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD   = cv::imread(vstrFilenamesDEP[ni],CV_LOAD_IMAGE_UNCHANGED);

        if (is_first) {
            prev_rgb = imRGB;
            is_first = false;
            continue;
        }
        cv::Mat imD_f, imD_r;

        imD.convertTo(imD_f, CV_16UC1);

        std::vector<mask_rcnn::SemanticObject> semantic_objects;
        cv::Mat mask_viz, tracked_mask;
        mask_rcnn.analyse(imRGB, mask_mat, mask_viz, semantic_objects);

        //NOTE: in order to display we convert to a CV_32SC1!!!! This doesnt make sense!!! 
        // mask_mat.convertTo(mask_mat, CV_32SC1);



        // mask_mat.convertTo(mask_mat, CV_32SC1);
        cv::Mat viz;
        flow_net.analyse(imRGB,prev_rgb, flow_mat, viz);


        double tframe = vTimestamps[ni];
        mTcw_gt = vPoseGT[ni];

        // Object poses in current frame
        vector<vector<float> > vObjPose_gt(vObjPoseID[ni].size());
        for (int i = 0; i < vObjPoseID[ni].size(); ++i) {
            vObjPose_gt[i] = vObjPoseGT[vObjPoseID[ni][i]];
        }

        auto sceneptr = slam.TrackRGBD(imRGB, imD_f, flow_mat, mask_mat,mTcw_gt,vObjPose_gt,tframe,imTraj,nImages);

        cv::imshow("MASK Viz", mask_viz);
        cv::waitKey(1);

        // cv::imshow("Flow", viz);
        // cv::waitKey(1);

        prev_rgb = imRGB.clone();
    }



}


void LoadData(const string &strPathToSequence, vector<string> &vstrFilenamesRGB,vector<string> &vstrFilenamesDEP,
              vector<double> &vTimestamps, vector<cv::Mat> &vPoseGT, vector<vector<float> > &vObjPoseGT)
{
    // +++ timestamps +++
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    fTimes.close();

    // +++ image, depth, semantic and moving object tracking mask +++
    string strPrefixImage = strPathToSequence + "/image_0/";         // image  image_0
    string strPrefixDepth = strPathToSequence + "/depth/";           // depth_gt  depth
    // string strPrefixSemantic = strPathToSequence + "/semantic/";     // semantic_gt  semantic
    // string strPrefixFlow = strPathToSequence + "/flow/";             // flow_gt  flow

    const int nTimes = vTimestamps.size();
    vstrFilenamesRGB.resize(nTimes);
    vstrFilenamesDEP.resize(nTimes);
    // vstrFilenamesSEM.resize(nTimes);
    // vstrFilenamesFLO.resize(nTimes);


    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrFilenamesRGB[i] = strPrefixImage + ss.str() + ".png";
        vstrFilenamesDEP[i] = strPrefixDepth + ss.str() + ".png";
        // vstrFilenamesSEM[i] = strPrefixSemantic + ss.str() + ".txt";
        // vstrFilenamesFLO[i] = strPrefixFlow + ss.str() + ".flo";
    }


    // +++ ground truth pose +++
    string strFilenamePose = strPathToSequence + "/pose_gt.txt"; //  pose_gt.txt  kevin_extrinsics.txt
    // vPoseGT.resize(nTimes);
    ifstream fPose;
    fPose.open(strFilenamePose.c_str());
    while(!fPose.eof())
    {
        string s;
        getline(fPose,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            int t;
            ss >> t;
            cv::Mat Pose_tmp = cv::Mat::eye(4,4,CV_32F);
            ss >> Pose_tmp.at<float>(0,0) >> Pose_tmp.at<float>(0,1) >> Pose_tmp.at<float>(0,2) >> Pose_tmp.at<float>(0,3)
               >> Pose_tmp.at<float>(1,0) >> Pose_tmp.at<float>(1,1) >> Pose_tmp.at<float>(1,2) >> Pose_tmp.at<float>(1,3)
               >> Pose_tmp.at<float>(2,0) >> Pose_tmp.at<float>(2,1) >> Pose_tmp.at<float>(2,2) >> Pose_tmp.at<float>(2,3)
               >> Pose_tmp.at<float>(3,0) >> Pose_tmp.at<float>(3,1) >> Pose_tmp.at<float>(3,2) >> Pose_tmp.at<float>(3,3);

            vPoseGT.push_back(Pose_tmp);
            // if(t==410)
            //     cout << "ground truth pose 0 (for validation):" << endl << vPoseGT[t] << endl;
        }
    }
    fPose.close();


    // +++ ground truth object pose +++
    string strFilenameObjPose = strPathToSequence + "/object_pose.txt";
    ifstream fObjPose;
    fObjPose.open(strFilenameObjPose.c_str());

    while(!fObjPose.eof())
    {
        string s;
        getline(fObjPose,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            std::vector<float> ObjPose_tmp(10,0);
            ss >> ObjPose_tmp[0] >> ObjPose_tmp[1] >> ObjPose_tmp[2] >> ObjPose_tmp[3]
               >> ObjPose_tmp[4] >> ObjPose_tmp[5] >> ObjPose_tmp[6] >> ObjPose_tmp[7]
               >> ObjPose_tmp[8] >> ObjPose_tmp[9];

            vObjPoseGT.push_back(ObjPose_tmp);

        }
    }
    fObjPose.close();

}
