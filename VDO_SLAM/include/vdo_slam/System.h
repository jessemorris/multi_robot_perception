/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/


#ifndef VDO_SLAM_SYSTEM_H
#define VDO_SLAM_SYSTEM_H

#include <string>
#include <thread>
#include <memory>
#include <opencv2/core/core.hpp>

#include "vdo_slam/Tracking.h"
#include "vdo_slam/Map.h"
#include "vdo_slam/Scene.h"

using namespace std;


namespace VDO_SLAM {


    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

    /**
     * @brief Define a struct that contains all the params needed to initalise the System. Can be used
     * as a replacement for the yaml settings file. 
     * 
     */
    struct VdoParams {

        //Camera intrinsics
        float fx;
        float fy;
        float cx;
        float cy;

        //distortion params
        float k1;
        float k2;
        float p1;
        float p2;
        float p3 = 0;

        //image size
        int width;
        int height;

        int fps;

        // stero baseline times fx
        float bf;

        // Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
        int RGB;

        //System Params


        int data_code;

        eSensor sensor_type;

        float depth_map_factor;

        // Close/Far Depth threshold
        float thdepth_bg;
        float thdepth_obj;

        // Max Tracking Points on Background and Object in each frame
        float max_track_points_bg;
        float max_track_points_obj;

        // Scene Flow Magnitude and Distribution Threshold
        float sf_mg_thresh;
        float sf_ds_thresh;

        // Window Size and Overlapping Size for Local Batch Optimization
        int window_size;
        int overlap_size;

        //  Use sampled feature or detected feature for background (1: sampled, 0: detected)
        int use_sample_feature;

        // Orb Params

        // Number of features per image
        int n_features;

        // Scale factor between levels in the scale pyramid
        float scale_factor;

        // Number of levels in the scale pyramid
        int n_levels;

        // Fast threshold
        int ini_th_fast;
        int min_th_fast;

    };

    typedef const std::shared_ptr<VdoParams> VdoParamsConstPtr;
    typedef std::shared_ptr<VdoParams> VdoParamsPtr;


    class Map;
    class Tracking;

    class System
    {

    public:

        // Initialize the SLAM system.
        System(const string &strSettingsFile, const eSensor sensor);
        System(const VdoParams& params);


        // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
        // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Input depthmap: Float (CV_32F).
        // Returns the camera pose (empty if tracking fails).
        std::unique_ptr<Scene> TrackRGBD(const cv::Mat &im, cv::Mat &depthmap, const cv::Mat &flowmap, const cv::Mat &masksem,
                        const cv::Mat &mTcw_gt, const vector<vector<float> > &vObjPose_gt, const double &timestamp,
                        cv::Mat &imTraj, const int &nImage);

        void SaveResultsIJRR2020(const string &filename);

    private:

        // Input sensor
        VDO_SLAM::eSensor mSensor;

        // Map structure.
        VDO_SLAM::Map* mpMap;

        // Tracker. It receives a frame and computes the associated camera pose.
        VDO_SLAM::Tracking* mpTracker;

    };

}// namespace VDO_SLAM

#endif // SYSTEM_H
