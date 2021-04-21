#include "preprocessing/ImagePreprocessing.hpp"
#include "utils/RosUtils.hpp"
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <memory>

#include <mask_rcnn/SemanticObject.h>
#include <vision_msgs/BoundingBox2D.h>
#include <realtime_vdo_slam/VdoInput.h>
#include "CameraInformation.hpp"
#include <vdo_slam/System.h>
#include <vdo_slam/utils/Types.h>
#include <vdo_slam/utils/VdoUtils.h>

using namespace VDO_SLAM;
using namespace VDO_SLAM::preprocessing;


BaseProcessing::BaseProcessing(ros::NodeHandle& n) :
        handler(n),
        image_transport(n),
        is_first(true),
        scene_flow_success(false),
        mask_rcnn_success(false),
        mono_depth_success(false)
{
    handler.getParam("/vdo_preprocessing/rgb_topic", rgb_topic);
    handler.getParam("/vdo_preprocessing/depth_topic", depth_topic);
    handler.getParam("/vdo_preprocessing/seg_topic", seg_topic);
    handler.getParam("/vdo_preprocessing/flow_topic", flow_topic);
    handler.getParam("/vdo_preprocessing/rgb_info", rgb_info);
    handler.getParam("/apply_undistortion", undistord_images);


    input_type = BaseProcessing::get_input_type(handler);

    if (input_type == InputType::INVALID) {
        ROS_ERROR_STREAM("Invalid input configuration.");
        ros::shutdown();
    }

    // sceneflow = flow_ptr;
    // mask_rcnn_interface = mask_ptr;
    // mono_depth = mono_ptr;
    // midas_depth(n),

    rgb_repub = image_transport.advertise("/vdoslam/input/camera/rgb/image_raw", 10);

    maskrcnn_raw = image_transport.advertise("/vdoslam/input/camera/mask/image_raw", 10);
    maskrcnn_viz = image_transport.advertise("/vdoslam/input/camera/mask/colour_mask", 10);

    flownet_raw = image_transport.advertise("/vdoslam/input/camera/flow/image_raw", 10);
    flownet_viz = image_transport.advertise("/vdoslam/input/camera/flow/colour_map", 10);
    monodepth_raw = image_transport.advertise("/vdoslam/input/camera/depth/image_raw", 10);

    vdo_input_pub = handler.advertise<realtime_vdo_slam::VdoInput>("/vdoslam/input/all", 10);

    //todo capture camera info topic

}


BaseProcessing::~BaseProcessing() {}


InputType BaseProcessing::get_input_type(ros::NodeHandle& n) {
    std::string rgb_topic;
    std::string camera_info_topic;
    std::string depth_topic;
    std::string seg_topic;
    std::string flow_topic;
    n.getParam("/vdo_preprocessing/rgb_topic", rgb_topic);
    n.getParam("/vdo_preprocessing/depth_topic", depth_topic);
    n.getParam("/vdo_preprocessing/seg_topic", seg_topic);
    n.getParam("/vdo_preprocessing/flow_topic", flow_topic);
    // handler.getParam("/vdo_preprocessing/rgb_info", rgb_info);

    if (!flow_topic.empty() && !seg_topic.empty() && !depth_topic.empty() && !rgb_topic.empty()) {
        return InputType::RGB_DEPTH_SEG_FLOW;
    }
    else if (!seg_topic.empty() && !depth_topic.empty() && !rgb_topic.empty()) {
        return InputType::RGB_DEPTH_SEG;
    }
    else if (!depth_topic.empty() && !rgb_topic.empty()) {
        ROS_INFO_STREAM("Preprocessing type is RGB-D");
        return InputType::RGB_DEPTH;
    }
    else if (!rgb_topic.empty()) {
        ROS_INFO_STREAM("Preprocessing type is RGB");
        return InputType::RGB;
    }
    else {
        return InputType::INVALID;
    }

}


void BaseProcessing::undistortImage(cv::Mat& input, cv::Mat& undistorted) {
    camera_info->apply_undistortion(input,undistorted);
}

inline bool BaseProcessing::using_rgb_topic() {return !rgb_topic.empty();}
inline bool BaseProcessing::using_depth_topic() {return !depth_topic.empty();}
inline bool BaseProcessing::using_seg_topic() {return !seg_topic.empty();}
inline bool BaseProcessing::using_flow_topic() {return !flow_topic.empty();}



// ImageRgb::ImageRgb(ros::NodeHandle& nh_, PROCESSING_INTERFACES)
//     :   BaseProcessing(nh_, INIT_INTERFACES)
// {
//     if (input_type == InputType::RGB) {
//         ROS_INFO_STREAM("Starting RGB preprocesser");
//         rgb_subscriber = image_transport.subscribe(rgb_topic, 100,  &ImageRgb::image_callback, this);
//         run_scene_flow = true;
//         run_mono_depth = true;
//         run_mask_rcnn = true;

//     }
//     else {
//         rgb_subscriber_synch = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(handler, rgb_topic, 1000);

//     }
    
// }

// void ImageRgb::start_services() {
//     mask_rcnn_interface->start_service();
//     mask_rcnn::MaskRcnnInterface::set_mask_labels(handler);
    
//     sceneflow->start_service();

//     mono_depth->start_service();
// }



// void ImageRgb::image_callback(ImageConstPtr& rgb) {

//     cv_bridge::CvImagePtr cv_ptr_rgb = cv_bridge::toCvCopy(*rgb, sensor_msgs::image_encodings::RGB8);

//     cv::Mat image_resize_rgb;
//     cv::resize(cv_ptr_rgb->image, image_resize_rgb, cv::Size(640, 480));

//     rgb_repub.publish(cv_ptr_rgb->toImageMsg());



//     cv::Mat distorted = image_resize_rgb;
//     cv::Mat undistorted;

//     cv::Mat image;
//     if (undistord_images) {
//         undistortImage(distorted, undistorted);
//         image = undistorted;
//     }
//     else {
//         image = distorted;
//     }


//     std_msgs::Header original_header = rgb->header;

//     std::vector<std::string> mask_rcnn_labels;
//     std::vector<int> mask_rcnn_label_indexs;
//     std::vector<mask_rcnn::SemanticObject> semantic_objects;

//     cv::Mat tracked_mask;
//     cv::Mat tracked_viz;

//     realtime_vdo_slam::VdoInput input_msg;
//     input_msg.rgb = *rgb;


    


//     if (is_first) {
//         previous_image = image;
//         previous_time = rgb->header.stamp;
//         is_first = false;
//         return;
//     }
//     else {
//         input_msg.header.stamp = rgb->header.stamp;
//         cv::Mat current_image = image;
//         current_time = rgb->header.stamp;


//         // //TODO: what should this be
//         // original_header.frame_id = "base_link";
//         if (run_scene_flow) {
//             scene_flow_success = sceneflow->analyse(current_image, previous_image, scene_flow_mat, scene_flow_viz);

//             if (scene_flow_success) {
//                 //#TODO: cannot fisplay until convert from scene flow to rgb               
//                 sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "32FC2", scene_flow_mat).toImageMsg();
//                 flownet_raw.publish(img_msg);
//                 input_msg.flow = *img_msg;


//                 img_msg = cv_bridge::CvImage(original_header, "rgb8", scene_flow_viz).toImageMsg();
//                 flownet_viz.publish(img_msg);

//             }
//             else {
//                 ROS_WARN_STREAM("Could not analyse scene flow images");
//             }
//         }

//         if (run_mask_rcnn) {
//             mask_rcnn_success = mask_rcnn_interface->analyse(current_image, mask_rcnn_mat, mask_rcnn_viz, semantic_objects);

//             if (mask_rcnn_success) {

//                 sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "mono8", mask_rcnn_mat).toImageMsg();
//                 maskrcnn_raw.publish(img_msg);
//                 input_msg.mask = *img_msg;
//                 input_msg.semantic_objects = semantic_objects;


//                 img_msg = cv_bridge::CvImage(original_header, "rgb8", mask_rcnn_viz).toImageMsg();
//                 maskrcnn_viz.publish(img_msg);

                
//             }
//             else {
//                 ROS_WARN_STREAM("Could not analyse mask rcnn images");
//             }
//         }

//         if (run_mono_depth) {
//             mono_depth_success = mono_depth->analyse(current_image, mono_depth_mat);
//             // mono_depth_success = midas_depth.analyse(current_image, mono_depth_mat);

//             if (mono_depth_success) {
//                 sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "mono16", mono_depth_mat).toImageMsg();
//                 monodepth_raw.publish(img_msg);
//                 input_msg.depth = *img_msg;
//             }
//             else {
//                 ROS_WARN_STREAM("Could not analyse midas depth depthimages");
//             }
//         }

//         if(scene_flow_success && mask_rcnn_success && scene_flow_success) {
//             input_msg.header = original_header;
//             vdo_input_pub.publish(input_msg);
//         }

//         previous_image = current_image;


//     }
// }


// ImageRgbDepth::ImageRgbDepth(ros::NodeHandle& nh_, PROCESSING_INTERFACES)
//     :   ImageRgb(nh_, INIT_INTERFACES)
// {
//     depth_subscriber_synch = std::make_shared<MessageFilterSubscriber>(handler, depth_topic, 1000);

//     if (input_type == InputType::RGB_DEPTH) {
//         ROS_INFO_STREAM("Starting RGB-Depth preprocesser");
//         rgb_depth_synch_ptr = std::make_shared<ApproxRgbDepthSynch>(ApproxRgbDepthSynch(50), *rgb_subscriber_synch, *depth_subscriber_synch);
//         rgb_depth_synch_ptr->registerCallback(boost::bind(&ImageRgbDepth::image_callback,this, _1, _2));
//         run_scene_flow = true;
//         run_mask_rcnn = false;

//     }



// }

// void ImageRgbDepth::start_services() {
//     mask_rcnn_interface->start_service();
//     mask_rcnn::MaskRcnnInterface::set_mask_labels(handler);
//     sceneflow->start_service();
// }

// void ImageRgbDepth::image_callback(ImageConstPtr& rgb, ImageConstPtr& depth) {
//     cv_bridge::CvImagePtr cv_ptr_rgb = cv_bridge::toCvCopy(*rgb, sensor_msgs::image_encodings::RGB8);
//     //we assume we can do this as the VDO pipeline expects the format to be in MONO16
//     cv_bridge::CvImagePtr cv_ptr_depth = convert_img_msg(depth, sensor_msgs::image_encodings::MONO16);

//     cv::Mat image_resize_rgb;
//     cv::resize(cv_ptr_rgb->image, image_resize_rgb, cv::Size(640, 480));
//     rgb_repub.publish(cv_ptr_rgb->toImageMsg());

//     //use any of the attached functions to preprocess the images
//     cv::Mat processed_depth;
//     mono_depth->preprocessor(cv_ptr_depth->image, processed_depth);


//     cv::Mat image_resize_depth;
//     cv::resize(processed_depth, image_resize_depth, cv::Size(640, 480));

//     sensor_msgs::Image processed_depth_msg;
//     utils::mat_to_image_msg(processed_depth_msg, image_resize_depth,sensor_msgs::image_encodings::MONO16);
//     monodepth_raw.publish(processed_depth_msg);




//     cv::Mat distored = image_resize_rgb;
//     cv::Mat undistorted;
    
//     cv::Mat image;
//     if (undistord_images) {
//         undistortImage(distored, undistorted);
//         image = undistorted;
//     }
//     else {
//         image = distored;
//     }

//     std_msgs::Header original_header = rgb->header;

//     std::vector<mask_rcnn::SemanticObject> semantic_objects;
//     realtime_vdo_slam::VdoInput input_msg;

//     sensor_msgs::Image resized_rgb_msg;
//     utils::mat_to_image_msg(resized_rgb_msg, distored, sensor_msgs::image_encodings::RGB8);


//     input_msg.rgb = resized_rgb_msg;
//     input_msg.depth = processed_depth_msg;

//     cv::Mat tracked_mask;
//     cv::Mat tracked_viz;


//     if (is_first) {
//         previous_image = image;
//         previous_time = rgb->header.stamp;
//         is_first = false;
//         return;
//     }
//     else {
//         input_msg.header = original_header;

//         cv::Mat current_image = image;
//         current_time = rgb->header.stamp;

//         if (run_scene_flow) {
//             scene_flow_success = sceneflow->analyse(current_image, previous_image, scene_flow_mat, scene_flow_viz);

//             if (scene_flow_success) {
//                 //#TODO: cannot fisplay until convert from scene flow to rgb               
//                 sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "32FC2", scene_flow_mat).toImageMsg();
//                 flownet_raw.publish(img_msg);
//                 input_msg.flow = *img_msg;

//                 img_msg = cv_bridge::CvImage(original_header, "rgb8", scene_flow_viz).toImageMsg();
//                 flownet_viz.publish(img_msg);
//             }
//             else {
//                 ROS_WARN_STREAM("Could not analyse scene flow images");
//             }
//         }

//         if (run_mask_rcnn) {
//             mask_rcnn_success = mask_rcnn_interface->analyse(current_image, mask_rcnn_mat, mask_rcnn_viz, semantic_objects);

//             if (mask_rcnn_success) {

//                 sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "mono8", mask_rcnn_mat).toImageMsg();
//                 maskrcnn_raw.publish(img_msg);
//                 input_msg.mask = *img_msg;
//                 input_msg.semantic_objects = semantic_objects;

//                 img_msg = cv_bridge::CvImage(original_header, "rgb8", mask_rcnn_viz).toImageMsg();
//                 maskrcnn_viz.publish(img_msg);
                
//             }
//             else {
//                 ROS_WARN_STREAM("Could not analyse mask rcnn images");
//             }
//         }


//         if(scene_flow_success && mask_rcnn_success) {
//             vdo_input_pub.publish(input_msg);
//         }

//         previous_image = current_image;

//     }
// }

// ImageRgbDepthSeg::ImageRgbDepthSeg(ros::NodeHandle& nh_, PROCESSING_INTERFACES)
//     :   ImageRgbDepth(nh_, INIT_INTERFACES)
// {
//     seg_subscriber_synch = std::make_shared<MessageFilterSubscriber>(handler, seg_topic, 100);

//     if (input_type == InputType::RGB_DEPTH_SEG) {
//         ROS_INFO_STREAM("Starting RGB-Depth-Seg preprocesser");
//         rgb_depth_seg_synch = std::make_shared<RgbDepthSegSynch>(*rgb_subscriber_synch, 
//                             *depth_subscriber_synch,
//                             *seg_subscriber_synch, 100);
//         rgb_depth_seg_synch->registerCallback(boost::bind(&ImageRgbDepthSeg::image_callback, this, _1, _2, _3));
//         run_scene_flow = true;
//     }
// }

// void ImageRgbDepthSeg::start_services() {
//     sceneflow->start_service();

// }


// //note: as for now we loose all the visualizaing tools for seg becuase it is not parsing through 
// //this pipeline and all the vis code is in python.
// void ImageRgbDepthSeg::image_callback(ImageConstPtr& rgb, ImageConstPtr& depth, ImageConstPtr& seg) {


//     cv_bridge::CvImagePtr cv_ptr_rgb = cv_bridge::toCvCopy(*rgb, sensor_msgs::image_encodings::RGB8);
//     cv_bridge::CvImagePtr cv_ptr_depth = convert_img_msg(depth, sensor_msgs::image_encodings::MONO16);
//     cv_bridge::CvImagePtr cv_ptr_seg = convert_img_msg(seg, sensor_msgs::image_encodings::MONO8);


//     //use any of the attached functions to preprocess the images
//     cv::Mat processed_depth;
//     mono_depth->preprocessor(cv_ptr_depth->image, processed_depth);

//     cv::Mat processed_seg;
//     mask_rcnn_interface->preprocessor(cv_ptr_seg->image, processed_seg);


//     cv::Mat image_resize_rgb;
//     cv::resize(cv_ptr_rgb->image, image_resize_rgb, cv::Size(640, 480));
//     rgb_repub.publish(cv_ptr_rgb->toImageMsg());



//     cv::Mat image_resize_depth;
//     cv::resize(processed_depth, image_resize_depth, cv::Size(640, 480));

//     sensor_msgs::Image processed_depth_msg;
//     utils::mat_to_image_msg(processed_depth_msg, image_resize_depth,sensor_msgs::image_encodings::MONO16);
//     monodepth_raw.publish(processed_depth_msg);


//     cv::Mat image_resize_seg;
//     cv::resize(processed_seg, image_resize_seg, cv::Size(640, 480));

//     sensor_msgs::Image processed_seg_msg;
//     utils::mat_to_image_msg(processed_seg_msg, image_resize_seg,sensor_msgs::image_encodings::MONO8);
//     maskrcnn_raw.publish(processed_seg_msg);


//     cv::Mat distored = image_resize_rgb;
//     cv::Mat undistorted;
    
//     cv::Mat image;
//     if (undistord_images) {
//         undistortImage(distored, undistorted);
//         image = undistorted;
//     }
//     else {
//         image = distored;
//     }

//     std_msgs::Header original_header = rgb->header;

//     realtime_vdo_slam::VdoInput input_msg;

//     sensor_msgs::Image resized_rgb_msg;
//     utils::mat_to_image_msg(resized_rgb_msg, distored, sensor_msgs::image_encodings::RGB8);


//     // sensor_msgs::Image resized_depth_msg;
//     // utils::mat_to_image_msg(resized_depth_msg, image_resize_depth, sensor_msgs::image_encodings::MONO16);

//     sensor_msgs::Image resized_seg_msg;
//     utils::mat_to_image_msg(resized_seg_msg, image_resize_seg, sensor_msgs::image_encodings::MONO8);

//     input_msg.rgb = resized_rgb_msg;
//     input_msg.depth = processed_depth_msg;
//     input_msg.mask = resized_seg_msg;

//     cv::Mat tracked_mask;
//     cv::Mat tracked_viz;


//     if (is_first) {
//         previous_image = image;
//         previous_time = rgb->header.stamp;
//         is_first = false;
//         return;
//     }
//     else {
//         input_msg.header = original_header;

//         cv::Mat current_image = image;
//         current_time = rgb->header.stamp;

//         if (run_scene_flow) {
//             scene_flow_success = sceneflow->analyse(current_image, previous_image, scene_flow_mat, scene_flow_viz);

//             if (scene_flow_success) {
//                 //#TODO: cannot fisplay until convert from scene flow to rgb               
//                 sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "32FC2", scene_flow_mat).toImageMsg();
//                 flownet_raw.publish(img_msg);
//                 input_msg.flow = *img_msg;

//                 img_msg = cv_bridge::CvImage(original_header, "rgb8", scene_flow_viz).toImageMsg();
//                 flownet_viz.publish(img_msg);
//             }
//             else {
//                 ROS_WARN_STREAM("Could not analyse scene flow images");
//             }
//         }

       
//         if(scene_flow_success) {
//             vdo_input_pub.publish(input_msg);
//         }

//         previous_image = current_image;

//     }
// }


// ImageAll::ImageAll(ros::NodeHandle& nh_, PROCESSING_INTERFACES)
//     :   ImageRgbDepthSeg(nh_, INIT_INTERFACES)
// {
//     ROS_INFO_STREAM("Starting all preprocesser");
//     flow_subscriber_synch = std::make_shared<MessageFilterSubscriber>(handler, flow_topic, 100);
//     all_synch = std::make_shared<AllSynch>(*rgb_subscriber_synch, 
//                             *depth_subscriber_synch,
//                             *seg_subscriber_synch, 
//                             *flow_subscriber_synch, 100);


//     all_synch->registerCallback(boost::bind(&ImageAll::image_callback, this, _1, _2, _3, _4));
// }

// void ImageAll::start_services() {}

// //TODO!
// void ImageAll::image_callback(ImageConstPtr& rgb, ImageConstPtr& depth, ImageConstPtr& seg, ImageConstPtr& flow) {
//     // cv_bridge::CvImagePtr cv_ptr_rgb = cv_bridge::toCvCopy(*rgb, sensor_msgs::image_encodings::RGB8);
//     // cv_bridge::CvImagePtr cv_ptr_depth = convert_img_msg(depth, sensor_msgs::image_encodings::MONO16);
//     // cv_bridge::CvImagePtr cv_ptr_seg = convert_img_msg(seg, sensor_msgs::image_encodings::MONO8);
//     // // cv_bridge::CvImagePtr cv_ptr_seg = convert_img_msg(seg, sensor_msgs::image_encodings::MONO8);


//     // cv::Mat image_resize_rgb;
//     // cv::resize(cv_ptr_rgb->image, image_resize_rgb, cv::Size(640, 480));
//     // rgb_repub.publish(cv_ptr_rgb->toImageMsg());

//     // cv::Mat image_resize_depth;
//     // cv::resize(cv_ptr_depth->image, image_resize_depth, cv::Size(640, 480));
//     // monodepth_raw.publish(cv_ptr_depth->toImageMsg());

//     // cv::Mat image_resize_seg;
//     // cv::resize(cv_ptr_seg->image, image_resize_seg, cv::Size(640, 480));
//     // maskrcnn_raw.publish(cv_ptr_seg->toImageMsg());


//     // cv::Mat distored = image_resize_rgb;
//     // cv::Mat undistorted;
    
//     // cv::Mat image;
//     // if (undistord_images) {
//     //     undistortImage(distored, undistorted);
//     //     image = undistorted;
//     // }
//     // else {
//     //     image = distored;
//     // }

//     // std_msgs::Header original_header = rgb->header;

//     // realtime_vdo_slam::VdoInput input_msg;

//     // sensor_msgs::Image resized_rgb_msg;
//     // utils::mat_to_image_msg(resized_rgb_msg, distored, sensor_msgs::image_encodings::RGB8);


//     // sensor_msgs::Image resized_depth_msg;
//     // utils::mat_to_image_msg(resized_depth_msg, image_resize_depth, sensor_msgs::image_encodings::MONO16);

//     // sensor_msgs::Image resized_seg_msg;
//     // utils::mat_to_image_msg(resized_seg_msg, image_resize_seg, sensor_msgs::image_encodings::MONO8);

//     // sensor_msgs::Image resized_flow_msg;
//     // utils::mat_to_image_msg(resized_seg_msg, image_resize_seg, sensor_msgs::image_encodings::MONO8);

//     // input_msg.rgb = resized_rgb_msg;
//     // input_msg.depth = resized_depth_msg;
//     // input_msg.mask = resized_seg_msg;





       
//     //     if(scene_flow_success) {
//     //         vdo_input_pub.publish(input_msg);
//     //     }

//     //     previous_image = current_image;

//     // }
// }


