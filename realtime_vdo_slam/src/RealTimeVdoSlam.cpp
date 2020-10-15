#include "RealTimeVdoSlam.hpp"


RealTimeVdoSLAM::RealTimeVdoSLAM(ros::NodeHandle& n) :
        handler(n),
        sceneflow(n),
        image_transport(n),
        is_first(true)
{

    handler.getParam("realtime_vdo_slam/image_topic", output_video_topic);
    image_subscriber = image_transport.subscribe(output_video_topic, 10,
                                               &RealTimeVdoSLAM::image_callback, this);

    results = image_transport.advertise("vdoslam/results", 1);
}

RealTimeVdoSLAM::~RealTimeVdoSLAM() {}

void RealTimeVdoSLAM::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    cv::Mat flow_matrix;

    if (is_first) {
        previous_image = image;
        is_first = false;
        return;
    }
    else {
        cv::Mat current_image = image;
        bool result = sceneflow.analyse_image(current_image, previous_image, flow_matrix);

        if (result) {
            std_msgs::Header header = std_msgs::Header();

            //TODO: proper headers
            header.frame_id = "base_link";
            header.stamp = ros::Time::now();
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", flow_matrix).toImageMsg();
            results.publish(img_msg);
        }
        else {
            ROS_WARN_STREAM("Could not analyse scene flow images");
        }

        previous_image = current_image.clone();


    }



}