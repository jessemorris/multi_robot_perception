#include "RealTimeVdoSlam.hpp"


RealTimeVdoSLAM::RealTimeVdoSLAM(ros::NodeHandle& n) :
        handler(n),
        image_transport(n),
        is_first(true)
{

    handler.getParam("realtime_vdo_slam/image_topic", output_video_topic);
    image_subscriber = image_transport.subscribe(output_video_topic, 1,
                                               &RealTimeVdoSLAM::image_callback, this);

    results = image_transport.advertise("vdoslam/results", 1);
}

RealTimeVdoSLAM::~RealTimeVdoSLAM() {}

void RealTimeVdoSLAM::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    if (is_first) {
        cv::cvtColor(image, previous_image, cv::COLOR_BGR2GRAY);
        is_first = false;
        return;
    }
    else {
        cv::Mat current_image = image;
        cv::cvtColor(current_image, current_image, cv::COLOR_BGR2GRAY);

        cv::Mat flow(previous_image.size(), CV_32FC2);
        cv::calcOpticalFlowFarneback(previous_image, current_image, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        // visualization
        
        cv::Mat flow_parts[2];
        cv::split(flow, flow_parts);
        cv::Mat magnitude, angle, magn_norm;
        cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
        angle *= ((1.f / 360.f) * (180.f / 255.f));
        //build hsv image
        cv::Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        cv::merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);

        previous_image = current_image.clone();

        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr).toImageMsg();
        results.publish(img_msg);


    }



}