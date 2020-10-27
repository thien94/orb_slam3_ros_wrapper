/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle node_handler;
    std::string name_of_node = ros::this_node::getName();

    if ((!node_handler.hasParam(name_of_node + "/voc_file") || !node_handler.hasParam(name_of_node + "/settings_file")))
    {
        ROS_ERROR("Mono node failed to initialize");
        cerr << endl << "Please provide voc_file and settings_file in the launch file" << endl;       
        ros::shutdown();
        return 1;
    }

    std::string voc_file, settings_file;
    node_handler.param<std::string>(name_of_node + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(name_of_node + "/settings_file", settings_file, "file_not_set");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::MONOCULAR, true);
    ImageGrabber igb(&SLAM);

    ros::Subscriber sub = node_handler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}