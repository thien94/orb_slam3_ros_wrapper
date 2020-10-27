/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/core/core.hpp>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "include/System.h"
#include "include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle node_handler;
    std::string name_of_node = ros::this_node::getName();

    if (!node_handler.hasParam(name_of_node + "/voc_file") || !node_handler.hasParam(name_of_node + "/settings_file"))
    {
        ROS_ERROR("Mono-inertial node failed to initialize");
        cerr << endl << "Please provide voc_file and settings_file in the launch file. do_equalize is optional." << endl; 
        ros::shutdown();
        return 1;
    }

    bool bEqual = false;
    std::string voc_file, settings_file;
    node_handler.param<bool>(name_of_node + "/do_equalize", bEqual, false);
    node_handler.param<std::string>(name_of_node + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(name_of_node + "/settings_file", settings_file, "file_not_set");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR, true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb, bEqual);

    // Maximum delay, 5 seconds * 200Hz = 1000 samples
    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img0 = node_handler.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage,&igb);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    while(1)
    {
        cv::Mat im;
        double tIm = 0;
        if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
        {
            tIm = img0Buf.front()->header.stamp.toSec();
            if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;
            
            {
            this->mBufMutex.lock();
            im = GetImage(img0Buf.front());
            img0Buf.pop();
            this->mBufMutex.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            if(mbClahe)
                mClahe->apply(im,im);

            mpSLAM->TrackMonocular(im,tIm,vImuMeas);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}