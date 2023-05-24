// Message
#include <cv_bridge/cv_bridge.h>
// boost
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include "eBUS_node.h"

namespace IRALab
{
PhotonFocusDriver::PhotonFocusDriver(std::string camera_name, std::string ConnectID, const ros::NodeHandle & node_handle):
    node_handle(node_handle),
    image_transport(node_handle),
    camera_name(camera_name),
    calibration_manager(new camera_info_manager::CameraInfoManager(node_handle, camera_name))
{
    publisher = image_transport.advertiseCamera("/camera/image_color_RAW", 1);

    camera.reset(new IRALab::PhotonFocusCamera(ConnectID));
    camera->start();
    camera->callback = boost::bind(&PhotonFocusDriver::publishImage, this, _1);
    // TODO parameter server callback
    // reconfig_svr_.setCallback(boost::bind(&PhotonFocusDriver::configCb, this, _1, _2));

    std::cout << std::setw(80) << std::setfill(' ') << std::left << "===== PhotonFocus Camera ----- START ===== " << std::endl;
}

PhotonFocusDriver::~PhotonFocusDriver()
{
    camera->stop();
    camera.reset();
    std::cout << "===== PhotonFocus Camera ----- STOP  ===== " << std::endl;
}

void PhotonFocusDriver::publishImage(const cv::Mat img)
{
    cv_bridge::CvImage cv_image;
    cv_image.encoding = "bgr8";
    // cv_image.encoding = "bayer_gbrg8";
    cv_image.image = img;
    cv_image.header.stamp = ros::Time::now();
    image = cv_image.toImageMsg();

    sensor_msgs::CameraInfo::Ptr camera_info;
    if(calibration_manager->isCalibrated()) // calibration exists
        camera_info.reset(new sensor_msgs::CameraInfo(calibration_manager->getCameraInfo()));
    else // calibration doesn't exist
    {
        camera_info.reset(new sensor_msgs::CameraInfo());
        camera_info->width = image->width;
        camera_info->height = image->height;
    }

    // WARNING for calibration with cameracalibrator for ROS replace "stereo_rig" with ("/" + camera_name) in both next 2 lines
    image->header.frame_id = "stereo_rig"; 
    camera_info->header.frame_id = "stereo_rig";
    camera_info->header.stamp = cv_image.header.stamp;

    publisher.publish(image, camera_info);
}
}
