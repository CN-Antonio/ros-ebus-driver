#ifndef __EBUS_NODE_H__
#define __EBUS_NODE_H__

// boost
#include <boost/thread.hpp>
// eBUS SDK
#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvString.h>
#include <PvPipeline.h>
#include <PvBuffer.h>
#include <PvBufferConverter.h>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// ROS
// #include <ros/publisher.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ebus/ebusConfig.h> // auto generated

#define SP_20000C_ID "14FB00930ECE"
#define BUFFER_COUNT ( 16 )

// THIS MACRO EXPLOITS RESULT DESCRIPTION FOR THROWING EXCEPTIONS ON EXPR WHICH RETURNS A PvResult
#define CHECK_RESULT(expression)\
do{\
    PvResult result = expression;\
    if(!result.IsOK())\
        throw std::runtime_error(result.GetDescription().GetAscii());\
}while(false)

namespace IRALab
{
class PhotonFocusCamera
{
    PvDevice * lDevice;
    PvStream * lStream;
    PvPipeline * lPipeline;
    PvString lConnectionID; // ID/IP

    PvGenParameterArray *lDeviceParams;
    PvGenParameterArray *lStreamParams;

    boost::shared_ptr<boost::thread> image_thread;
    
public:
    boost::function<void(const cv::Mat &image)> callback;

    PhotonFocusCamera();
    PhotonFocusCamera(std::string ConnectionID);
    ~PhotonFocusCamera();

    void start();
    void stop();

    template <typename ParamType,typename ValueType>
    ValueType getDeviceAttribute(std::string name, ValueType * min = NULL, ValueType * max = NULL);

    template <typename ParamType,typename ValueType>
    void setDeviceAttribute(std::string name, ValueType value);

private:
    void open();
    void close();
    void acquireImages();
    cv::Mat PvImage2CV2Image(PvBuffer *aBuffer);

    PvAccessType getAccessType();
};

class PhotonFocusDriver
{
private:
    // ROS
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport;
    image_transport::CameraPublisher publisher;

     // ROS Message
    sensor_msgs::ImagePtr image;

    // PhotonFocus Camera
    boost::scoped_ptr<IRALab::PhotonFocusCamera> camera;
    std::string camera_name;

    // ebus: ProjectName
    // TODO Dynamic Reconfigure [with parameter server]
    dynamic_reconfigure::Server<ebus::ebusConfig> reconfig_svr_;  // related to cfg file

    // Calibration Manager
    boost::shared_ptr<camera_info_manager::CameraInfoManager> calibration_manager;

public:
    PhotonFocusDriver(std::string camera_name, std::string ConnectID, const ros::NodeHandle & node_handle);
    ~PhotonFocusDriver();
    void publishImage(const cv::Mat img);
    void configCb(ebus::ebusConfig & config, uint32_t level);
};
}

///
/// Function Prototypes
///
PvDevice *ConnectToDevice( const PvString &aConnectionID );
PvStream *OpenStream( const PvString &aConnectionID );
PvPipeline* CreatePipeline( PvDevice *aDevice, PvStream *aStream );
void ConfigureStream( PvDevice *aDevice, PvStream *aStream );
bool ProcessImages();
void AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline );
void AcquireROSImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline );
cv::Mat PvImage2CV2Image(PvBuffer *aBuffer);
void TestPvBuffer(PvBuffer *aBuffer);

#endif