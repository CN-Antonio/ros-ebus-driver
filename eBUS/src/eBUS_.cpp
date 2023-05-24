// ROS
#include <ros/ros.h>
// #include <ros/publisher.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>

// Message
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// #include <image_transport/camera_publisher.h>

// boost
#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include "eBUS_node.h"

namespace IRALab
{
// this was imported from driver_common, since the latter seems deprecated.
enum{
  RECONFIGURE_CLOSE = 3,
  RECONFIGURE_STOP = 1,
  RECONFIGURE_RUNNING = 0,
};
PhotonFocusCamera::PhotonFocusCamera()
{
    std::cout << "Camera On" <<std::endl;
}

PhotonFocusCamera::PhotonFocusCamera(std::string ConnectionID)
    :lConnectionID(ConnectionID.c_str())
{    
    PvResult lResult;

    // Connect to the GigE Vision or USB3 Vision device
    cout << "1. Connecting to device." << endl;
    lDevice = PvDevice::CreateAndConnect(lConnectionID, &lResult);
    if ( (lDevice == NULL ) || (!lResult.IsOK()) )
    {
        throw std::runtime_error(std::string("Unable to connect device ") + lConnectionID.GetAscii() + ".");
        cout << "Unable to connect to device: "
        << lResult.GetCodeString().GetAscii()
        << " ("
        << lResult.GetDescription().GetAscii()
        << ")" << endl;
    }

    std::cout << "   Camera On: " << lConnectionID.GetAscii() << std::endl;

    lDeviceParams = lDevice->GetParameters();
}

PhotonFocusCamera::~PhotonFocusCamera()
{
    cout << "1. Disconnecting from device." << endl;
    CHECK_RESULT(lDevice->Disconnect());
    PvDevice::Free(lDevice);
    std::cout << "Camera Off" <<std::endl;
}

void PhotonFocusCamera::start()
{
    open();

    // All is set and ready, now say to the camera to start sending images
    // The buffers are queued in the stream, we just have to tell the device
    // to start sending us images.
    cout << "   Sending StartAcquisition command to device" << endl;
    lDeviceParams->ExecuteCommand("AcquisitionStart");

    // Start the thread which polls images from the camera buffer
    image_thread.reset(new boost::thread(boost::bind(&IRALab::PhotonFocusCamera::acquireImages, this)));
}

void PhotonFocusCamera::stop()
{
    // Tell the camera to stop sending images
    cout << "   Sending StartAcquisition command to device" << endl;
    lDeviceParams->ExecuteCommand( "AcquisitionStop" );
    close();
}

void PhotonFocusCamera::open()
{
    PvResult lResult;

    // Open stream to the GigE Vision or USB3 Vision device
    cout << "2. opening stream." << endl;
    lStream = PvStream::CreateAndOpen(lConnectionID, &lResult);
    if ( ( lStream == NULL ) || !lResult.IsOK() )
    {
        throw std::runtime_error(std::string("Unable to stream from ") + lConnectionID.GetAscii() + ".");
        cout << "Unable to stream from device. "
        << lResult.GetCodeString().GetAscii()
        << " ("
        << lResult.GetDescription().GetAscii()
        << ")"
        << endl;
    }

    std::cout << "   Stream On: " << std::endl;

    lStreamParams = lStream->GetParameters(); // get stream parameters (for future usages)

    // Pipeline initialization (it manages buffers)
    lPipeline = new PvPipeline(lStream);
    if ( lPipeline != NULL )
    {
        // Reading payload size from device
        uint32_t lSize = lDevice->GetPayloadSize();

        // Set the Buffer count and the Buffer size
        lPipeline->SetBufferCount( BUFFER_COUNT );
        lPipeline->SetBufferSize( lSize );
    }

    // Note: the pipeline must be initialized before we start acquisition
    cout << "3. Starting pipeline" << endl;
    lPipeline->Start();

    // Enables stream before sending the AcquisitionStart command.
    cout << "4. Enable streaming on the controller." << endl;
    lDevice -> StreamEnable();
}

void PhotonFocusCamera::close()
{
    // STOP THE THREAD WHICH RETRIEVE IMAGES FROM THE CAMERA AND WAIT FOR ITS CONCLUSION!
    image_thread->interrupt();
    image_thread->join(); // TODO is it necessary?
    image_thread.reset();

    cout << "4. Disable streaming on the controller." << endl;
    lDevice->StreamDisable();

    cout << "3. Stopping pipeline" << endl;
    lPipeline->Stop();
    delete lPipeline;

    cout << "2. Closing stream." << endl;
    lStream->Close();
    PvStream::Free(lStream);
}

void PhotonFocusCamera::acquireImages()
{
    char lDoodle[] = "|\\-|-/";
    int lDoodleIndex = 0;

    double framerate = 0.0;
    double bandwidth = 0.0;
    long error_count = 0;
    long image_average = 0;
    PvString last_error;

    cout << "<press a key to stop streaming>" << endl;

    // TODO Delete
    // test begin
    ros::NodeHandle node_handle;  //创建句柄，实例化node
    ros::Publisher eBUS_publish = node_handle.advertise<std_msgs::String>("eBUS_topic", 10); //(发送的目标topic，消息队列长度)
    std_msgs::String msg;
    std::stringstream ss;
    // test end

    while(!PvKbHit())
    {
        PvResult lResult;
        PvResult lOperationResult;

        PvBuffer *lBuffer = NULL;
        PvImage *image = NULL;
        cv::Mat raw_image;

        // Retrieve next buffer
        lResult = lPipeline->RetrieveNextBuffer(&lBuffer, 1000, &lOperationResult);
        if(lResult.IsOK()) // operation results says about the retrieving from the pipeline
        {
            if(lOperationResult.IsOK()) // buffer results says about the retrieved buffer status
            {
                CHECK_RESULT(lStreamParams->GetFloatValue("AcquisitionRate", framerate));
                CHECK_RESULT(lStreamParams->GetFloatValue("Bandwidth", bandwidth));
                CHECK_RESULT(lStreamParams->GetIntegerValue("ErrorCount",error_count));
                CHECK_RESULT(lStreamParams->GetEnumValue("LastError",last_error));
                // CHECK_RESULT(device_parameters->GetIntegerValue("Average_Value",image_average));

                std::cout << std::fixed << std::setprecision(1);
                std::cout << lDoodle[lDoodleIndex];
                if(lBuffer->GetPayloadType() == PvPayloadTypeImage)
                {
                    image = lBuffer->GetImage();
                    raw_image = cv::Mat(image->GetHeight(),image->GetWidth(),CV_8UC1,image->GetDataPointer());

                    // std::cout << " W:" << std::setw(4) << std::setfill(' ') << std::left << std::dec << raw_image.cols
                    //           << " H:" << std::setw(4) << std::setfill(' ') << std::left << std::dec << raw_image.rows;

                    // compress image
                    cv::cvtColor(raw_image, raw_image, CV_BayerBG2BGR);
                    std::vector<uchar> data_encode;
                    std::vector<int> quality;
                    quality.push_back(cv::IMWRITE_JPEG_QUALITY);
                    quality.push_back(50);  //进行50%的压缩
                    cv::resize(raw_image, raw_image, cv::Size(raw_image.cols/8, raw_image.rows/8));
                    cv::imencode(".jpg", raw_image, data_encode,quality);//将图像编码

                    std::cout << " W:" << std::setw(4) << std::setfill(' ') << std::left << std::dec << raw_image.cols
                              << " H:" << std::setw(4) << std::setfill(' ') << std::left << std::dec << raw_image.rows;

                    // !!!! THIS IS THE POINT WHERE THE EXTERNAL CALLBACK IS CALLED !!!!
                    callback(raw_image);
                }
                else
                    std::cout << " (buffer does not contain image)";

                std::cout << " "
                          << std::setw(3) << std::setfill(' ') << std::fixed << std::setprecision(1) << framerate << "FPS "
                          << std::setw(3) << (int)(bandwidth / 1000000.0) << "Mb/s"
                          << " AvgVal: " << std::setw(4) << std::setfill(' ') << std::right << image_average
                          << " * Errors:" << error_count << " - " << std::setw(26) << std::setfill(' ') << std::left << last_error.GetAscii() << "\r";
            }
            else{
                std::cout << lDoodle[lDoodleIndex] << " " << lOperationResult.GetCode() << " " << lOperationResult.GetDescription().GetAscii() << "\r";
                ss << lDoodle[lDoodleIndex] << " " << lOperationResult.GetCode() << " " << lOperationResult.GetDescription().GetAscii() << "\r";
                msg.data = ss.str();
                eBUS_publish.publish(msg);
            }
            // release the buffer back to the pipeline
            lPipeline->ReleaseBuffer(lBuffer);
        }
        else
        {
            // Timeout
            cout << lDoodle[ lDoodleIndex ] << " Timeout\r";
            ss << lDoodle[ lDoodleIndex ] << " Timeout\r";
            msg.data = ss.str();
            eBUS_publish.publish(msg);
        }
        // when the interruption on the thread is called, its execution must reach this point! in this way the whole should be in a clear state
        boost::this_thread::interruption_point();
        ++lDoodleIndex %= 6;
    }
}
/*/
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

    publisher.publish(image,camera_info);
}//*/

// void PhotonFocusDriver::configCb(ebus::ebusConfig & config, uint32_t level)
// {
//     if(level >= (uint32_t) IRALab::RECONFIGURE_STOP)
//         camera->stop();

//     camera->setDeviceAttribute<PvGenEnum,std::string>("PixelFormat","Mono8");

//     //# ----- Image Size Control -----
//     camera->setDeviceAttribute<PvGenInteger,long>("Width",config.Width*32+768);
//     camera->setDeviceAttribute<PvGenInteger,long>("Height",config.Height);

//     camera->setDeviceAttribute<PvGenInteger,long>("OffsetX",config.OffsetX*32);
//     camera->setDeviceAttribute<PvGenInteger,long>("OffsetY",config.OffsetY);

//     //# ----- Exposure and FrameRate -----
//     camera->setDeviceAttribute<PvGenFloat,double>("ExposureTimeAbs",config.ExposureTimeAbs);
//     camera->setDeviceAttribute<PvGenBoolean,bool>("ConstantFramerate_CFR",config.ConstantFramerate_CFR);
//     if(config.ConstantFramerate_CFR)
//         camera->setDeviceAttribute<PvGenFloat,double>("Frametime",config.Frametime);

//     camera->setDeviceAttribute<PvGenBoolean,bool>("Trigger_Interleave",config.Trigger_Interleave);
//     if(!config.Trigger_Interleave)\
//     {
//         camera->setDeviceAttribute<PvGenEnum,long>("LinLog_Mode",config.LinLog_Mode);
//         if(config.LinLog_Mode == 4)
//         {
//             std::cout << "UserDefined" << std::endl;
//             camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Value1",config.LinLog_Value1);
//             camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Value2",config.LinLog_Value2);
//             camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Time1",config.LinLog_Time1);
//             camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Time2",config.LinLog_Time2);
//         }
//     }
//     camera->setDeviceAttribute<PvGenInteger,long>("Voltages_BlackLevelOffset",config.Voltages_BlackLevelOffset);

//     if(level >= (uint32_t) IRALab::RECONFIGURE_STOP)
//         camera->start();
// }

}

// eBUS SDK buffers
PvBuffer* gPvBuffers;
// 3rd part

bool SelectDevice(PvString *lConnectionID)
{
    if ( !PvSelectDevice( lConnectionID ) )
    {
        cout << "No device selected." << endl;
        return false;
    }
}

PvDevice *ConnectToDevice( const PvString &aConnectionID )
{
    PvDevice *lDevice;
    PvResult lResult;

    // Connect to the GigE Vision or USB3 Vision device
    cout << "Connecting to device." << endl;
    lDevice = PvDevice::CreateAndConnect( aConnectionID, &lResult );
    if ( (lDevice == NULL ) || (!lResult.IsOK()) )
    {
        cout << "Unable to connect to device: "
        << lResult.GetCodeString().GetAscii()
        << " ("
        << lResult.GetDescription().GetAscii()
        << ")" << endl;
    }

    return lDevice;
}

PvStream *OpenStream( const PvString &aConnectionID )
{
    PvStream *lStream;
    PvResult lResult;

    // Open stream to the GigE Vision or USB3 Vision device
    cout << "Opening stream from device." << endl;
    lStream = PvStream::CreateAndOpen( aConnectionID, &lResult );
    if ( ( lStream == NULL ) || !lResult.IsOK() )
    {
        cout << "Unable to stream from device. "
            << lResult.GetCodeString().GetAscii()
            << " ("
            << lResult.GetDescription().GetAscii()
            << ")"
            << endl;
    }

    return lStream;
}

void ConfigureStream( PvDevice *aDevice, PvStream *aStream )
{
    // If this is a GigE Vision device, configure GigE Vision specific streaming parameters
    PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV *>( aDevice );
    if ( lDeviceGEV != NULL )
    {
        PvStreamGEV *lStreamGEV = static_cast<PvStreamGEV *>( aStream );

        // Negotiate packet size
        lDeviceGEV->NegotiatePacketSize();

        // Configure device streaming destination
        lDeviceGEV->SetStreamDestination( lStreamGEV->GetLocalIPAddress(), lStreamGEV->GetLocalPort() );
    }
}

PvPipeline *CreatePipeline( PvDevice *aDevice, PvStream *aStream )
{
    // Create the PvPipeline object
    PvPipeline* lPipeline = new PvPipeline( aStream );

    if ( lPipeline != NULL )
    {        
        // Reading payload size from device
        uint32_t lSize = aDevice->GetPayloadSize();
    
        // Set the Buffer count and the Buffer size
        lPipeline->SetBufferCount( BUFFER_COUNT );
        lPipeline->SetBufferSize( lSize );
    }
    
    return lPipeline;
}

//
// 1. Allocates native 3rd party library buffers.
// 2. Allocates eBUS SDK buffers attached to the 3rd party library buffers.
// 3. Queues the eBUS SDK buffers in the PvStream object.
//
void CreateBuffers( PvDevice* aDevice, PvStream* aStream ) 
{
    gPvBuffers = NULL;
    PvGenParameterArray *lDeviceParams = aDevice->GetParameters();

    // Set device in RGB8 to match what our imaging library expects
    lDeviceParams->SetEnumValue( "PixelFormat", PvPixelRGB8 );

    // Get width, height from device
    int64_t lWidth = 0, lHeight = 0;
    lDeviceParams->GetIntegerValue( "Width", lWidth );
    lDeviceParams->GetIntegerValue( "Height", lHeight );

    // // Use min of BUFFER_COUNT and how many buffers can be queued in PvStream.
    // gBufferCount = ( aStream->GetQueuedBufferMaximum() < BUFFER_COUNT ) ? 
    //     aStream->GetQueuedBufferMaximum() : 
    //     BUFFER_COUNT;
    
    // // Create our image buffers which are holding the real memory buffers
    // gImagingBuffers = new SimpleImagingLib::ImagingBuffer[ gBufferCount ];
    // for ( uint32_t i = 0; i < gBufferCount; i++ )
    // {
    //     gImagingBuffers[ i ].AllocateImage( static_cast<uint32_t>( lWidth ), static_cast<uint32_t>( lHeight ), 3 );
    // }

    // Creates, eBUS SDK buffers, attach out image buffer memory
    // gPvBuffers = new PvBuffer[ gBufferCount ];
    // for ( uint32_t i = 0; i < gBufferCount; i++ )
    // {
    //     // Attach the memory of our imaging buffer to a PvBuffer. The PvBuffer is used as a shell
    //     // that allows directly acquiring image data into the memory owned by our imaging buffer
    //     gPvBuffers[ i ].GetImage()->Attach( gImagingBuffers[ i ].GetTopPtr(), 
    //         static_cast<uint32_t>( lWidth ), static_cast<uint32_t>( lHeight ), PvPixelRGB8 );

    //     // Set eBUS SDK buffer ID to the buffer/image index
    //     gPvBuffers[ i ].SetID( i );
    // }

    // Queue all buffers in the stream
    // for ( uint32_t i = 0; i < gBufferCount; i++ )
    // {
    //     aStream->QueueBuffer( gPvBuffers + i );
    // }
}