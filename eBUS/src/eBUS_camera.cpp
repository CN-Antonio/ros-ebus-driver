// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/imgproc/types_c.h> // Camera at /dev/videoX

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
    // Device Params
    // for()
    // set Resulution
    // lDeviceParams->SetIntegerValue("Width", 5120);
    // lDeviceParams->SetIntegerValue("Height", 2880);
    // cout << "set Resolution" << endl;
    // lDeviceParams->SetIntegerValue("OffsetX", 0);
    // lDeviceParams->SetIntegerValue("OffsetY", 480);
    // cout << "set Offset" << endl;

    open();  // open stream

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

    // test
    ros::NodeHandle node_handle;  //创建句柄，实例化node
    ros::Publisher eBUS_publish = node_handle.advertise<std_msgs::String>("eBUS_topic", 10); //(发送的目标topic，消息队列长度)
    std_msgs::String msg;
    std::stringstream ss;

    while(!PvKbHit())
    {
        PvResult lResult;
        PvResult lOperationResult;

        PvBuffer *lBuffer = NULL;
        PvImage *lImage = NULL;
        cv::Mat raw_image;
        cv::Mat rgb_image;

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
                // CHECK_RESULT(lDeviceParams->GetIntegerValue("Average_Value",image_average)); // No param

                std::cout << std::fixed << std::setprecision(1);
                std::cout << lDoodle[lDoodleIndex];
                if(lBuffer->GetPayloadType() == PvPayloadTypeImage)
                {
                    lImage = lBuffer->GetImage();
                    raw_image = PvImage2CV2Image(lBuffer);
                    // raw_image = cv::Mat(lImage->GetHeight(), lImage->GetWidth(),CV_8UC1, lImage->GetDataPointer());

                    // std::cout << " W:" << std::setw(4) << std::setfill(' ') << std::left << std::dec << raw_image.cols
                    //           << " H:" << std::setw(4) << std::setfill(' ') << std::left << std::dec << raw_image.rows;

                    // cv::cvtColor(raw_image, raw_image, CV_BayerBG2BGR);
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

cv::Mat PhotonFocusCamera::PvImage2CV2Image(PvBuffer *aBuffer)
{
    PvImage *lImage = aBuffer->GetImage();
    uint32_t lHeight = lImage->GetHeight();
    uint32_t lWidth = lImage->GetWidth();
    uint32_t lPixelBytes = lImage->GetBitsPerPixel() / 8;
    uint32_t lImageSize = lImage->GetImageSize();
    uint8_t *lPixelPtr = lImage->GetDataPointer();
    PvPixelType lPixelType = lImage->GetPixelType();

    //CV
    cv::Mat img = cv::Mat::zeros(lHeight, lWidth, CV_8UC3);
    cv::Mat PvImg(lHeight, lWidth, CV_8UC1, lPixelPtr);

    switch(lPixelType)
    {
        case PvPixelBayerRG8:
            cv::cvtColor(PvImg, img, CV_BayerBG2BGR);
            break;
        default:
            break;
    }
    
    // std::cout << "width: " << img.cols << " height: " << img.rows << " channels: " << img.channels() << std::endl;

    return img;
}
}