// /*
//  * convert Bayer RAW data with eBUS SDK & OpenCV
//  */

#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "msgs/Msg.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <boost/scoped_ptr.hpp>

#include "eBUS_node.h"

PV_INIT_SIGNAL_HANDLER();

//
// Main function
//
int main(int argc, char **argv)
{
    PvDevice *lDevice = NULL;
    PvStream *lStream = NULL;

    ros::init(argc,argv, "eBUS_CAM_publisher"); //解析参数，命名当前node
    ros::NodeHandle node_handle;  //创建句柄，实例化node
    // ros::Rate delay(1);
    // ros::Publisher eBUS_publish = node_handle.advertise<std_msgs::String>("eBUS_topic", 10); //(发送的目标topic，消息队列长度)
    // ros::Publisher say_pub_new = n.advertise<msgs::Msg>("say_topic_new",10);
    // delay.sleep(); // ensure pub register successful
    // ros::Rate loop_rate(1);    //控制rate(Hz)

    cout << "PvPipeline for ROS:" << endl << endl;
    // std_msgs::String msg;
    // std::stringstream ss;
    // ss << "PvPipeline for ROS:";
    // msg.data = ss.str();
    // eBUS_publish.publish(msg);
    // ss.str("");

    // Camera name
    std::string camera_name = ros::this_node::getName();
    camera_name = std::string(camera_name.begin()+ros::this_node::getNamespace().length(),camera_name.end());

    // ConnectionID
    std::string lConnectionID = SP_20000C_ID;

    boost::shared_ptr<IRALab::PhotonFocusDriver> camera_node(new IRALab::PhotonFocusDriver(camera_name, lConnectionID, node_handle));

    while(ros::ok() /*&& !ros_shutdown*/)
        ros::spinOnce();

    // ProcessImages();
    
    // cout << endl;
    // cout << "<press a key to exit>" << endl;
    // PvWaitForKeyPress();

    // the node is shutting down...cleaning
    camera_node.reset();

    ros::shutdown();
    return 0;
}

/*/
bool ProcessImages()
{
    PvResult lResult;

    //Get the selected device information.
    PvString lConnectionID = SP_20000C_ID;
    // if ( !PvSelectDevice( &lConnectionID ) )
    // {
    //     cout << "No device selected." << endl;
    //     return false;
    // }

    // Connect to the GigE Vision or USB3 Vision device
    PvDevice *lDevice = ConnectToDevice(lConnectionID);
    if(NULL == lDevice) return false;

    // Creates stream object
    PvStream* lStream = OpenStream( lConnectionID );
    if(NULL == lStream) return false;

    // Configure streaming for GigE Vision devices
    ConfigureStream(lDevice, lStream);

    // Create the PvPipeline object
    PvPipeline *lPipeline = CreatePipeline(lDevice, lStream);
   
    // Get device parameters need to control streaming
    PvGenParameterArray *lDeviceParams = lDevice->GetParameters();
    // Get stream parameters
    PvGenParameterArray *lStreamParams = lStream->GetParameters();

    // Map the GenICam AcquisitionStart and AcquisitionStop commands
    PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
    PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );

    // Note: the pipeline must be initialized before we start acquisition
    cout << "Starting pipeline" << endl;
    lPipeline->Start();

    // Get stream parameters/stats.
    PvGenInteger *lBlockCount = dynamic_cast<PvGenInteger *>( lStreamParams->Get( "BlockCount" ) );
    PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRate" ) );
    PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "Bandwidth" ) );
    
    // Enables stream before sending the AcquisitionStart command.
    cout << "Enable streaming on the controller." << endl;
    lDevice->StreamEnable();

    // The buffers are queued in the stream, we just have to tell the device
    // to start sending us images.
    cout << "Sending StartAcquisition command to device" << endl;
    lDeviceParams->ExecuteCommand( "AcquisitionStart" );

    char lDoodle[] = "|\\-|-/";
    int lDoodleIndex = 0;
    int64_t lBlockCountVal = 0;
    double lFrameRateVal = 0.0;
    double lBandwidthVal = 0.0;

    cout << endl;
    // Acquire images until the user instructs us to stop.
    cout << "<press a key to stop streaming>" << endl;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_color_RAW", 1);

    while ( !PvKbHit() )
    {
        PvBuffer *lBuffer = NULL;
        PvResult lOperationResult;
        // PvBufferConverter lBufferConverter;
        // SimpleImagingLib::ImagingContrastFilter lContrastFilter;

        // Retrieve next buffer
        // lResult = lStream->RetrieveBuffer( &lBuffer, &lOperationResult, 1000 );
        lResult = lPipeline->RetrieveNextBuffer( &lBuffer, 1000, &lOperationResult );
        if ( lResult.IsOK() )
        {
            if (lOperationResult.IsOK())
            {
                // We now have a valid buffer. This is where you would typically process the buffer.

                lBlockCount->GetValue( lBlockCountVal );
                lFrameRate->GetValue( lFrameRateVal );
                lBandwidth->GetValue( lBandwidthVal );

                // Retrieve the imaging buffer based on the buffer's custom ID
                // SimpleImagingLib::ImagingBuffer *lImagingBuffer = gImagingBuffers + lBuffer->GetID();

                // Retrieve our image based on buffer ID - which has been set to the index of the array
                // lContrastFilter.Apply( lImagingBuffer );

                // uint32_t lHeight = lImagingBuffer->GetHeight();
                // uint32_t lWidth = lImagingBuffer->GetWidth();

                cout << fixed << setprecision( 1 );
                cout << lDoodle[ lDoodleIndex ];
                cout << " BlockID: " << uppercase << hex << setfill('0') << setw(16) << lBuffer->GetBlockID() 
                     << " W: " << dec << lBuffer->GetImage()->GetWidth() 
                     << " H: " << lBuffer->GetImage()->GetHeight() 
                     << " " << lFrameRateVal << " FPS " << ( lBandwidthVal / 1000000.0 ) << " Mb/s  \r";

                // publish ROS message
                cv::Mat image = PvImage2CV2Image(lBuffer);
                cv::resize(image, image, cv::Size(image.cols/4, image.rows/4));

                // display
                // cv::imshow("colorview", image);

                sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                pub.publish(imgmsg);
            }

            // Release the buffer back to the pipeline
            lPipeline->ReleaseBuffer( lBuffer );
        }
        else
        {
            // Timeout
            cout << lDoodle[ lDoodleIndex ] << " Timeout\r";
        }

        ++lDoodleIndex %= 6;
    }

    PvGetChar(); // Flush key buffer for next stop.
    cout << endl << endl;

    // Tell the device to stop sending images.
    cout << "Sending AcquisitionStop command to the device" << endl;
    // lStop->Execute();
    lDeviceParams->ExecuteCommand( "AcquisitionStop" );

    // Disable stream after sending the AcquisitionStop command.
    cout << "Disable streaming on the controller." << endl;
    lDevice->StreamDisable();

    // Stop the pipeline
    cout << "Stop pipeline" << endl;
    lPipeline->Stop();

    // ReleaseBuffers();

    // Now close the stream. Also optional but nice to have.
    cout << "Closing stream" << endl;
    lStream->Close();

    // Disconnect the device. Optional, still nice to have.
    cout << "Disconnecting device" << endl;
    lDevice->Disconnect();

    // Free the objects allocated by PvDevice and PvStream factory methods
    PvStream::Free( lStream );
    PvDevice::Free( lDevice );

    return true;
}//*/

cv::Mat PvImage2CV2Image(PvBuffer *aBuffer)
{
    PvImage *lImage = aBuffer->GetImage();
    uint32_t lHeight = lImage->GetHeight();
    uint32_t lWidth = lImage->GetWidth();
    uint32_t lPixelBytes = lImage->GetBitsPerPixel() / 8;
    uint32_t lImageSize = lImage->GetImageSize();
    uint8_t *lPixelPtr = lImage->GetDataPointer();
    PvPixelType lPixelType = lImage->GetPixelType();

    //CV
    cv::Mat RGBimg = cv::Mat::zeros(lHeight, lWidth, CV_8UC3);
    cv::Mat bayerImg(lHeight, lWidth, CV_8UC1, lPixelPtr);

    switch(lPixelType)
    {
        case PvPixelBayerRG8:
            cv::cvtColor(bayerImg, RGBimg, CV_BayerBG2BGR);
            break;
        default:
            break;
    }

    // cv::resize(RGBimg, RGBimg, cv::Size(lWidth/2, lHeight/2));
    
    // std::cout << "width: " << img.cols << " height: " << img.rows << " channels: " << img.channels() << std::endl;

    return RGBimg;
}