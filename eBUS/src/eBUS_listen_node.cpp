#include "ros/ros.h"
#include "std_msgs/String.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
// #include <opencv2/imgcodecs/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

void CallBack(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;  // declare CvImagePtr type
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img_input = cv_ptr->image;

    ROS_INFO("sub img width=%d, height=%d", img_input.cols, img_input.rows);
}

int main(int argc, char ** argv){
    ros::init(argc,argv, "eBUS_listener");
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);
    image_transport::Subscriber image_subscriber;
    image_subscriber = image_transport.subscribe("/camera/image_color_RAW", 1, &CallBack); //(接收来自topic,队列长度，回调函数)
    ros::Rate r(10);

    // while(ros::ok())
    // {
    //     sub = node_handle.subscribe("/camera/image_color_RAW", 10, CallBack); //(接收来自topic,队列长度，回调函数)

    //     ros::spinOnce(); //反复查看队列，处理、清空队列；反复调用当前可触发的回调函数
    //     r.sleep();
    // }

    ros::spin();
    
    return 0;
}