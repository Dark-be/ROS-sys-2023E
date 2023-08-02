#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
static const char WINDOW[] = "Image window";

 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
 
  //Reading an image from the file
  cv::VideoCapture cap;
  cap.open("/dev/video0",cv::CAP_ANY);

  if (!cap.isOpened()) {
    ROS_INFO("Open camera failed!");
    return -1;
  }
  
  cv::Mat cv_image;
  //Show the image
  cv::namedWindow(WINDOW);
  //Convert OpenCV image to ROS message
  ros::NodeHandle node;
  image_transport::ImageTransport transport(node);
  image_transport::Publisher image_pub; 
  image_pub=transport.advertise("OutImage", 1);
  sensor_msgs::Image im;

  ros::Rate loop_rate(50);
  while(ros::ok()){
    cap.read(cv_image);
    if(cv_image.empty() )
    {
        ROS_ERROR("Read the picture failed!");
        return -1;
    }
 

    ros::Time time=ros::Time::now(); 
 
    cv_bridge::CvImage cvi;
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = cv_image;
 
    
    cvi.toImageMsg(im);
    image_pub.publish(im);
    ROS_INFO("Converted Successfully!");
 
    
    cv::imshow(WINDOW,cv_image);
    loop_rate.sleep();
  }
  cv::waitKey(0);
  cap.release(); // 释放VideoCapture对象
  cv::destroyAllWindows(); // 关闭所有打开的窗口

  return 0;
}
