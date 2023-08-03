#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>  // For file operations
#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>
#include "CataCamera.h"
#include "PinholeCamera.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>

using namespace cv;

/*
 * set global parameters
*/
bool redFlag = true;
bool pubRedFlag = false;
bool greenFlag = false;
bool pubGreenFlag = false;
bool recFlag = false;
bool pubRecFlag = false;
bool trackFlag = false;
bool standardizeFlag = false;

double camIntrinsics[4];
double camDist[4];
cv::Point2f redPointLocation, greenPointLocation;
std::vector<cv::Point2f> srcPoints ={{68.5,12.2},{528.7,9.6},{538.8,465.0},{69.9,471.4}};
float dstWidth = 500;
float dstHeight = 500;
std::vector<cv::Point2f> dstPoints = {{0,0}, {dstWidth,0}, {dstWidth,dstHeight}, {0,dstHeight} };
std_msgs::Float32MultiArray pubPoint;
camodocal::PinholeCamera cam;

//********************the declaration of functions***********************
bool getRedPoint(cv::Mat input);
bool getGreenPoint(cv::Mat input);
bool getRec(cv::Mat input);
double calculatePixelDistance(const cv::Point2f& point1, const cv::Point2f& point2);
std::vector<cv::Point2f> sortPoints(cv::Point2f points[4]);
Eigen::Vector3d getRealLocation(cv::Point2f targetPoint);
bool compareContourAreas(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2);
void getState(const std_msgs::Int8 stateFlag);


int main(int argc, char** argv) 
{
    //------------------------ros initialize-----------------------------
    ros::init(argc, argv, "image_proc"); 
    ros::NodeHandle nh;
    ros::Subscriber stateSub;
    ros::Publisher pointPub;
    stateSub = nh.subscribe("img/write",10,&getState);
    pointPub = nh.advertise<std_msgs::Float32MultiArray>("img/read",10);

    //---------------------read parameters from yaml--------------------

    YAML::Node yaml_node = YAML::LoadFile("/home/jetson/workspace/trackingsys/src/image_proc/config/config.yaml");
    for (int i = 0; i < 4; i++)
    {
        camIntrinsics[i] = yaml_node["Cam"]["intrinsics"][i].as<double>();
        camDist[i] = yaml_node["Cam"]["distortion"][i].as<double>();
    }
    //-----------------------camera initialize---------------------------
    VideoCapture cap(0); 
    cap.set(CAP_PROP_FRAME_WIDTH,640);
    cap.set(CAP_PROP_FRAME_HEIGHT,480);//ajust resoluction
    cap.set(cv::CAP_PROP_FPS, 30);
    // cap.set(cv::CAP_PROP_EXPOSURE,10);
    // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cv::Mat input, imageROI;
    cam = camodocal::PinholeCamera("pinholeCam", 640, 480, camDist[0], camDist[1], camDist[2],
                                camDist[3], camIntrinsics[0], camIntrinsics[1], camIntrinsics[2], camIntrinsics[3]);

    //---------------------------Undistortion---------------------------
    const cv::Mat K = (cv::Mat_<double>(3, 3) << camIntrinsics[0], 0, camIntrinsics[2],
                       0, camIntrinsics[1], camIntrinsics[3],
                       0, 0, 1);
    const cv::Mat D = (cv::Mat_<double>(4, 1) << camDist[0], camDist[1],
                       camDist[2], camDist[3]);
    cv::Size imageSize(640, 480);
    const double alpha = 1; 
    cv::Mat  map1, map2;
    Eigen::Matrix3f factorK;
    //-------------------------set parameters---------------------------
              
    while(ros::ok()) 
    {
        //-----------------------------------------ros callback-----------------------------------------------------
        ros::spinOnce();
        cap.read(input);
        //-------------------------------------apply undistortion to image------------------------------------------
        initUndistortRectifyMap(K, D, cv::Mat_<double>::eye(3, 3), K.clone(), imageSize, CV_16SC2, map1, map2);
        factorK << camIntrinsics[0], 0, camIntrinsics[2],
               0, camIntrinsics[1], camIntrinsics[3],
               0, 0, 1;
        cv::remap(input, input, map1, map2, cv::INTER_LINEAR);
        cv::Size s = input.size();
        cv::imshow("undistortion", input);
        //-------------------------------standardization finished, start to set roi---------------------------------
        if(standardizeFlag)
        {
            cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(srcPoints,dstPoints);
            cv::warpPerspective(input, imageROI, perspectiveMatrix, Point(dstWidth,dstHeight));
        }
        //---------------------------------------find red/green laser in image--------------------------------------
        if(redFlag)
        {
            if(standardizeFlag)
            {
                if(getRedPoint(imageROI) && pubRedFlag)
                {
                    pointPub.publish(pubPoint);
                }
            }
            else
            {
                if(getRedPoint(input) && pubRedFlag)
                {
                    pointPub.publish(pubPoint);
                }
            }   
        }
        if(greenFlag)
        {
            if(standardizeFlag)
            {
                if(getGreenPoint(imageROI) && pubGreenFlag)
                {
                    pointPub.publish(pubPoint);
                }
            }
            else
            {
                if(getGreenPoint(input) && pubGreenFlag)
                {
                    pointPub.publish(pubPoint);
                }
            }   
        }
        //----------------------------------find black rectangular in image-----------------------------------------
        if(recFlag)
        {
            if(standardizeFlag)
            {
                if(getRec(imageROI) && pubRecFlag)
                {
                    pointPub.publish(pubPoint);
                }
            }
            else
            {
                if(getRec(input) && pubRecFlag)
                {
                    pointPub.publish(pubPoint);
                }
            }
        }
        //--------------------------------make green laser track red laser------------------------------------------
        if(trackFlag)
        {
            if(standardizeFlag)
            {
                if(getRedPoint(imageROI) && getGreenPoint(imageROI)) 
                {
                    pointPub.publish(pubPoint);
                }
            }
            else
            {
                if(getRedPoint(input) && getGreenPoint(input))
                {
                    pointPub.publish(pubPoint);
                }
            }
        }
    
        //--------------------------------ros publish topic-------------------------------------------
        
        pubPoint.data.clear();
        
        cv::waitKey(1);
    }
    return 0;
}

/* 
 * image process function
 * compare area of contours
*/
bool compareContourAreas(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2) 
{
    return cv::contourArea(contour1, false) < cv::contourArea(contour2, false);
}

/* 
 * image process function
 * get real Point's location
*/
Eigen::Vector3d getRealLocation(cv::Point2f targetPoint)
{
    Eigen::Vector2d srcPoint, originSrc;
    Eigen::Vector3d dstPoint, originDst;
    srcPoint << targetPoint.x, targetPoint.y;
    originSrc << static_cast<double>((srcPoints[0].x+srcPoints[2].x)/2), static_cast<double>((srcPoints[0].y+srcPoints[2].y)/2);
    std::cout << "originSrc : " << originSrc[0] << " , " << originSrc[1] << std::endl;
    cam.liftProjective(srcPoint, dstPoint);
    cam.liftProjective(originSrc, originDst);
    std::cout << "originDst : " << originDst[0] << " , " << originDst[1] << std::endl;
    dstPoint[0] = 1.1 * (dstPoint[0] - originDst[0]);
    dstPoint[1] = 1.1 * (dstPoint[1] - originDst[1]);
    return dstPoint;
}

/* 
 * image process function
 * get red Point from input
*/
bool getRedPoint(cv::Mat input)
{
    // cv::Scalar lowerRed = cv::Scalar(160, 100, 200);
    // cv::Scalar upperRed = cv::Scalar(180, 255, 255);
    // cv::Mat redMask, hsvImg;
    // cv::cvtColor(input, hsvImg, cv::COLOR_BGR2HSV);
    // cv::inRange(hsvImg, lowerRed, upperRed, redMask);
    // cv::Mat redDiffMask;
    // cv::inRange(input, cv::Scalar(0, 0, 50), cv::Scalar(255, 255, 255), redDiffMask);
    cv::Mat bgr[3];
    cv::split(input, bgr);

    cv::Mat redMask = (bgr[2] - bgr[1]) > 50;  // B - G
    cv::imshow("redMask", redMask);

    std::vector<std::vector<cv::Point>> redContours;
    cv::findContours(redMask, redContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if(redContours.size())
    {
        std::vector<cv::Point> maxRedContours = *std::max_element(redContours.begin(), redContours.end(),
        [](const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2)
        {
            return cv::contourArea(contour1, false) < cv::contourArea(contour2, false);
        });

        cv::Moments redM = cv::moments(maxRedContours);
        redPointLocation.x = redM.m10 / redM.m00;
        redPointLocation.y = redM.m01 / redM.m00;

        if(pubRedFlag)
        {
            // Eigen::Vector3d redTargetPoint = getRealLocation(redPointLocation);
            redPointLocation.x = (redPointLocation.x - dstWidth/2) / 10;
            redPointLocation.y = (redPointLocation.y - dstHeight/2) / 10;
            pubPoint.data.push_back(redPointLocation.x);
            pubPoint.data.push_back(redPointLocation.y);
            // std::cout << "redRealPoint : " << redTargetPoint[0] << " , " << redTargetPoint[1] << std::endl;
        }

        std::cout << "redPoint : " << redPointLocation.x << " , " << redPointLocation.y << std::endl;
        return true;
    }

    return false;
}
/* 
 * image process function
 * get green Point from input
*/
bool getGreenPoint(cv::Mat input)
{
    cv::Mat bgr[3];
    cv::split(input, bgr);

    cv::Mat greenMask = (bgr[1] - bgr[2]) > 30;  // B - R
    cv::imshow("greenMask", greenMask);

    std::vector<std::vector<cv::Point>> greenContours;
    cv::findContours(greenMask, greenContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if(greenContours.size())
    {
        std::vector<cv::Point> maxGreenContours = *std::max_element(greenContours.begin(), greenContours.end(),
        [](const std::vector<cv::Point> &contour1, const std::vector<cv::Point> &contour2)
        {
            return cv::contourArea(contour1, false) < cv::contourArea(contour2, false);
        });

        cv::Moments greenM = cv::moments(maxGreenContours);
        greenPointLocation.x = greenM.m10 / greenM.m00;
        greenPointLocation.y = greenM.m01 / greenM.m00;

        if(pubGreenFlag)
        {
            // Eigen::Vector3d greenTargetPoint = getRealLocation(greenPointLocation);
            std::cout << "starting pub greenPoint" << std::endl;
            greenPointLocation.x = (greenPointLocation.x - dstWidth/2) / 10;
            greenPointLocation.y = (greenPointLocation.y - dstHeight/2) / 10;
            pubPoint.data.push_back(greenPointLocation.x);
            pubPoint.data.push_back(greenPointLocation.y);
            // std::cout << "greenRealPoint : " << greenTargetPoint[0] << " , " << greenTargetPoint[1] << std::endl;
        }

        std::cout << "greenPoint : " << greenPointLocation.x << " , " << greenPointLocation.y << std::endl;
        return true;
    }

    return false;
}

/*
 * image process function
 * get four corners of rectangular in roi
*/
bool getRec(cv::Mat input)
{
    cv::Mat recGrayImg, recBinaryImg;
    cv::cvtColor(input, recGrayImg, cv::COLOR_BGR2GRAY);
    cv::threshold(recGrayImg, recBinaryImg, 50.0, 255.0, cv::THRESH_BINARY_INV);
    std::vector<std::vector<cv::Point> > recContours;
    cv::findContours(recBinaryImg, recContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    // sort the area of recContours
    std::sort(recContours.begin(), recContours.end(), compareContourAreas);
    // get the top two area of recContours
    std::vector<cv::Point> maxRecContours[2];
    if (recContours.size() >= 2) 
    {
        maxRecContours[0] = recContours[recContours.size() - 1]; 
        maxRecContours[1] = recContours[recContours.size() - 2]; 
    } 
    else 
    {
        for (size_t i = 0; i < recContours.size(); ++i) {
            maxRecContours[i] = recContours[i];
        }
    }
    for(auto & contour : maxRecContours) 
    {
        // !!!!!!!!!!!!! considering situation to decide setting area of contour!!!!!!!!!!!!!!!!!
        if(cv::contourArea(contour) < 500) 
        {
            continue;
        }
        double epsilon = 0.1 * cv::arcLength(contour, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, epsilon, true);
        if(approx.size() >= 4) 
        {
            cv::drawContours(input, std::vector<std::vector<cv::Point>>(1, approx), -1, cv::Scalar(0, 255, 0), 2);
        }   
    }
    cv::RotatedRect rect1 = cv::minAreaRect(maxRecContours[0]);
    cv::RotatedRect rect2 = cv::minAreaRect(maxRecContours[1]);
    cv::Point2f box1[4], box2[4];
    rect1.points(box1);
    rect2.points(box2);
    std::vector<cv::Point2f> sorted_box1 = sortPoints(box1);
    std::vector<cv::Point2f> sorted_box2 = sortPoints(box2);
    cv::Point2f center_points[4];
    for(int i = 0; i < 4; i++) 
    {
        center_points[i] = (sorted_box1[i] + sorted_box2[i]) / 2;
        // std::cout << "rectangular" << i << " : " << center_points[i] << std::endl;
    }
    if(pubRecFlag)
    {
        // Eigen::Vector3d recTargetPoint = getRealLocation(center_points[i]);
        center_points[0].x = (center_points[0].x - dstWidth/2) / 10;
        center_points[0].y = (center_points[0].y - dstHeight/2) / 10;
        pubPoint.data.push_back(center_points[0].x);
        pubPoint.data.push_back(center_points[0].y);
        center_points[1].x = (center_points[1].x - dstWidth/2) / 10;
        center_points[1].y = (center_points[1].y - dstHeight/2) / 10;
        pubPoint.data.push_back(center_points[1].x);
        pubPoint.data.push_back(center_points[1].y);
        center_points[3].x = (center_points[3].x - dstWidth/2) / 10;
        center_points[3].y = (center_points[3].y - dstHeight/2) / 10;
        pubPoint.data.push_back(center_points[3].x);
        pubPoint.data.push_back(center_points[3].y);
        center_points[2].x = (center_points[2].x - dstWidth/2) / 10;
        center_points[2].y = (center_points[2].y - dstHeight/2) / 10;
        pubPoint.data.push_back(center_points[2].x);
        pubPoint.data.push_back(center_points[2].y);
        return true;
    }
    return false;
}

/*
 * image process function
 * sort four corners of rectangular from the leftest in clockwise
*/
std::vector<cv::Point2f> sortPoints(cv::Point2f points[4]) 
{
    // 将四个点存储到std::vector中
    std::vector<cv::Point2f> sortedPoints(points, points + 4);
    
    // 根据y坐标对点进行排序，y坐标小的在前面
    std::sort(sortedPoints.begin(), sortedPoints.end(), 
        [](const cv::Point2f &a, const cv::Point2f &b) -> bool { 
            return a.y < b.y; 
        });
    
    // 现在，sorted_points[0]和sorted_points[1]是上面的两个点，sorted_points[2]和sorted_points[3]是下面的两个点
    // 再根据x坐标对上面和下面的点进行排序，x坐标小的在前面
    if (sortedPoints[0].x > sortedPoints[1].x) 
        std::swap(sortedPoints[0], sortedPoints[1]);
    if (sortedPoints[2].x > sortedPoints[3].x) 
        std::swap(sortedPoints[2], sortedPoints[3]);
    
    // 现在，sorted_points中的点是顺时针排序的，从左上角开始
    return sortedPoints;
}

/*
 * image process function
 * calculate the distance between pixels
*/
double calculatePixelDistance(const cv::Point2f& point1, const cv::Point2f& point2) 
{
    double dx = point2.x - point1.x;
    double dy = point2.y - point1.y;
    return std::sqrt(dx * dx + dy * dy);
}

/*
 * ros callback function
 * get selected mode and enable flag
*/
void getState(const std_msgs::Int8 stateFlag)
{
    int8_t flag = stateFlag.data;
    ROS_INFO("flag is %d", flag);
    //---------------- start standarlization and standardizing--------------------
    if(flag==1 || flag==2 || flag==3)
    {
        recFlag = false;
        standardizeFlag = false;
        trackFlag = false;
        pubRedFlag = false;
        pubRecFlag = false;
        pubGreenFlag = false;
        if(redFlag)
        {
            srcPoints.push_back(redPointLocation);
            std::cout <<  redPointLocation << std::endl;
        }
        if(greenFlag)
        {
            srcPoints.push_back(greenPointLocation);
            std::cout <<  greenPointLocation << std::endl;
        }  
    }
    //-------------------- finish standarlization--------------------------------
    if(flag == 4)
    {
        recFlag = false;
        trackFlag = false;
        pubRedFlag = false;
        pubRecFlag = false;
        pubGreenFlag = false;
        if(redFlag)
        {
            srcPoints.push_back(redPointLocation);
            standardizeFlag = true;
            std::cout <<  redPointLocation << std::endl;
        }
        if(greenFlag)
        {
            srcPoints.push_back(greenPointLocation);
            standardizeFlag = true;
            std::cout <<  greenPointLocation << std::endl;
        }  
    }
    //----------------------basic requirement(1)--------------------------------
    //---------------------get and pub red laser point--------------------------
    if(flag == 5)
    {
        if(redFlag)
        {
            recFlag = false;
            pubRedFlag = true;
            pubRecFlag = false;
            pubGreenFlag = false;
            trackFlag = false;
        }
    }
    //----------------------basic requirement(2)--------------------------------
    //---------------------get and pub red laser point--------------------------
    //------------------extend requirement(1) for red laser system--------------
    if(flag == 6)
    {
        if(redFlag)
        {
            recFlag = false;
            pubRedFlag = true;
            pubRecFlag = false;
            pubGreenFlag = false;
            trackFlag = false;
        }
    }
    //----------------------basic requirement(3)(4)-----------------------------
    //---------------------get and pub rectangular point------------------------
    //---------------------get and pub red laser point--------------------------
    //------------------extend requirement(2) for red laser system--------------
    if(flag == 7)
    {
        if(redFlag)
        {
            recFlag = true;
            pubRedFlag = false;
            pubRecFlag = true;
            pubGreenFlag = false;
            trackFlag = false;
        }
    }
    //-------------extend requirement(1)(2) for green laser system--------------
    //-------------------get and pub red+green laser point----------------------
    if(flag == 8)
    {
        if(greenFlag)
        {
            recFlag = false;
            trackFlag = true;
            pubRecFlag = false;
            pubGreenFlag = false;
            pubRedFlag = false;
        }
    }
}
