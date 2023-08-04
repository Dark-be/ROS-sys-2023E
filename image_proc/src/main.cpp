#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>  // For file operations
#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>

using namespace cv;

/*
 * set global parameters
*/
bool recFlag = false;
bool trackFlag = false;
bool standardizeFlag = false;
bool readFile = false;
bool writeFile = false;

int8_t flag = 0;
int recCount = 0;
double camIntrinsics[4];
double camDist[4];
cv::Point2f redPointLocation, greenPointLocation;
cv::Point2f redPointTransLocation, greenPointTransLocation;
std::vector<cv::Point2f> srcPoints;
float dstWidth = 500;
float dstHeight = 500;
std::vector<cv::Point2f> dstPoints = {{0,0}, {dstWidth,0}, {dstWidth,dstHeight}, {0,dstHeight} };
std_msgs::Float32MultiArray pubPoint;



//********************the declaration of functions***********************
bool getRedPoint(cv::Mat input);
bool getGreenPoint(cv::Mat input);
bool getRec(cv::Mat input);
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
    cap.set(CAP_PROP_FRAME_HEIGHT,480);//adjust resoluction
    cap.set(cv::CAP_PROP_FPS, 30);
    // cap.set(cv::CAP_PROP_EXPOSURE,10);
    // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cv::Mat input, imageROI;

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
    std::ofstream outFile;
    std::ifstream srcFile;

    while(ros::ok()) 
    {
        //-----------------------------------------ros callback-----------------------------------------------------
        ros::spinOnce();
        cap.read(input);
        cv::imshow("input", input);
        //-------------------------------------apply undistortion to image------------------------------------------
        initUndistortRectifyMap(K, D, cv::Mat_<double>::eye(3, 3), K.clone(), imageSize, CV_16SC2, map1, map2);
        factorK << camIntrinsics[0], 0, camIntrinsics[2],
               0, camIntrinsics[1], camIntrinsics[3],
               0, 0, 1;
        // cv::remap(input, input, map1, map2, cv::INTER_LINEAR);
        cv::Size s = input.size();
        cv::imshow("undistortion", input);
        //-------------------------------standardization finished, start to set roi---------------------------------
        if(standardizeFlag)
        {
            cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(srcPoints,dstPoints);
            cv::warpPerspective(input, imageROI, perspectiveMatrix, Point(dstWidth,dstHeight));
        }
        //---------------------------------------find red/green laser in image--------------------------------------
        if( !trackFlag && !recFlag)
        {
            if(standardizeFlag)
            {
                if(getRedPoint(imageROI))
                {
                    pubPoint.data.push_back(redPointTransLocation.x);
                    pubPoint.data.push_back(redPointTransLocation.y);
                    pointPub.publish(pubPoint);
                }
            }
            else
            {
                if(getRedPoint(input))
                {
                    pubPoint.data.push_back(redPointTransLocation.x);
                    pubPoint.data.push_back(redPointTransLocation.y);
                    pointPub.publish(pubPoint);
                }
            }   
        }
        //----------------------------------find black rectangular in image-----------------------------------------
        if(recFlag)
        {
            if(standardizeFlag)
            {
                if(getRec(imageROI))
                {
                    recCount++;
                    if(recCount > 10)
                    {
                        recCount = 0;
                        pointPub.publish(pubPoint);
                        recFlag = false;
                    }
                    
                    std::cout << "already pub rectangular data" << std::endl;
                    
                }
            }
            else
            {
                if(getRec(input))
                {
                    pointPub.publish(pubPoint);
                    recFlag = false;
                }
            }
        }
        //--------------------------------make green laser track red laser------------------------------------------
        if(trackFlag)
        {
            if(getRedPoint(input) && getGreenPoint(input))
            {
                pubPoint.data.push_back(redPointTransLocation.x - greenPointTransLocation.x);
                pubPoint.data.push_back(redPointTransLocation.y - greenPointTransLocation.y);
                pointPub.publish(pubPoint);
            }
        }
        if(readFile)
        {
            srcFile.open("/home/jetson/workspace/trackingsys/src/image_proc/config/standard.txt");
            std::cout << "open file successfully" << std::endl;
            float x;
            int count = 0;
            cv::Point2f middlePoint;
            while(srcFile >> x)
            {
                if(count == 0)
                {
                    count++;
                    middlePoint.x = x;
                }
                else if(count == 1)
                {
                    count = 0;
                    middlePoint.y = x;
                    srcPoints.push_back(middlePoint);
                    std::cout << "middlePoint : " << middlePoint << std::endl;
                }
            }
            readFile = false;
            standardizeFlag = true;
            srcFile.close();
        }
        
        if(writeFile)
        {
            
            if(flag == 1)
            {
                outFile.open("/home/jetson/workspace/trackingsys/src/image_proc/config/standard.txt"); 
            }
            outFile << redPointLocation.x << std::endl;
            outFile << redPointLocation.y << std::endl;
            writeFile = false;
            if(flag==4)
            {
                outFile.close();
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

    cv::Mat redMask = (bgr[2] - bgr[1]) > 80;  // B - G
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

        // Eigen::Vector3d redTargetPoint = getRealLocation(redPointLocation);
        redPointTransLocation.x = (redPointLocation.x - dstWidth/2) / 10;
        redPointTransLocation.y = (redPointLocation.y - dstHeight/2) / 10;
        // std::cout << "redRealPoint : " << redTargetPoint[0] << " , " << redTargetPoint[1] << std::endl;

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

        // Eigen::Vector3d greenTargetPoint = getRealLocation(greenPointLocation);
        std::cout << "starting pub greenPoint" << std::endl;
        greenPointTransLocation.x = (greenPointLocation.x - dstWidth/2) / 10;
        greenPointTransLocation.y = (greenPointLocation.y - dstHeight/2) / 10;
        // std::cout << "greenRealPoint : " << greenTargetPoint[0] << " , " << greenTargetPoint[1] << std::endl;

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
            cv::imshow("rectangular", input);
            std::cout << "get rectangular" << std::endl;
        }   
    }
    cv::RotatedRect rect1 = cv::minAreaRect(maxRecContours[0]);
    cv::RotatedRect rect2 = cv::minAreaRect(maxRecContours[1]);
    cv::Point2f box1[4], box2[4];
    rect1.points(box1);
    rect2.points(box2);
    // std::vector<cv::Point2f> sorted_box1 = sortPoints(box1);
    // std::vector<cv::Point2f> sorted_box2 = sortPoints(box2);
    cv::Point2f center_points[4];
    for(int i = 0; i < 4; i++) 
    {
        center_points[i] = (box1[i] + 3*box2[i]) / 4;
        // std::cout << "rectangular" << i << " : " << center_points[i] << std::endl;
        center_points[i].x = (center_points[i].x - dstWidth/2) / 10;
        center_points[i].y = (center_points[i].y - dstHeight/2) / 10;
        pubPoint.data.push_back(center_points[i].x);
        pubPoint.data.push_back(center_points[i].y);
        if(i==3)
        {
            return true;
        }
    }
    return false;
}


/*
 * ros callback function
 * get selected mode and enable flag
*/
void getState(const std_msgs::Int8 stateFlag)
{
    flag = stateFlag.data;
    ROS_INFO("flag is %d", flag);
    //---------------- start standarlization and standardizing--------------------
    if(flag==1 || flag==2 || flag==3 || flag==4)
    {
        recFlag = false;
        standardizeFlag = false;
        trackFlag = false;
        writeFile = true;
        std::cout <<  redPointLocation << std::endl;

    }
    //---------------------basic requirement(3)(4)------------------------------
    //------------------extend requirement(1) for red laser system--------------
    //---------------------get and pub red laser point--------------------------
    if(flag == 5)
    {
        if(!standardizeFlag)
            readFile = true;
        recFlag = true;
        trackFlag = false;
    }
    //-------------extend requirement(1)(2) for green laser system--------------
    //-------------------get and pub red+green laser point----------------------
    if(flag == 6)
    {
        if(!standardizeFlag)
            readFile = true;
        recFlag = false;
        trackFlag = true;
    }
}


