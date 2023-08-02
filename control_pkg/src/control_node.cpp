#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>

//key 1~16 ------------------------------------------------------------------------------------
int key_index=0;
void key_callback(const std_msgs::Int8& msg){
    uint8_t buffer[1] = {static_cast<uint8_t>(msg.data)};
    ROS_INFO("Control_node:recv key %c and ready to change something",msg.data);
}

//img point或者发rectangle 2 ------------------------------------------------------------------------------------
int img_flag=0;
float LaserPoint[2]={0,0};
float RectanglePoint[8]={0,0,0,0, 0,0,0,0};
void img_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    ROS_INFO("Control_node:recv img: length:%d,point:%f %f",msg->data.size(),msg->data[0],msg->data[1]);
    if(msg->data.size()==2);
}

//servo pitch roll ------------------------------------------------------------------------------------
void servo_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
    ROS_INFO("Control_node:recv servo: ",msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ROS_INFO("Intializing Control...");
    //订阅话题读取键盘按键，点坐标（分别是点和矩形四点），舵机角度反馈
    ros::Subscriber key_sub = nh.subscribe("key/read", 10, key_callback);
    ros::Subscriber img_sub = nh.subscribe("img/read", 10, img_callback);
    ros::Subscriber servo_sub = nh.subscribe("servo/read", 10, servo_callback);
    //发布话题控制舵机，控制声光提示,控制图像处理模式
    ros::Publisher key_pub = nh.advertise<std_msgs::Int8>("key/write", 10);
    ros::Publisher img_pub = nh.advertise<std_msgs::Int8>("img/write", 10);
    ros::Publisher servo_pub = nh.advertise<std_msgs::Int16MultiArray>("servo/write", 10);
    // 50hz频率执行
    std_msgs::Int8 key_msg;
    std_msgs::Int8 img_msg;
    std_msgs::Int16MultiArray servo_msg;

    double start_time=ros::Time::now().toSec();
    double run_time=0;
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        key_msg.data='y';
        

        img_msg.data=img_flag;
        

        servo_msg.data.resize(4);
        servo_msg.data[0]=1000;//pos
        servo_msg.data[1]=1000;//vel
        servo_msg.data[2]=1000;//pos
        servo_msg.data[3]=1000;//vel
        

        run_time=ros::Time::now().toSec()-start_time;
        if((int)run_time%4<2){
            servo_msg.data[0]=0;//pos
            servo_msg.data[2]=0;//pos
            ROS_INFO("0");
        }
        else{

        }



        img_pub.publish(img_msg);
        key_pub.publish(key_msg);
        servo_pub.publish(servo_msg);


        loop_rate.sleep();
    }
}
