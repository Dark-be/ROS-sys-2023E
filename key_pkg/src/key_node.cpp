#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Int8.h>


//串口类
serial::Serial ser;
//开串口
int open_serial(std::string port){
    try {
        ser.setPort(port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR("Unable to open serial port %s", port.c_str());
        return -1;
    }
    if (ser.isOpen()) {
        return 0;
    } else {
        return -1;
    }
}

//发送到下位机
void servo_callback(const std_msgs::Int8& msg){
    uint8_t buffer[1] = {static_cast<uint8_t>(msg.data)};
    ROS_INFO("Key_node:recv %c and send to keyboard",msg.data);
    ser.write(buffer, 1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "key_node");
    ros::NodeHandle nh;
    std::string serial_port="/dev/ttyUSB0";
    //读取其他节点发布的keywrite话题调用write_callback
    ros::Subscriber key_sub = nh.subscribe("key/write", 10, servo_callback);
    //向其它节点发布keyread信息
    ros::Publisher key_pub = nh.advertise<std_msgs::Int8>("key/read",10);
    ROS_INFO("Intializing Key...");
    if(open_serial(serial_port)==-1){
        ROS_ERROR("Failed to init key!");
        return -1;
    }
    ROS_INFO("Key initialized");
    // 100hz频率执行
    std_msgs::Int8 read_msg;
    std::string read_str;

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        if (ser.available()) {
            read_str=ser.read(ser.available());

            read_msg.data=read_str[0];
            key_pub.publish(read_msg);

            ROS_INFO("Key_node read: %d",read_str[0]);
        }
        loop_rate.sleep();
    }
}