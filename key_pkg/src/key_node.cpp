#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Int8.h>

#include <signal.h>

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
const uint8_t head[2]={0xC8,0xFF};
//浮点转字节码并发送字节码，通知速度
void write_callback(const std_msgs::Int8& msg){
    uint8_t buffer[1] = {static_cast<uint8_t>(msg.data)};
    ROS_INFO("Send %c",msg.data);
    ser.write(head, 2);
    ser.write(buffer, 1);
}
void signalHandler(int sig)
{
    const uint8_t end[3]={0xC8,0xFF,'N'};
    ser.write(end, 3);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "key_node");
    ros::NodeHandle nh;
    std::string serial_port="/dev/ttyUSB0";
    ros::Subscriber write_sub = nh.subscribe("key/write", 100, write_callback);//写串口话题
    signal(SIGINT, signalHandler);
    ROS_INFO("Intializing Key...");
    if(open_serial(serial_port)==-1){
        ROS_ERROR("Failed to init key!");
        return -1;
    }
    ROS_INFO("Key initialized");
    // 100hz频率执行
    std_msgs::Int8 read_msg;
    std::string read_str;

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        if (ser.available()) {
            read_str=ser.read(ser.available());
            read_msg.data=std::stoi(read_str);
            ROS_INFO("Read: %s",read_str.c_str());
        }
        loop_rate.sleep();
    }
}