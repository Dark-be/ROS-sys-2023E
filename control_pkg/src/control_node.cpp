#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <fstream>

/*
1  2  3  4
5  6  7  8
9  10 11 12
13 14 15 16
*/
#define KEY_CONFIRM 15
#define KEY_INDEX_1 9
#define KEY_INDEX_2 10
#define KEY_INDEX_3 11
#define KEY_INDEX_4 12
#define KEY_INDEX_5 13
#define KEY_INDEX_6 14

#define KEY_Up 1
#define KEY_Down 6
#define KEY_Left 5
#define KEY_Right 2

#define KEY_ChangeS 3
#define KEY_Pause 16
#define KEY_RESET 8

typedef struct 
{
    float A;
    float B;
    float C;
    float D;
    float E;
    float F;
    float G;
    float H;
    float I;
} _parameters;

_parameters M;

void Calibration(float x[3],float y[3],float x_[3],float y_[3]){
    M.A=((x_[0]-x_[1])/(y[0]-y[1]) - (x_[0]-x_[2])/(y[0]-y[2]))
        /((x[0]-x[1])/(y[0]-y[1]) - (x[0]-x[2])/(y[0]-y[2]));

    M.B=((x_[0]-x_[2])/(y[0]-y[2])) - M.A*((x[0]-x[2])/(y[0]-y[2]));

    M.C=x_[0]-M.A*x[0]-M.B*y[0];

    M.E=((y_[2]-y_[0])/(x[2]-x[0]) - (y_[2]-y_[1])/(x[2]-x[1]))
        /((y[2]-y[0])/(x[2]-x[0]) - (y[2]-y[1])/(x[2]-x[1]));

    M.D=((y_[2]-y_[0])/(x[2]-x[0])) - M.E*((y[2]-y[0])/(x[2]-x[0]));

    M.F=y_[0]-M.D*x[0]-M.E*y[0];

    M.G=0;

    M.H=0;

    M.I=1;
}

void Transfer(float x_camera,float y_camera, float* x_servo, float* y_servo){
    *x_servo=M.A*x_camera+M.B*y_camera+M.C;
    *y_servo=M.D*x_camera+M.E*y_camera+M.F;
}

//按键信息 ------------------------------------------------------------------------------------
int key_index=0;
int sys_pause=0;
void key_callback(const std_msgs::Int8& msg){
    uint8_t buffer[1] = {static_cast<uint8_t>(msg.data)};
    key_index=msg.data-'0';
    ROS_INFO("Control_node:recv key %d and ready to change something",key_index);
}

//图像信息 ------------------------------------------------------------------------------------
int img_flag=0;//send
int rect_flag=0;
float laser_point[2]={0,0};
float rectangle_point[8]={0,0, 0,0, 0,0, 0,0};
size_t arrayLength = 0;

void img_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    ROS_INFO("Control_node:recv img: length:%d",msg->data.size());
    arrayLength = msg->data.size();
    if(arrayLength==2){
        laser_point[0]=msg->data[0];
        laser_point[1]=msg->data[1];
        ROS_INFO("Get Laser Point %f %f",
        laser_point[0],laser_point[1]);
        
    }
    if(arrayLength==8){
        rect_flag=1;
        // Transfer(msg->data[0],msg->data[1],&rectangle_point[0],&rectangle_point[1]);
        // Transfer(msg->data[2],msg->data[3],&rectangle_point[2],&rectangle_point[3]);
        // Transfer(msg->data[4],msg->data[5],&rectangle_point[4],&rectangle_point[5]);
        // Transfer(msg->data[6],msg->data[7],&rectangle_point[6],&rectangle_point[7]);
        rectangle_point[0]=msg->data[0];
        rectangle_point[1]=msg->data[1];
        rectangle_point[2]=msg->data[2];
        rectangle_point[3]=msg->data[3];
        rectangle_point[4]=msg->data[4];
        rectangle_point[5]=msg->data[5];
        rectangle_point[6]=msg->data[6];
        rectangle_point[7]=msg->data[7];
        ROS_INFO("Get Rectangle Point %f %f %f %f %f %f %f %f",
        rectangle_point[0],rectangle_point[1],rectangle_point[2],rectangle_point[3],
        rectangle_point[4],rectangle_point[5],rectangle_point[6],rectangle_point[7]);
        
    }
}

//tf信息 ------------------------------------------------------------------------------------
float tf_point[8]={0,0,0,0,0,0,0,0};
void tf_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){

}

//舵机控制 ------------------------------------------------------------------------------------
float servo_rawtarget[2]={1024,1024};
float servo_raw[2]={1024,1024};

float servo_angletarget[2]={0,0};
float servo_angle[2]={0,0};
float servo_XYtarget[2]={0,0};
float servo_XY[2]={0,0};

float error[2]={0,0};
float error_sum[2]={0,0};
float error_last[2]={0,0};

//暂不使用舵机反馈
// void servo_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
//     ROS_INFO("Control_node:recv servo: ",msg->data);
// }

//边线和中心标定
float edge_point[8]={-19,48.7,  33,49,  32,-1.4,  -20.2,-3};//{-25,25 25,25 25,-25 -25,-25}
void SaveRectangle(int index,float x,float y){
    edge_point[index*2]=x;
    edge_point[index*2+1]=y;
}
float center_point[2]={4.545,23.5};
void SaveCenter(int x,int y){
    center_point[0]=x;
    center_point[1]=y;
}

//建模XY与PitchRoll的关系
const float L=100;
const float l=4;
float servo_to_camera[]={0,-24,1.24,-1};//offset scale servo 1~camera 2;then 2 
void AngleToXY(float pitch,float roll,float& x,float& y){
    y=tanf(pitch)*(L)+l*cosf(pitch)+l/cosf(pitch);
    x=tanf(roll)*(sqrtf(y*y+L*L));
}
void XYToAngle(float x,float y,float& pitch,float& roll){
    pitch=atan((y-(l*cosf(pitch)+l/cosf(pitch)))/L);
    roll=atan((x)/sqrtf(y*y+L*L));
}

std::string filename="/home/jetson/workspace/trackingsys/src/control_pkg/data.txt";
// A function that reads data from a file and stores them in an array
void read_data(const std::string& filename, float data[8]) {
    std::ifstream input(filename);
    if (input.is_open()) {
        for (int i = 0; i < 8; i++) {
            input >> data[i];
        }
        input >> center_point[0] >> center_point[1];
        input.close();
    } 
    else {
        std::cerr << "Error: Unable to open file " << filename << "\n";
    }
}

// A function that writes data from an array to a file
void write_data(const std::string& filename, const float data[8]) {
    std::ofstream output(filename);
    if(output.is_open()) {
        for(int i = 0; i < 8; i++) {
            output << data[i] << "\n";
        }
        output << center_point[0] << "\n" << center_point[1];
        output.close();
    }
    else {
        std::cerr << "Error: Unable to open file " << filename << "\n";
    }
}

float fix_speed=20;
int speed_flag=0;
void key_move_laser(int key_index){
    if(key_index==KEY_Up){
        servo_XYtarget[1]+=fix_speed;
    }
    else if(key_index==KEY_Right){
        servo_XYtarget[0]+=fix_speed;
    }
    else if(key_index==KEY_Down){
        servo_XYtarget[1]-=fix_speed;
    }
    else if(key_index==KEY_Left){
        servo_XYtarget[0]-=fix_speed;
    }
    else if(key_index==KEY_ChangeS){
        speed_flag++;
        if(speed_flag>3){
            speed_flag=0;
        }
        if(speed_flag==3){
            fix_speed=0.1;
        }
        else if(speed_flag==2){
            fix_speed=0.5;
        }
        else if(speed_flag==1){
            fix_speed=5;
        }
        else if(speed_flag==0){
            fix_speed=20;
        }
    }
}



int main(int argc, char **argv){
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ROS_INFO("Intializing Control...");
    std::string offsetxStr, offsetyStr, scalexStr, scaleyStr;
    if (nh.getParam("control_node_/offsetx", offsetxStr) &&
      nh.getParam("control_node_/offsety", offsetyStr) &&
      nh.getParam("control_node_/scalex", scalexStr) &&
      nh.getParam("control_node_/scaley", scaleyStr))
    {
        // 将字符串值转换为浮点数类型
        servo_to_camera[0] = std::stof(offsetxStr);
        servo_to_camera[1] = std::stof(offsetyStr);
        servo_to_camera[2] = std::stof(scalexStr);
       
        servo_to_camera[3] = std::stof(scaleyStr);
    }
    read_data(filename,edge_point);
    float c_pointx[]={-25,-25,25};
    float c_pointy[]={25,-25,-25};
    float s_pointx[]={edge_point[6],edge_point[0],edge_point[2]};
    float s_pointy[]={edge_point[7],edge_point[1],edge_point[3]};
    //-25,25  -25,-25  25,-25 
    Calibration(c_pointx,c_pointy,s_pointx,s_pointy);

    //ROS_INFO("%f %f %f %f %f %f %f %f %f",M.A,M.B,M.C,M.D,M.E,M.F,M.G,M.H,M.I);
    ROS_INFO("read: %f %f %f %f %f %f %f %f",edge_point[0], edge_point[1], edge_point[2], edge_point[3], edge_point[4], edge_point[5], edge_point[6], edge_point[7]);
    //订阅话题读取键盘按键，点坐标（分别是点和矩形四点），舵机角度反馈
    ros::Subscriber key_sub = nh.subscribe("key/read", 10, key_callback);
    ros::Subscriber img_sub = nh.subscribe("img/read", 10, img_callback);
    //暂不使用
    //ros::Subscriber servo_sub = nh.subscribe("servo/read", 10, servo_callback);
    //发布话题控制舵机，控制声光提示,控制图像处理模式
    ros::Publisher key_pub = nh.advertise<std_msgs::Int8>("key/write", 10);
    ros::Publisher img_pub = nh.advertise<std_msgs::Int8>("img/write", 10);
    ros::Publisher servo_pub = nh.advertise<std_msgs::Int16MultiArray>("servo/write", 10);

    // 100hz频率执行
    std_msgs::Int8 key_msg;
    std_msgs::Int8 img_msg;
    std_msgs::Int16MultiArray servo_msg;
    servo_msg.data.resize(4);
    servo_msg.data[1]=600;//vel
    servo_msg.data[3]=600;//vel

    double start_time=ros::Time::now().toSec();
    double run_time=0;

    int global_state=0;
    int select_state=0;
    int count=1;

    long counter=0;

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        counter++;
        if(key_index!=0){
            ROS_INFO("key_index:%d",key_index);
            //key_msg.data=key_index;
        }

        servo_angle[0]=(servo_raw[0]-1024)/2048.0*3.1415926;
        servo_angle[1]=(servo_raw[1]-1024)/2048.0*3.1415926;

        AngleToXY(servo_angle[0],servo_angle[1],servo_XY[1],servo_XY[0]);//get xy
        if(key_index==KEY_Pause){
            start_time=ros::Time::now().toSec();
            sys_pause=!sys_pause;
        }
        //暂停未完成
        if(!sys_pause){
            run_time+=(ros::Time::now().toSec()-start_time);
            start_time=ros::Time::now().toSec();
            //case 0 初始状态
            //case 1 标定屏幕边线四角
            //case 2 红色激光回到中心
            //case 3 巡屏幕边线
            //case 4 寻A4标靶边线
            switch (global_state){
            case 0:{
                if(key_index!=0&&key_index!=KEY_CONFIRM){
                    select_state=key_index;
                }
                if(key_index==KEY_CONFIRM){
                    global_state=select_state;
                    switch (select_state)
                    {
                    case 1:{
                        count=1;
                    }
                    case 4:{
                        rect_flag=0;
                        img_msg.data=5;
                        img_pub.publish(img_msg);
                        break;
                    }
                    default:
                        break;
                    }
                    select_state=0;
                }

                break;
            }
            //按键调整激光位置 0.1 0.5 5 20cm 4挡 按顺序 左上 右上 右下 左下 中心
            case 1:{
                key_move_laser(key_index);
                if(key_index==KEY_CONFIRM){
                    if(count<5){
                        img_msg.data=count;
                        img_pub.publish(img_msg);
                        SaveRectangle(count-1,servo_XY[0],servo_XY[1]);
                    }
                    else{
                        SaveCenter(servo_XY[0],servo_XY[1]);
                    }
                    count++;
                    if(count==6){//标定完成
                        ROS_INFO("Save Finished");
                        write_data(filename,edge_point);
                        global_state=0;
                        count=1;
                    }
                }
                if(key_index==KEY_RESET){
                    count=0;
                    global_state=0;
                }
                break;
            }
                
            case 2:{
                key_move_laser(key_index);
                if(key_index==KEY_CONFIRM){
                    servo_XYtarget[0]=center_point[0];
                    servo_XYtarget[1]=center_point[1];
                }
                break;
            }
            
            case 3:{
                const int index0=2*(((int)run_time%4/1));//0~23 /6=0~3  0 2 4 6
                const int index1=index0+1;// 1 3 5 7
                float x_delta=abs(edge_point[index0]-servo_XYtarget[0]);
                float y_delta=abs(edge_point[index1]-servo_XYtarget[1]);
                if(sqrtf(x_delta*x_delta+y_delta*y_delta)>10){
                    x_delta=0.1;
                    y_delta=0.1;
                }
                else if(sqrtf(x_delta*x_delta+y_delta*y_delta)>5){
                    x_delta=0.12;
                    y_delta=0.12;
                }
                else if(sqrtf(x_delta*x_delta+y_delta*y_delta)>3){
                    x_delta=1;
                    y_delta=1;
                }
                servo_XYtarget[0]+=x_delta*(edge_point[index0]-servo_XYtarget[0]);//pos x
                servo_XYtarget[1]+=y_delta*(edge_point[index1]-servo_XYtarget[1]);//pos y
                break;
            }
            
            case 4:{
                const int index0=2*(((int)run_time%8/2));//0~23 /6=0~3  0 2 4 6
                const int index1=index0+1;// 1 3 5 7
                if(rect_flag==1){
                    float x_delta=abs((rectangle_point[index0]+servo_to_camera[0])*servo_to_camera[2]-servo_XYtarget[0]);
                    float y_delta=abs((rectangle_point[index1]+servo_to_camera[1])*servo_to_camera[3]-servo_XYtarget[1]);
                    if(sqrtf(x_delta*x_delta+y_delta*y_delta)>10){
                        x_delta=0.1;
                        y_delta=0.1;
                    }
                    else if(sqrtf(x_delta*x_delta+y_delta*y_delta)>5){
                        x_delta=0.15;
                        y_delta=0.15;
                    }
                    else if(sqrtf(x_delta*x_delta+y_delta*y_delta)>3){
                        x_delta=1;
                        y_delta=1;
                    }
                    servo_XYtarget[0]+=x_delta*((rectangle_point[index0]+servo_to_camera[0])*servo_to_camera[2]-servo_XYtarget[0]);//pos x
                    servo_XYtarget[1]+=y_delta*((rectangle_point[index1]+servo_to_camera[1])*servo_to_camera[3]-servo_XYtarget[1]);//pos y
                }
                break;
            }
            
            case 5:{
                key_move_laser(key_index);
                if(key_index==KEY_INDEX_1){
                    count=1;
                }
                else if(key_index==KEY_INDEX_2){
                    count=2;
                }
                else if(key_index==KEY_INDEX_3){
                    count=3;
                }
                else if(key_index==KEY_INDEX_4){
                    count=4;
                }
                else if(key_index==KEY_INDEX_5){
                    count=5;
                }
                else if(key_index==KEY_INDEX_6){
                    ROS_INFO("saved %d",count);
                    if(count<5){
                        img_msg.data=count;
                        img_pub.publish(img_msg);
                        SaveRectangle(count-1,servo_XY[0],servo_XY[1]);
                    }
                    else{
                        SaveCenter(servo_XY[0],servo_XY[1]);
                    }
                }
                else if(key_index==KEY_CONFIRM){//标定完成
                    ROS_INFO("Save Finished");
                    write_data(filename,edge_point);
                    global_state=0;
                    count=1;
                }
                break;
            }
            
            default:
                break;
            }

            error[0]=servo_XYtarget[0]-servo_XY[0];
            error[1]=servo_XYtarget[1]-servo_XY[1];
            //纯P
            servo_raw[0]+=error[0]*5;
            servo_raw[1]+=error[1]*5;

            servo_msg.data[0]=servo_raw[0];//pos x
            servo_msg.data[2]=servo_raw[1];//pos y

            ROS_INFO("state:%d speed:%d pos:%.3f %.3f target:%.3f %.3f",
            global_state,speed_flag,servo_XY[0],servo_XY[1],servo_XYtarget[0],servo_XYtarget[1],servo_raw[0],servo_raw[1]);

            
            servo_pub.publish(servo_msg);
        }
        else {
            ROS_INFO("System Pause");
        }
        if(key_index==KEY_RESET){
            global_state=0;
        }
        key_index=0;
        loop_rate.sleep();
    }
}
