#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

/*
1  2  3  4
5  6  7  8
9  10 11 12
13 14 15 16
*/
#define KEY_CONFIRM 4
#define KEY_Up 1
#define KEY_Down 6
#define KEY_Left 5
#define KEY_Right 2
#define KEY_ChangeS 3

//key 1~16 ------------------------------------------------------------------------------------
int key_index=0;
void key_callback(const std_msgs::Int8& msg){
    uint8_t buffer[1] = {static_cast<uint8_t>(msg.data)};
    key_index=msg.data;
    ROS_INFO("Control_node:recv key %d and ready to change something",msg.data);
}

//img point或者发rectangle 2 ------------------------------------------------------------------------------------
int img_flag=0;
size_t arrayLength = 0;
std::vector<float> LaserPoint;
float RectanglePoint[8]={1420,1167,902,1156,916,654,1359,645};
void img_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    ROS_INFO("Control_node:recv img: length:%d",msg->data.size());
    //if(msg->data.size()==2);
    arrayLength = msg->data.size();
    for (size_t i = 0; i < arrayLength; i++) {
        ROS_INFO("Control_node:recv img: %f",msg->data[i]);
        LaserPoint.push_back(msg->data[i]);
    }
}

//servo pitch roll ------------------------------------------------------------------------------------
float servo_rawtarget[2]={1024,1024};
float servo_raw[2]={1024,1024};

float servo_angletarget[2]={0,0};
float servo_angle[2]={0,0};
float servo_XYtarget[2]={0,0};
float servo_XY[2]={0,0};

float error[2]={0,0};
float error_sum[2]={0,0};
float error_last[2]={0,0};

//暂不使用
void servo_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
    ROS_INFO("Control_node:recv servo: ",msg->data);
}

//pitch roll
float rectangle_point[8]={16,-5.5,  -15.0,-6,  -15.7,25.5,  16.4,26};
void SaveRectangle(int index,int x,int y){
    rectangle_point[index*2]=x;
    rectangle_point[index*2+1]=y;
}
//绝对坐标的误差
void SetServoTarget(float target_x,float target_y,float current_x,float current_y,float dt){
    
}

const float L=60;
const float l=4;
void AngleToXY(float pitch,float roll,float& x,float& y){
    y=tanf(pitch)*(L)+l*cosf(pitch)+l*sinf(pitch)*sinf(pitch);
    x=tanf(roll)*(sqrtf(y*y+L*L));
}
void XYToAngle(float x,float y,float& pitch,float& roll){
    pitch=atan(y-(l*cosf(pitch)+l*sinf(pitch)*sinf(pitch))/L);
    roll=atan(x/sqrtf(y*y+L*L));
}


int main(int argc, char **argv){
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
    servo_msg.data.resize(4);
    servo_msg.data[1]=100;//vel
    servo_msg.data[3]=100;//vel
    double start_time=ros::Time::now().toSec();
    double run_time=0;

    int global_state=0;
    int select_state=0;
    int count=1;
    int speed_flag=0;//0:slow 1:fast
    float fix_speed=0.005;



    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        //key_msg.data='y';
        
        run_time=ros::Time::now().toSec()-start_time;

        //case 0 初始状态
        //case 1 标定屏幕边线四角
        //case 2 红色激光回到中心
        //case 3 巡屏幕边线
        //case 4 寻A4标靶边线
        if(key_index!=0){
            ROS_INFO("key_index:%d",key_index);
            key_msg.data=key_index;
        }
        servo_angle[0]=(servo_raw[0]-1024)/2048.0*3.1415926;//roll->x
        servo_angle[1]=(servo_raw[1]-1024)/2048.0*3.1415926;//pitch->y

        AngleToXY(servo_angle[0],servo_angle[1],servo_XY[1],servo_XY[0]);//get xy

        switch (global_state)
        {
        case 0:{
            if(key_index!=0&&key_index!=KEY_CONFIRM){
                select_state=key_index;
            }
            if(key_index==KEY_CONFIRM){
                global_state=select_state;
            }
            break;
        }
        //按键调整激光位置 1 16 81
        case 1:{
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
                if(speed_flag==0){
                    fix_speed=0.1;
                }
                else if(speed_flag==1){
                    fix_speed=0.5;
                }
                else if(speed_flag==2){
                    fix_speed=5;
                }
                else if(speed_flag==3){
                    fix_speed=20;
                }
            }
            else if(key_index==KEY_CONFIRM){
                ROS_INFO("Save %d",count);
                img_msg.data=count;
                img_pub.publish(img_msg);
                SaveRectangle(count-1,servo_XY[0],servo_XY[1]);
                count++;
                if(count==5){//标定完成
                    global_state=0;
                    count=1;
                    ROS_INFO("%d %d %d %d %d %d %d %d",rectangle_point[0],rectangle_point[1],rectangle_point[2],rectangle_point[3],rectangle_point[4],rectangle_point[5],rectangle_point[6],rectangle_point[7]);
                }
            }
            break;
        }
        case 2:{
            img_msg.data=5;
            img_pub.publish(img_msg);
            float center[]={(rectangle_point[0]+rectangle_point[2]+rectangle_point[4]+rectangle_point[6]),(rectangle_point[1]+rectangle_point[3]+rectangle_point[5]+rectangle_point[7])/4};
            servo_XYtarget[0]+=0.16*(center[0]-servo_XYtarget[0]);
            servo_XYtarget[1]+=0.16*(center[1]-servo_XYtarget[1]);
            if(key_index==KEY_CONFIRM){
                global_state=0;
            }
            break;
        }
        case 3:{//((int)((run_time%8)/2)) / 2     
            const int index0=2*(((int)run_time%24/6));// 0 2 4 6
            const int index1=index0+1;// 1 3 5 7

            servo_XYtarget[0]+=0.1*(rectangle_point[index0]-servo_XYtarget[0]);//pos x
            servo_XYtarget[1]+=0.1*(rectangle_point[index1]-servo_XYtarget[1]);//pos y
            if(key_index==KEY_CONFIRM){
                global_state=0;
            }
            break;
        }
        case 4:{

            break;
        }
        default:
            break;
        }
        
        error[0]=servo_XYtarget[0]-servo_XY[0];
        error[1]=servo_XYtarget[1]-servo_XY[1];


        servo_raw[0]+=error[0]*5;
        servo_raw[1]+=error[1]*5;


        servo_msg.data[0]=servo_raw[0];//pos x
        servo_msg.data[2]=servo_raw[1];//pos y

        ROS_INFO("state:%d s:%d xy:%.3f %.3f,target:%.3f %.3f target raw:%d %d",
        global_state,speed_flag,servo_XY[0],servo_XY[1],servo_XYtarget[0],servo_XYtarget[1],servo_raw[0],servo_raw[1]);
        key_index=0;
        //img_pub.publish(img_msg);
        if(key_msg.data!=0)
        key_pub.publish(key_msg);
        servo_pub.publish(servo_msg);

        loop_rate.sleep();
    }
}
