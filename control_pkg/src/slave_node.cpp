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
#define KEY_Pause 16
#define KEY_RESET 8

//按键信息 ------------------------------------------------------------------------------------
int key_index=0;
int sys_pause=0;
void key_callback(const std_msgs::Int8& msg){
    uint8_t buffer[1] = {static_cast<uint8_t>(msg.data)};
    key_index=msg.data-'0';
    ROS_INFO("Control_node:recv key %d and ready to change something",msg.data);
}

//图像信息 ------------------------------------------------------------------------------------
int img_flag=0;//send
float laser_point[2]={0,0};
float rectangle_point[8]={0,0, 0,0, 0,0, 0,0};
size_t arrayLength = 0;

void img_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    ROS_INFO("Control_node:recv img: length:%d",msg->data.size());
    arrayLength = msg->data.size();
    if(arrayLength==2){
        ROS_INFO("Get Laser Point %f %f");
        laser_point[0]=msg->data[0];
        laser_point[1]=msg->data[1];
    }
    if(arrayLength==8){
        ROS_INFO("Get Rectangle Point %f %f %f %f %f %f %f %f");
        rectangle_point[0]=msg->data[0];
        rectangle_point[1]=msg->data[1];
        rectangle_point[2]=msg->data[2];
        rectangle_point[3]=msg->data[3];
        rectangle_point[4]=msg->data[4];
        rectangle_point[5]=msg->data[5];
        rectangle_point[6]=msg->data[6];
        rectangle_point[7]=msg->data[7];
    }
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
float laser_vel[2]={0,0};
//暂不使用舵机反馈
// void servo_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
//     ROS_INFO("Control_node:recv servo: ",msg->data);
// }

//边线和中心标定
float edge_point[8]={-20,49,  33,49.8,  32,-0.9,  -20.6,-2.9};
void SaveRectangle(int index,float x,float y){
    edge_point[index*2]=x;
    edge_point[index*2+1]=y;
}
float center_point[2]={4.545,23.5};
void SaveCenter(int x,int y){
    center_point[0]=x;
    center_point[1]=y;
}

void SetServoTarget(float target_x,float target_y){
    
}

//建模XY与PitchRoll的关系
const float L=100;
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
    ros::init(argc, argv, "slave_node");
    ros::NodeHandle nh;
    ROS_INFO("Intializing Control...");
    //订阅话题读取键盘按键，点坐标（分别是点和矩形四点），舵机角度反馈
    ros::Subscriber key_sub = nh.subscribe("key/read", 10, key_callback);
    ros::Subscriber img_sub = nh.subscribe("img/read", 10, img_callback);
    //暂不使用
    //ros::Subscriber servo_sub = nh.subscribe("servo/read", 10, servo_callback);
    //发布话题控制舵机，控制声光提示,控制图像处理模式
    ros::Publisher key_pub = nh.advertise<std_msgs::Int8>("key/write", 10);
    ros::Publisher img_pub = nh.advertise<std_msgs::Int8>("img/write", 10);
    ros::Publisher servo_pub = nh.advertise<std_msgs::Int16MultiArray>("servo/write", 10);
    // 10hz频率执行
    std_msgs::Int8 key_msg;
    std_msgs::Int8 img_msg;
    std_msgs::Int16MultiArray servo_msg;
    servo_msg.data.resize(4);
    servo_msg.data[1]=400;//vel
    servo_msg.data[3]=400;//vel

    double start_time=ros::Time::now().toSec();
    double run_time=0;

    int global_state=0;
    int select_state=0;
    int count=1;
    int speed_flag=0;
    float fix_speed=20;

    long counter=0;
    float Kp=0.1;
    float Ki=0.0;
    float Kd=0.0;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        counter++;
        if(key_index!=0){
            ROS_INFO("key_index:%d",key_index);
            key_msg.data=key_index;
        }
        run_time=ros::Time::now().toSec()-start_time;
        servo_angle[0]=(servo_raw[0]-1024)/2048.0*3.1415926;
        servo_angle[1]=(servo_raw[1]-1024)/2048.0*3.1415926;

        AngleToXY(servo_angle[0],servo_angle[1],servo_XY[1],servo_XY[0]);//get xy
        if(key_index==KEY_Pause){
            sys_pause=!sys_pause;
        }
        //暂停未完成
        if(sys_pause){
            key_index=0;
            continue;
        }
        
        //case 0 初始状态
        //case 1 标定屏幕边线四角
        //case 2 红色激光回到中心
        //case 3 巡屏幕边线
        //case 4 寻A4标靶边线
        switch (global_state)
        {
        case 0:{
            if(key_index!=0&&key_index!=KEY_CONFIRM){
                select_state=key_index;
            }
            if(key_index==KEY_CONFIRM){
                global_state=select_state;
                switch (select_state)
                {
                case 4:{
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
        //按键调整激光位置 0.1 0.5 5 20cm 4挡
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
            else if(key_index==KEY_CONFIRM){
                img_msg.data=count;
                img_pub.publish(img_msg);
                if(count<5){
                    SaveRectangle(count-1,servo_XY[0],servo_XY[1]);
                }
                else{
                    SaveCenter(servo_XY[0],servo_XY[1]);
                }
                count++;
                if(count==6){//标定完成
                    ROS_INFO("Save Finished");
                    global_state=0;
                    count=1;
                }
            }
            if(key_index==KEY_RESET){
                global_state=0;
            }
            break;
        }
        case 2:{
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
            else if(key_index==KEY_CONFIRM){
                servo_XYtarget[0]=center_point[0];
                servo_XYtarget[1]=center_point[1];
            }
            
            if(key_index==KEY_RESET){
                global_state=0;
            }
            break;
        }
        case 3:{
            img_flag=1;
            //img_pub.publish(img);
            //servo_XYtarget[0]+=x_delta*(edge_point[index0]-servo_XYtarget[0]);//pos x
            //servo_XYtarget[1]+=y_delta*(edge_point[index1]-servo_XYtarget[1]);//pos y
            laser_point[0];
            laser_point[1];

            if(key_index==KEY_CONFIRM){
                global_state=0;
            }
            break;
        }
        case 4:{
            if(error_sum[0]>1000){
                error_sum[0]=1000;
            }
            else if(error_sum[1]<-1000){
                error_sum[1]=-1000;
            }
            if(error_sum[0]>1000){
                error_sum[0]=1000;
            }
            else if(error_sum[1]<-1000){
                error_sum[1]=-1000;
            }
            
            float target[2]={laser_point[0],laser_point[1]};
            laser_vel[0]=Kp*error[0]+Ki*error_sum[0]+Kd*(error[0]-error_last[0]);

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

        ROS_INFO("state:%d s:%d xy:%.3f %.3f,target:%.3f %.3f target raw:%.3f %.3f",
        global_state,speed_flag,servo_XY[0],servo_XY[1],servo_XYtarget[0],servo_XYtarget[1],servo_raw[0],servo_raw[1]);

        key_index=0;
        servo_pub.publish(servo_msg);
        loop_rate.sleep();
    }
}
