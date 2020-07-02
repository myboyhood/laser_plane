/*
 * position_control.cpp
 *
 * Author:mz
 *
 * Time: 2018.11.27
 *
 * 说明: mavros位置控制示例程序
 *      输入：mavros发布的位置/速度信息
 *      输出：无人机的推力和姿态信息
 *      采用位置环/速度环串级PID控制，位置环P控制，速度环PID控制
 */
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <Eigen/Core> 

#include <ros/ros.h>
#include "Parameter.h"
#include <PID.h>


//topic
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/Imu.h>

using namespace Eigen;

// //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

mavros_msgs::State current_state;           //无人机当前状态
geometry_msgs::PoseStamped pos_drone;       //读入的无人机当前位置
geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度
sensor_msgs::Imu att_drone;       //读入的无人机姿态
geometry_msgs::Vector3 angle_receive;       //读入的无人机姿态（欧拉角）

//geometry_msgs::Quaternion orientation_target;   //发给无人机的姿态指令
mavros_msgs::AttitudeTarget orientation_target;
geometry_msgs::Vector3 angle_target;
geometry_msgs::Vector3 vel_target;
geometry_msgs::Vector3 xyz_position;
geometry_msgs::Vector3 target_point_exe;
geometry_msgs::Vector3 target_point_exe_enable;
geometry_msgs::Vector3 target_point_cal;
geometry_msgs::Vector3 target_point_cal_vel;
geometry_msgs::Vector3 initial_point;
geometry_msgs::Vector3 plane_euler_now;
geometry_msgs::PoseStamped object_position;
geometry_msgs::TwistStamped object_vel;

float thrust_target;        //期望推力
float Yaw_Init;
float Yaw_Locked = 0;           //锁定的偏航角(一般锁定为0)
PID PIDVX, PIDVY, PIDVZ;    //声明PID类
Parameter param;
bool arrived_destination = false;
//bool received_pub_point = false;
std_msgs::Bool move_enable;
std::ofstream logfile;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//欧拉角转四元数
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);

float get_ros_time(ros::Time time_begin);                                            //获取ros当前时间
int pix_controller(float cur_time, float destination_x,float destination_y,float destination_z );                                                  //控制程序
void data_log(float cur_time);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;

}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pos_drone = *msg;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    vel_drone = *msg;
}

bool hasGotAtt = false;
void att_cb(const sensor_msgs::Imu::ConstPtr &msg){
    att_drone = *msg;
    hasGotAtt = true;
    angle_receive = quaternion2euler(att_drone.orientation.x, att_drone.orientation.y, att_drone.orientation.z, att_drone.orientation.w);
}

void object_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    object_position = *msg;
    target_point_cal.x = object_position.pose.position.x+param.delta_x;
    target_point_cal.y = object_position.pose.position.y+param.delta_y;
    target_point_cal.z = object_position.pose.position.z+param.delta_z;
    std::cout << "cal_x: "<< target_point_cal.x << "\tcal_y: " << target_point_cal.y << "\tcal_z: "<< target_point_cal.z << std::endl;
    if(move_enable.data == true)
    {
        target_point_exe.x = target_point_cal.x;
        target_point_exe.y = target_point_cal.y;
        target_point_exe.z = target_point_cal.z;
    }
}
void object_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    object_vel = *msg;
    target_point_cal_vel.x = object_vel.twist.linear.x;
    target_point_cal_vel.y = object_vel.twist.linear.y;
    target_point_cal_vel.z = object_vel.twist.linear.z;
//    if(move_enable.data == true)
//    {
//        target_point_exe.x = target_point_cal_vel.x;
//        target_point_exe.y = target_point_cal_vel.y;
//        target_point_exe.z = target_point_cal_vel.z;
//    }
}

void plane_move_enable_cb(const std_msgs::Bool::ConstPtr &msg){
    move_enable = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;
    std::ofstream logfile;
    std::cout << "initialize ros node"<<std::endl;
    // 【订阅】无人机当前状态/位置/速度信息
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 20, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mocap/pose_01", 20, pos_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mocap/vel_01", 20, vel_cb);
    ros::Subscriber attitude_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 20, att_cb);
    ros::Subscriber object_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mocap/pose_02",20,object_cb);
    ros::Subscriber object_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mocap/vel_02",20,object_vel_cb);
    ros::Subscriber move_enable_sub = nh.subscribe<std_msgs::Bool>("/move_enable",20,plane_move_enable_cb);

    // 【发布】飞机姿态/拉力信息 坐标系:NED系
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float32>("/cmd/thrust", 20);
    ros::Publisher orientataion_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 20);
    ros::Publisher xyz_posepub = nh.advertise<geometry_msgs::Vector3>("/xyz_position",20);
    ros::Publisher move_enable_pub = nh.advertise<std_msgs::Bool>("/move_enable",20);
    // 频率 [20Hz]
    ros::Rate rate(20.0);
    std::cout << "initialize ros node succeed"<<std::endl;
    // 读取PID参数
    std::string paraadr("/home/uav/wzy_ws/src/offb_posctl-basic_PID/src/param");
    if (param.readParam(paraadr.c_str()) == 0){
        std::cout<<"read config file error!"<<std::endl;
        return 0;
    }
    std::cout << "read param succeed"<<std::endl;
    // log输出文件初始化
    logfile.open("/home/uav/wzy_ws/src/offb_posctl-basic_PID/log.csv", std::ios::out);
    if (! logfile.is_open()){
        std::cout<<"log to file error!"<<std::endl;
        return 0;
    }

    std::cout << "delta_y: "<<param.delta_y<<"\tdelta_z: "<<param.delta_z<<std::endl;
    // 设置速度环PID参数 比例参数 积分参数 微分参数
    PIDVX.setPID(param.vx_p, param.vx_i, param.vx_d);
    PIDVY.setPID(param.vy_p, param.vy_i, param.vy_d);
    PIDVZ.setPID(param.vz_p, param.vz_i, param.vz_d);
    // 设置速度环积分上限 控制量最大值 误差死区
    PIDVX.set_sat(2, 3, 0);
    PIDVY.set_sat(2, 3, 0);
    PIDVZ.set_sat(2, 5, 0);
    std::cout << "set param succeed"<<std::endl;
    // 等待和飞控的连接
    while(ros::ok() && current_state.connected == 0)
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        ROS_INFO("Not Connected");
    }
    ROS_INFO("Connected!!");

    while(ros::ok() && !hasGotAtt)
    {
        ros::Duration(1).sleep();
        ros::spinOnce();
        ROS_INFO_STREAM("waitting for att...");
    }
    float x = att_drone.orientation.x;
    float y = att_drone.orientation.y;
    float z = att_drone.orientation.z;
    float w = att_drone.orientation.w;
    angle_receive = quaternion2euler(x, y, z, w);
    Yaw_Init = angle_receive.z;
    ROS_INFO_STREAM("Got Yaw_Init: " << Yaw_Init);

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();

        if (current_state.mode == "STABILIZED")
        {
            initial_point.x = pos_drone.pose.position.x;
            initial_point.y = pos_drone.pose.position.y;
            initial_point.z = pos_drone.pose.position.z;
            target_point_exe_enable.x = initial_point.x;
            target_point_exe_enable.y = initial_point.y;
            target_point_exe_enable.z = initial_point.z;
        }
        if(current_state.mode == "OFFBOARD" && move_enable.data == false)
        {
            target_point_exe_enable.x = initial_point.x;
            target_point_exe_enable.y = initial_point.y;
            target_point_exe_enable.z = initial_point.z;
        }
        if(current_state.mode == "OFFBOARD" && move_enable.data == true)
        {
            target_point_exe_enable.x = target_point_exe.x;
            target_point_exe_enable.y = target_point_exe.y;
            target_point_exe_enable.z = target_point_exe.z;
        }
        arrived_destination = false;
        while(arrived_destination == false)
        {
            ros::spinOnce();
            //发布xyz位置，方便在matpolt查看曲线图
            xyz_position.x = pos_drone.pose.position.x;
            xyz_position.y = pos_drone.pose.position.y;
            xyz_position.z = pos_drone.pose.position.z;
            xyz_posepub.publish(xyz_position);

            if(move_enable.data == true)
                std::cout << "-----pub_point-----"<< std::endl;

            float cur_time = get_ros_time(begin_time);  // 当前时间
            pix_controller(cur_time, (float) target_point_exe_enable.x, (float) target_point_exe_enable.y,
                           (float) target_point_exe_enable.z);                   //控制程序

            if (current_state.mode == "OFFBOARD") {
                data_log(cur_time);//log输出
                std::cout << "IS OFFBOARD!" << std::endl;
            } else {
                std::cout << "not OFFBOARD!" << std::endl;
            }
            orientataion_pub.publish(orientation_target);
            plane_euler_now = quaternion2euler(att_drone.orientation.x,att_drone.orientation.y,att_drone.orientation.z,att_drone.orientation.w);
            std::cout << "yaw_curr " << plane_euler_now.z <<std::endl;
            std::cout << "yaw_init: " << Yaw_Init <<std::endl;

            rate.sleep();
        }
        rate.sleep();
    }
    logfile.close();
    return 0;
}

/**
 * 获取当前时间 单位：秒
 */
float get_ros_time(ros::Time time_begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-time_begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - time_begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>控 制 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int pix_controller(float cur_time,float destination_x,float destination_y,float destination_z)
{
//位 置 环
    //计算误差
    float error_x = destination_x - pos_drone.pose.position.x;
    float error_y = destination_y - pos_drone.pose.position.y;
    float error_z = destination_z - pos_drone.pose.position.z;
    arrived_destination = false;
    if (abs(error_x) < 0.1 && abs(error_y) < 0.1 && abs(error_z) < 0.1)
        arrived_destination = true;
    std::cout << "error: x：" << error_x << "\ty：" << error_y << "\tz：" << error_z << std::endl;
    //计算指定速度误差
    float vel_xd = param.x_p * error_x;
    float vel_yd = param.y_p * error_y;
    float vel_zd = param.z_p * error_z;
    vel_target.x = vel_xd;
    vel_target.y = vel_yd;
    vel_target.z = vel_zd;

//速 度 环
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDVX.start_intergrate_flag = true;
    PIDVY.start_intergrate_flag = true;
    PIDVZ.start_intergrate_flag = true;
    if(current_state.mode != "OFFBOARD"){
        PIDVX.start_intergrate_flag = false;
        PIDVY.start_intergrate_flag = false;
        PIDVZ.start_intergrate_flag = false;
    }
    //计算误差
    float error_vx = vel_xd - vel_drone.twist.linear.x;
    float error_vy = vel_yd - vel_drone.twist.linear.y;
    float error_vz = vel_zd - vel_drone.twist.linear.z;
    std::cout << "error_vel: x：" << error_vx << "\ty：" << error_vy << "\tz：" << error_vz << std::endl;
    //传递误差
    PIDVX.add_error(error_vx, cur_time);
    PIDVY.add_error(error_vy, cur_time);
    PIDVZ.add_error(error_vz, cur_time);
    //计算输出
    PIDVX.pid_output();
    PIDVY.pid_output();
    PIDVZ.pid_output();

    Matrix2f A_yaw;
    A_yaw << sin(Yaw_Locked), cos(Yaw_Locked),
            -cos(Yaw_Locked), sin(Yaw_Locked);
    Vector2f mat_temp(PIDVX.Output,PIDVY.Output);       //赋值到期望推力和姿态
    Vector2f euler_temp= 1/9.8 * A_yaw.inverse() * mat_temp;
    angle_target.x = -euler_temp[1];
    angle_target.y = euler_temp[0];
    angle_target.z = Yaw_Locked + Yaw_Init;
//    angle_target.z = Yaw_Locked;

    orientation_target.orientation = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);
    thrust_target = (float)(0.05 * (9.8 + PIDVZ.Output)+param.base_thrust-0.49);   //目标推力值
//    thrust_target = (float)(0.05*PIDVZ.Output + param.base_thrust);   //目标推力值
    orientation_target.thrust = thrust_target;
//    std::cout << "PIDVZ.Output" << PIDVZ.Output << std::endl;
    std::cout << " thrustauto: " << thrust_target  << " \t base_thrust: " << param.base_thrust <<std::endl;
    std::cout <<"\t angle_yaw_expect: "<<angle_target.z<<std::endl;
    return 0;
}

/**
 * 将欧拉角转化为四元数
 * @param roll
 * @param pitch
 * @param yaw
 * @return 返回四元数
 */
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

/**
 * 将四元数转化为欧拉角形式
 * @param x
 * @param y
 * @param z
 * @param w
 * @return 返回Vector3的欧拉角
 */
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    temp.y = asin(2.0 * (z * x - w * y));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

/**
 * 将进入offboard后的位置&速度&姿态信息记录进文件
 * @param cur_time
 */
void data_log(float cur_time)
{
    logfile <<cur_time<<","<<param.pos_x <<","<<param.pos_y <<","<<param.pos_z <<","                           //set_pos
        <<pos_drone.pose.position.x <<","<<pos_drone.pose.position.y <<","<<pos_drone.pose.position.z <<","    //uav_pos
        <<vel_target.x <<","<<vel_target.y <<","<<vel_target.z <<","                                           //set_vel
        <<vel_drone.twist.linear.x <<","<<vel_drone.twist.linear.y <<","<<vel_drone.twist.linear.z <<","       //uav_vel
        <<angle_target.x  <<","<<angle_target.y  <<","<<angle_target.z  <<","                                  //set_att
        <<angle_receive.x <<","<<angle_receive.y <<","<<angle_receive.z <<","                                  //uav_att
        <<thrust_target<<std::endl;

}