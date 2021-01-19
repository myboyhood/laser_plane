#include <ros/ros.h>
#include <serial/serial.h> //ROS已经内置了的串口包
#include <nav_msgs/Odometry.h>
#include <string>
#include <string.h>
#include <iostream>

// [need to install serial pkg first!!!!!] sudo apt-get install ros-kinetic-serial

#define BYTE0(dwTemp) (*((uint8_t *)(&dwTemp)))
#define BYTE1(dwTemp) (*((uint8_t *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((uint8_t *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((uint8_t *)(&dwTemp) + 3))

#define SCALE 10000000.0
#define SCALE_XYZ 10000.0
serial::Serial ser;   //声明串口对象
double coordinate[2]; //lat,lon
float enu_xyz[3];
float enu_vxyz[3];
float yaw;
u_char data;
u_char pkg_tmp[37];
int _coordinate[4];
int _enu_xyz[2];
int _enu_vxyz[2];
int _yaw;
nav_msgs::Odometry msg;
ros::Publisher read_pub;

void update_state()
{
    u_char check = 0x00;
    // gps
    for (int i = 0; i < 4; i++)
    {
        BYTE3(_coordinate[i]) = pkg_tmp[4 * i];
        BYTE2(_coordinate[i]) = pkg_tmp[4 * i + 1];
        BYTE1(_coordinate[i]) = pkg_tmp[4 * i + 2];
        BYTE0(_coordinate[i]) = pkg_tmp[4 * i + 3];
        check += pkg_tmp[4 * i];
        check += pkg_tmp[4 * i + 1];
        check += pkg_tmp[4 * i + 2];
        check += pkg_tmp[4 * i + 3];
    }
    // xyz
    for (int i = 0; i < 2; i++)
    {
        BYTE3(_enu_xyz[i]) = pkg_tmp[16 + 4 * i];
        BYTE2(_enu_xyz[i]) = pkg_tmp[17 + 4 * i];
        BYTE1(_enu_xyz[i]) = pkg_tmp[18 + 4 * i];
        BYTE0(_enu_xyz[i]) = pkg_tmp[19 + 4 * i];
        check += pkg_tmp[16 + 4 * i];
        check += pkg_tmp[17 + 4 * i];
        check += pkg_tmp[18 + 4 * i];
        check += pkg_tmp[19 + 4 * i];
    }
    for (int i = 0; i < 2; i++)
    {
        BYTE3(_enu_vxyz[i]) = pkg_tmp[24 + 4 * i];
        BYTE2(_enu_vxyz[i]) = pkg_tmp[25 + 4 * i];
        BYTE1(_enu_vxyz[i]) = pkg_tmp[26 + 4 * i];
        BYTE0(_enu_vxyz[i]) = pkg_tmp[27 + 4 * i];
        check += pkg_tmp[24 + 4 * i];
        check += pkg_tmp[25 + 4 * i];
        check += pkg_tmp[26 + 4 * i];
        check += pkg_tmp[27 + 4 * i];
    }
    BYTE3(_yaw) = pkg_tmp[32];
    BYTE2(_yaw) = pkg_tmp[33];
    BYTE1(_yaw) = pkg_tmp[34];
    BYTE0(_yaw) = pkg_tmp[35];
    check+=pkg_tmp[32];
    check+=pkg_tmp[33];
    check+=pkg_tmp[34];
    check+=pkg_tmp[35];
    if (check == pkg_tmp[36])
    {
        coordinate[0] = double(_coordinate[0]) + double(_coordinate[1] / SCALE);
        coordinate[1] = double(_coordinate[2]) + double(_coordinate[3] / SCALE);
        enu_xyz[0] = float(_enu_xyz[0]/SCALE_XYZ);
        enu_xyz[1] = float(_enu_xyz[1]/SCALE_XYZ);
        enu_vxyz[0] = float(_enu_vxyz[0]/SCALE_XYZ);
        enu_vxyz[1] = float(_enu_vxyz[1]/SCALE_XYZ);
        yaw = float(_yaw/SCALE_XYZ);
        // ROS_INFO_STREAM("lat: "<< _coordinate[0]<<_coordinate[1] <<" lon: "<< _coordinate[2]<<_coordinate[3]);
        ROS_INFO("lat: %8.8f  lon:%8.8f ", coordinate[0], coordinate[1]);
        ROS_INFO("ENU X: %4.4f  Y:%4.4f vX: %4.4f  vY:%4.4f yaw: %4.4f", enu_xyz[0], enu_xyz[1], enu_vxyz[0], enu_vxyz[1],-yaw+1.57);
        msg.pose.pose.position.x = enu_xyz[0];
        msg.pose.pose.position.y = enu_xyz[1];
        msg.twist.twist.linear.x = enu_vxyz[0];
        msg.twist.twist.linear.y = enu_vxyz[1];
        msg.pose.pose.orientation.x = coordinate[0];
        msg.pose.pose.orientation.y = coordinate[1];
        msg.pose.pose.orientation.z = -yaw+1.57;
        msg.header.stamp = ros::Time::now();
        read_pub.publish(msg);
    }
    else
    {
        ROS_INFO_STREAM("data check failed!!");
    }
}

void data_decoding()
{
    ser.read(&data, 1);
    if (data == 0xFE)
    {
        data = '0';
        ser.read(&data, 1);
        if (data == 0x22)
        {
            for (int i = 0; i < 37; i++)
            {
                data = '0';
                ser.read(&data, 1);
                pkg_tmp[i] = data;
            }
            data = '0';
            ser.read(&data, 1);
            update_state();
        }
    }
}

int main(int argc, char **argv)
{
    //初始化节点 T265_parse
    ros::init(argc, argv, "px4flow_node_T265");
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅主题，并配置回调函数
    read_pub = nh.advertise<nav_msgs::Odometry>("/car_pos_vel_to_mission_topic", 1);

    try
    {
        ser.setPort("/dev/xbee_base");
        ser.setBaudrate(19200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    while (ros::ok())
    {
        if (ser.isOpen())
        {
            data_decoding();
        }
    }
}
