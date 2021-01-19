//
// Created by wzy on 2020/9/1.
//

#include "mission_core.h"


mission_core::mission_core() : rate(LOOPRATE){

    //! >>>>>>>> service <<<<<<<<//
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");



    //! >>>>>>>> subscriber <<<<<<<<//
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, &mission_core::state_cb ,this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom",1, &mission_core::odom_cb, this);
    start_trace_sub = nh.subscribe<std_msgs::Bool>("start_trace_topic",1, &mission_core::start_trace_cb, this);
    car_pos_sub = nh.subscribe<nav_msgs::Odometry>("/car_pos_vel_to_mission_topic",1,&mission_core::car_pos_cb,this);
//    img_ir_sub = nh.subscribe<sensor_msgs::Image>("/camera/infra1/image_rect_raw",1,&mission_core::img_ir_cb,this);
//    img_bgr_sub = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw",1,&mission_core::img_bgr_cb,this);
    ocp_control_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/ocp/setpoint_raw/attitude",1,&mission_core::ocp_control_cb,this);
    drone_pos_vision_sub = nh.subscribe<geometry_msgs::PoseStamped>("/drone_pos_vision",1,&mission_core::drone_pos_vision_cb,this);
    plane_height_sub = nh.subscribe<mavros_msgs::Altitude>("mavros/altitude", 1, &mission_core::plane_altitude_enu_cb,this); //lss

    //! >>>>>>>> ros publisher <<<<<<<< //
    px4_setpoint_position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    pva_setpoint_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    start_velocity_esti_pub = nh.advertise<std_msgs::Bool>("/start_velocity_esti_topic",1);
    pva_odom_sp_enu_pub = nh.advertise<nav_msgs::Odometry>("drone_pva_sp_enu",1);
//    drone_vision_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("drone_vision_pos",1);
    start_trace_pub = nh.advertise<std_msgs::Bool>("start_trace_topic",1);
    plane_pos_from_start_pub = nh.advertise<geometry_msgs::PoseStamped>("plane_pos_topic",1);
    car_pos_from_planeHome_pub = nh.advertise<nav_msgs::Odometry>("car_pos_vel_topic",1);
    relative_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("relative_pose",1);
    ocp_control_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",1);
    //!publish for debug
    pnp_expect_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("topic_pnp_expect_pos",1);
    drone_pos_vision_in_enu_pub = nh.advertise<geometry_msgs::PoseStamped>("topic_drone_pos_vision_in_enu",1);
    gps_target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("topic_gps_target_pos",1);
    pnp_gps_bias_pub = nh.advertise<geometry_msgs::PoseStamped>("topic_pnp_gps_bias",1);
    //!>>>>>>>> test publisher <<<<<<<< //
    drone_euler_pub = nh.advertise<geometry_msgs::PoseStamped>("drone_euler_topic",1);

}

/**
 * callback function
 */

void mission_core::drone_pos_vision_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(msg->pose.orientation.x > 0.1){
        pnp_pose_is_good = true;
    } else{
        pnp_pose_is_good = false;
    }

    drone_pos_vision[0] = msg->pose.position.x;
    drone_pos_vision[1] = msg->pose.position.y;
    drone_pos_vision[2] = msg->pose.position.z;
    pnp_z = msg->pose.orientation.z;
    cout << "999999999999999999999999999999999999999999999999 pnp_z: " << pnp_z << endl;

    cs_road_to_enu(drone_pos_vision[0],drone_pos_vision[1],drone_pos_vision_in_enu[0],drone_pos_vision_in_enu[1]);
    drone_pos_vision_in_enu[2] = drone_pos_vision[2];
    ROS_WARN("drone_pos_vision_in_enu.x(): %f",drone_pos_vision_in_enu[0]);
    ROS_WARN("drone_pos_vision_in_enu.y(): %f",drone_pos_vision_in_enu[1]);
}

void mission_core::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state_msg = *msg;
    ROS_ERROR("current_state_msg.armed: ");
    cout << "####################################current_state_msg.armed" << to_string(current_state_msg.armed) << endl;
}

void mission_core::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom_msg = *msg;
    curr_p << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z;
    curr_v << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z;
//    cout << "curr_p0:" << curr_p[0] << endl;

    drone_pos_from_start[0] = curr_p[0] - x_pos_home_origin;
    drone_pos_from_start[1] = curr_p[1] - y_pos_home_origin;
    cout << "********************* curr_p[0]: " << curr_p[0] << endl;
    cout << "curr_p[1]: " << curr_p[1] << endl;
    cout << "x_pos_home_origin: " << x_pos_home_origin << endl;
    cout << "y_pos_home_origin: " << y_pos_home_origin << endl;
//    drone_pos_from_start[2] = curr_p[2] - z_pos_home_origin;
    drone_pos_from_start[2] = plane_height_enu;
    ROS_WARN("drone_pos_from_start[2]: %f", drone_pos_from_start[2]);
}

void mission_core::plane_altitude_enu_cb(const mavros_msgs::Altitude::ConstPtr &msg)//lss
{
        Eigen::Quaterniond current_attitude;
        Eigen::Quaterniond relative_height;
        Eigen::Quaterniond enu_height;
        current_attitude.x()=odom_msg.pose.pose.orientation.x;
        current_attitude.y()=odom_msg.pose.pose.orientation.y;
        current_attitude.z()=odom_msg.pose.pose.orientation.z;
        current_attitude.w()=odom_msg.pose.pose.orientation.w;

        relative_height.x()=-0.05;
        relative_height.y()=0.05;
        relative_height.z()=-(*msg).local - 0.1;
        relative_height.w()=0;

        enu_height=current_attitude.inverse() * relative_height * current_attitude;

        cout<<"height"<<-enu_height.z()<<endl;
        plane_height_enu = -enu_height.z();

//    cout<<"planntoOriginposFLU.pose.position.x"<< planntoOriginposFLU.pose.position.x<<endl;
//    cout<<"planntoOriginposFLU.pose.position.y"<< planntoOriginposFLU.pose.position.y<<endl;
}

void mission_core::car_pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    cout << "/////////////////////msg->pose.position.x: "<<msg->pose.pose.position.x << endl;

    car_pos[0] = msg->pose.pose.position.x;
    car_pos[1] = msg->pose.pose.position.y;
    car_pos[2] = msg->pose.pose.position.z;
    if(!car_bias_yaw_flag){
        if(car_bias_count < 100){
            car_bias_count++;
            car_bias_yaw += msg->pose.pose.orientation.z;
        }
        if(car_bias_count == 100)
        {
            car_bias_yaw_flag = true;
            car_bias_yaw_const = car_bias_yaw/100.0;
        }
    }
    cout << "==================car_bias_yaw_const: " << car_bias_yaw_const << endl;

    cout << "]]]]]]]]]]]]]]]]]]]]]]]]getCarHomeFlag: " << getCarHomeFlag << endl;
    if(!getCarHomeFlag && abs(car_pos[0]) > 0.00001){
        getCarHomeFlag = true;
        carHome[0] = car_pos[0];
        carHome[1] = car_pos[1];
        carHome[2] = car_pos[2];
    }

    cs_to_enu(car_pos[0],car_pos[1],car_pos_rect[0],car_pos_rect[1], 0);
    car_pos_from_start[0] = car_pos_rect[0] - carHome[0];
    car_pos_from_start[1] = car_pos_rect[1] - carHome[1];
    cout << "------------------------car_pos[0]: " << car_pos[0] << "carHome[0]: "<< carHome[0] << endl;
    cout << "------------------------car_pos[1]: " << car_pos[1] << "carHome[1]: "<< carHome[1] << endl;
    cout << "car_pos_from_start[1]: " << car_pos_from_start[1] << endl;
    car_vel_from_start[0] = msg->twist.twist.linear.x;
    car_vel_from_start[1] = msg->twist.twist.linear.y;
    cs_to_enu(car_vel_from_start[0], car_vel_from_start[1], car_vel_rect[0], car_vel_rect[1], 0);
    car_vel_from_start[0] = car_vel_rect[0];
    car_vel_from_start[1] = car_vel_rect[1];

    // cout << "car_pos[0]:"<< car_pos[0] << endl;
}

void mission_core::start_trace_cb(const std_msgs::Bool::ConstPtr& msg){
    start_trace_flag = msg->data;
}

//void mission_core::img_ir_cb(const sensor_msgs::Image::ConstPtr& msg){
//
//    img_ir_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
//    img_ir = img_ir_ptr->image;
//}

void mission_core::ocp_control_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    ocp_control_msg.type_mask = msg->type_mask;
    ocp_control_msg.orientation.x = msg->orientation.x;
    ocp_control_msg.orientation.y = msg->orientation.y;
    ocp_control_msg.orientation.z = msg->orientation.z;
    ocp_control_msg.orientation.w = msg->orientation.w;
    ocp_control_msg.thrust = msg->thrust;
}

//void mission_core::img_bgr_cb(const sensor_msgs::Image::ConstPtr& msg){
//    img_bgr_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
//    img_bgr = img_bgr_ptr->image;
//}

/**
 * process function
 */
void mission_core::cs_road_to_enu(double &x_road,double &y_road,double &x_enu,double &y_enu){
    x_enu = x_road * cos(yaw_rotate) - y_road * sin(yaw_rotate);
    y_enu = x_road * sin(yaw_rotate) + y_road * cos(yaw_rotate);
}

void mission_core::cs_to_enu(double &x_road,double &y_road,double &x_enu,double &y_enu, double yaw){
    x_enu = x_road * cos(yaw) + y_road * sin(yaw);
    y_enu = -x_road * sin(yaw) + y_road * cos(yaw);
}

void mission_core::cs_enu_to_road(double &x_enu,double &y_enu,double &x_road,double &y_road){
    x_road = x_enu * cos(-yaw_rotate) - y_enu * sin(-yaw_rotate);
    y_road = x_enu * sin(-yaw_rotate) + y_enu * cos(-yaw_rotate);
}

/**
 * 将欧拉角转化为四元数
 * @param roll
 * @param pitch
 * @param yaw
 * @return 返回四元数
 */
geometry_msgs::Quaternion mission_core::euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}
Eigen::Quaterniond mission_core::euler2quaternion_eigen(float roll, float pitch, float yaw)
{
    Eigen::Quaterniond temp;
    temp.w() = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x() = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y() = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z() = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
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
geometry_msgs::Vector3 mission_core::quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    // I use ENU coordinate system , so I plus ' - '
    temp.y = - asin(2.0 * (z * x - w * y));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}
