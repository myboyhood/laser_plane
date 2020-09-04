//
// Created by wzy on 2020/9/1.
//

#ifndef MISSION_MISSION_CORE_H
#define MISSION_MISSION_CORE_H

#include "mission/ros_related.h"
#include "mission/common_usage.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define LOOPRATE 30

class mission_core {
public:


    ros::NodeHandle nh;
    ros::Rate rate;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber state_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber car_pos_sub;
    ros::Subscriber start_trace_sub;


    ros::Publisher px4_setpoint_position_pub;
    ros::Publisher pva_setpoint_attitude_pub;
    ros::Publisher pva_odom_sp_enu_pub;
    ros::Publisher start_velocity_esti_pub;
    /**
    * service / topic message
    */
    mavros_msgs::State current_state_msg;
    nav_msgs::Odometry odom_msg;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    nav_msgs::Odometry odom_sp_enu;




    /**
     * variable
     */
     //auto_takeoff
    float home_hover_height = 3.0;
    float yaw_rotate = 0.5;
    Eigen::Vector3d curr_p;
    Eigen::Vector3d curr_v;
    Eigen::Vector3d car_pos;
    Eigen::Vector2d car_esti_vel;
    double vel_time = 5;
    bool yaw_finished_flag = false;
    bool start_trace_flag = false;
    float x_pos_road_origin = -1.0; // 车辆起跑点，在飞机enu坐标系下的位置
    float y_pos_road_origin = 5.0;
    float x_velocity_start = 0.0;   // 估计车辆速度的起始点，在飞机enu坐标系下的位置
    float y_velocity_start = 0.0;
    float length_of_velocity_esti = 10.0; //用来估计车速的长度
    float x_pos_home_origin = 0.0;
    float y_pos_home_origin = 0.0;
    float z_pos_home_origin = 0.0;


    //pva
    bool get_traj = false;
    /* param */
    Eigen::Vector3d pp;
    Eigen::Vector3d pi;
    Eigen::Vector3d pd;

    Eigen::Vector3d vp;
    Eigen::Vector3d vi;
    Eigen::Vector3d vd;
    float distance_car_drone = 3.0; //默认车后3m位置是飞机期望位置

    //px4_traj_pose_control
    bool is_back_flag = false; //判断是否为返程中使用px4_traj_pose_control，正常追踪过程中默认为false
    int xyz_step_count = 1; //规划轨迹中，每次循环都规划新的轨迹，但是每条轨迹都是10个点，保证远距离快速追踪，近距离慢速跟进。
    geometry_msgs::PoseStamped px4_position_msg; //用于发布位置信息
    mavros_msgs::AttitudeTarget att_setpoint; //用于发布pva信息
    double step_x;
    double step_y;
    double step_z;
    std::vector<double> x_waypoints;
    std::vector<double> y_waypoints;
    std::vector<double> z_waypoints;
    int waypoint_count;
    Eigen::Vector3d car_pos_bias; //室外的飞机起飞点不是原点时，需要把x_pos_home_origin和y_pos_home_origin的偏置量加到car_pos,
                                    //仅仅是单独飞轨迹时使用，当真正使用车辆GPS数据估计速度时，还是要把飞机从原点（0,0,0）起飞


    //homeward_voyage
    int maxtime = 150;
    int timecount_for_backhome = 0;


    /**
     * function
     */
    mission_core();
    void cs_road_to_enu(float &x_road,float &y_road,float &x_enu,float &y_enu);
    geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
    geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);

    //pva
    bool readParam(const char* addr);
    bool setParam();
    inline double max3_double(double a, double b, double c);
    void motion_primitives(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0,
                           Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Vector3d af,
                           double T, double delt_t, int times,
                           Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t,
                           Eigen::Vector3d &max_v, Eigen::Vector3d &max_a);
    void compute_tracking_traj(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0,
                               Eigen::Vector3d goal_p0, Eigen::Vector3d goal_v0, Eigen::Vector3d goal_a0,
                               double v_limit, double a_limit, double j_limit, double delt_T, double delt_t,
                               Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t);

    void pva_tracker_control();
    void setPVA(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw = 0.0);
    void pva_calculate_and_pub_traj(Eigen::Vector3d goal_p,Eigen::Vector3d goal_v,Eigen::Vector3d goal_a, double init_T = 5);
    void pvaTrack(trajectory_msgs::JointTrajectoryPoint &msg);

    //px4_traj
    void px4_waypoint_calculate(Eigen::Vector3d start_p, Eigen::Vector3d end_p);
    void px4_traj_track_control();

    //homeward_voyage
    void backhome();


    /**
     * callback function
     */
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void car_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void start_trace_cb(const std_msgs::Bool::ConstPtr& msg);
    /**
     * process function
     */
    bool auto_takeoff_gazebo();
    bool yaw_rotate_process();
    void esti_car_velocity();//计算起跑点x轴方向负10m的位置点,并估计车辆在ENU坐标系下的x和y方向的速度。
};


#endif //MISSION_MISSION_CORE_H
