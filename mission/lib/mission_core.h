//
// Created by wzy on 2020/9/1.
//

#ifndef MISSION_MISSION_CORE_H
#define MISSION_MISSION_CORE_H

#include "mission/ros_related.h"
#include "mission/common_usage.h"
#include "mission/vision.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <mavros_msgs/Altitude.h>//lss

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
    ros::Subscriber img_ir_sub;
    ros::Subscriber img_bgr_sub;
    ros::Subscriber ocp_control_sub;
    ros::Subscriber drone_pos_vision_sub;
    ros::Subscriber plane_height_sub;

    ros::Publisher px4_setpoint_position_pub;
    ros::Publisher pva_setpoint_attitude_pub;
    ros::Publisher pva_odom_sp_enu_pub;
    ros::Publisher start_velocity_esti_pub;
    ros::Publisher start_trace_pub;
    ros::Publisher plane_pos_from_start_pub;
    ros::Publisher ocp_control_pub;
    ros::Publisher pnp_expect_pos_pub;
    ros::Publisher drone_pos_vision_in_enu_pub;
    ros::Publisher gps_target_pos_pub;
    ros::Publisher pnp_gps_bias_pub;
    ros::Publisher relative_pose_pub;
    ros::Publisher car_pos_from_planeHome_pub;


    //test publisher
    ros::Publisher drone_euler_pub;
    ros::Publisher drone_vision_pos_pub;

    /**
    * service / topic message
    */
    mavros_msgs::State current_state_msg{};
    nav_msgs::Odometry odom_msg;
    nav_msgs::Odometry car_pos_vel_msg;
    geometry_msgs::PoseStamped plane_pos_from_start_msg;
    nav_msgs::Odometry car_pos_from_planeHome_msg;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    nav_msgs::Odometry odom_sp_enu;
    std_msgs::Bool start_trace_msg;



    /**
     * variable
     */
     //auto_takeoff
    float home_hover_height = 1.2;
    double yaw_rotate = 0.0;
//    float yaw_rotate = -0.32;
    // float yaw_rotate = 2.66;
    geometry_msgs::Vector3 curr_euler;
    float euler_sum = 0.0;
    geometry_msgs::Quaternion takeoff_Quan;
    Eigen::Vector3d curr_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d drone_pos_from_start= Eigen::Vector3d::Zero();
    Eigen::Vector3d drone_pos_rect_from_start= Eigen::Vector3d::Zero();
    Eigen::Vector3d drone_pos_rect_from_start_prev= Eigen::Vector3d::Zero();
    Eigen::Vector3d curr_v= Eigen::Vector3d::Zero();
    Eigen::Vector3d car_pos= Eigen::Vector3d::Zero();
    Eigen::Vector3d car_pos_rect= Eigen::Vector3d::Zero();
    Eigen::Vector3d car_vel_rect= Eigen::Vector3d::Zero();

    double car_bias_yaw = 0.0;
    double car_bias_yaw_const = 0.0;
    int car_bias_count = 0;
    bool car_bias_yaw_flag = false;
    Eigen::Vector3d car_pos_from_start= Eigen::Vector3d::Zero();
    Eigen::Vector3d car_vel_from_start= Eigen::Vector3d::Zero();
    Eigen::Vector3d carHome= Eigen::Vector3d::Zero();
    Eigen::Vector3d car_curr_pos= Eigen::Vector3d::Zero();
    double pnp_last_time = 0.0;
    double plane_height_enu=0.0;//lss


    Eigen::Vector2d car_esti_vel = Eigen::Vector2d::Zero();
    double vel_time = 5;
    bool getCarHomeFlag = false;
    bool yaw_finished_flag = false;
    bool start_trace_flag = false;
    Eigen::Vector2d carToDroneDistance = Eigen::Vector2d::Zero(); // 在road坐标系下,以飞机home点为原点，车辆home点的位置
    Eigen::Vector2d carHomeToDroneHome = Eigen::Vector2d::Zero(); // 在road坐标系下,以飞机home点为原点，车辆home点的位置
    Eigen::Vector2d carToDroneDistanceEnu = Eigen::Vector2d::Zero(); // 在enu坐标系下,以飞机home点为原点，车辆home点的位置
    float x_velocity_start = 0.0;   // 估计车辆速度的起始点，在飞机enu坐标系下的位置
    float y_velocity_start = 0.0;
    float length_of_velocity_esti = 10.0; //用来估计车速的长度
    float x_pos_home_origin = 0.0;
    float y_pos_home_origin = 0.0;
    float z_pos_home_origin = 0.0;


    //pva
    bool get_traj = false;
    /* param */
    Eigen::Vector3d pp = Eigen::Vector3d::Zero();
    Eigen::Vector3d pi = Eigen::Vector3d::Zero();
    Eigen::Vector3d pd = Eigen::Vector3d::Zero();

    Eigen::Vector3d vp = Eigen::Vector3d::Zero();
    Eigen::Vector3d vi = Eigen::Vector3d::Zero();
    Eigen::Vector3d vd = Eigen::Vector3d::Zero();
    Eigen::Vector3d tf_camera_drone = Eigen::Vector3d::Zero();

    Eigen::Vector3d distance_car_drone = Eigen::Vector3d::Zero(); //默认车后3m位置是飞机期望位置

    //px4_traj_pmission_node_outdoor_with_pnp_hoverose_control
    bool is_back_flag = false; //判断是否为返程中使用px4_traj_pose_control，正常追踪过程中默认为false
    int xyz_step_count = 1; //规划轨迹中，每次循环都规划新的轨迹，但是每条轨迹都是10个点，保证远距离快速追踪，近距离慢速跟进。
    geometry_msgs::PoseStamped px4_position_msg; //用于发布位置信息
    mavros_msgs::AttitudeTarget att_setpoint; //用于发布pva信息
    double step_x = 0;
    double step_y = 0;
    double step_z = 0;
    std::vector<double> x_waypoints;
    std::vector<double> y_waypoints;
    std::vector<double> z_waypoints;
    int waypoint_count = 0;
    double px4_target_x = 0;
    double px4_target_y = 0;
//    Eigen::Vector3d car_pos_bias; //室外的飞机起飞点不是原点时，需要把x_pos_home_origin和y_pos_home_origin的偏置量加到car_pos,
//                                    //仅仅是单独飞轨迹时使用，当真正使用车辆GPS数据估计速度时，还是要把飞机从原点（0,0,0）起飞

    //pnp
    cv_bridge::CvImagePtr img_ir_ptr;
    cv_bridge::CvImagePtr img_bgr_ptr;
    cv::Mat img_ir;
    cv::Mat img_bgr;
    cv::Mat cameraMatrix;
    vector<double> distCoeffs;
    vector<cv::Point3f> marker_struct;
    Eigen::Isometry3d tf_image_to_enu;
    Eigen::Isometry3d tf_camera_to_drone;
    Eigen::Isometry3d tf_drone_to_world;

    cv::Mat ir_binary;
    cv::Mat ir_erode;
    cv::Mat ir_dilate;
    vector<vector<cv::Point>> ir_contours;
    vector<cv::Point2d> marker_pixels;
    cv::Point2d marker_sum;
    cv::Point2d marker_center;
    cv::Point2d left_up,left_down,right_up,right_down;
    vector<cv::Point2d> marker_sequence_pixel;
    cv::Mat ir_color;
    cv::Vec3d outputRvecRaw,outputTvecRaw;
    Eigen::Vector4d target_position_of_cam;
    Eigen::Vector4d target_position_of_drone;
    Eigen::Vector4d target_position_of_world;
    geometry_msgs::Vector3 drone_euler;
    geometry_msgs::Vector3 drone_euler_init;
    Eigen::Quaterniond drone_quaternion;

    //vision gps fuse
    geometry_msgs::PoseStamped drone_pos_vision_in_enu_msg;
    geometry_msgs::PoseStamped pnp_expect_pos_msg;
    Eigen::Vector3d drone_pos_vision = Eigen::Vector3d::Zero();
    double pnp_z = 0.0;
    Eigen::Vector3d drone_pos_vision_in_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d pnp_target_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d pnp_expect_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d pnp_pos_error = Eigen::Vector3d::Zero();

    double target_height = 3.6;
    geometry_msgs::PoseStamped relative_pose_msg;
    geometry_msgs::PoseStamped gps_relative_pose_msg;
    geometry_msgs::PoseStamped gps_target_pos_msg;
    Eigen::Vector3d gps_target_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d gps_pos_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d pnp_gps_fuse_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d pnp_gps_fuse_target_pos = Eigen::Vector3d::Zero();

    geometry_msgs::PoseStamped pnp_gps_bias_msg;
    Eigen::Vector3d pnp_gps_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d pnp_gps_bias_prev = Eigen::Vector3d::Zero();
    bool first_bias_flag = true;
    Eigen::Vector3d first_large_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d small_bias = Eigen::Vector3d::Zero();
    geometry_msgs::PoseStamped drone_pos_vision_msg;
    bool got_attitude_init = false;
    bool vision_prepare_ok = false;
    bool pnp_pose_is_good = false;

    //ocp
    mavros_msgs::AttitudeTarget ocp_control_msg;
    geometry_msgs::Vector3 ocp_euler_angle;
    geometry_msgs::PoseStamped failsafe_pos_msg;
    bool record_failsafe_flag = false;

    geometry_msgs::PoseStamped drone_euler_msg;
    //homeward_voyage
    int maxtime = 150;
    int timecount_for_backhome = 0;


    /**
     * function
     */
    mission_core();
    void cs_road_to_enu(double &x_road,double &y_road,double &x_enu,double &y_enu);
    void cs_enu_to_road(double &x_enu,double &y_enu,double &x_road,double &y_road);
    geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
    Eigen::Quaterniond euler2quaternion_eigen(float roll, float pitch, float yaw);
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
    void px4_traj_track_control(bool with_time_back);
    void trace_to_back();


    //pnp
    bool pnp_follow_control_prepare();
    void camera_param_set();
    void tf_param_set();
    bool vision_pose_acqurie();
    void vision_gps_pose_control();
    void get_init_yaw();
    void px4_traj_vision_follow_control();
    void ocp_vision_follow_control();

    //ocp
    bool ocp_control();

    //homeward_voyage
    void backhome();


    /**
     * callback function
     */
    void drone_pos_vision_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void car_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void start_trace_cb(const std_msgs::Bool::ConstPtr& msg);
//    void img_ir_cb(const sensor_msgs::Image::ConstPtr& msg);
//    void img_bgr_cb(const sensor_msgs::Image::ConstPtr& msg);
    void ocp_control_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    /**
     * process function
     */
    void plane_altitude_enu_cb(const mavros_msgs::Altitude::ConstPtr &msg);//lss
    bool auto_takeoff();
    bool auto_takeoff_field();
    bool manual_takeoff();
    bool yaw_rotate_process();
    void esti_car_velocity();//计算起跑点x轴方向负10m的位置点,并估计车辆在ENU坐标系下的x和y方向的速度。
    void cs_to_enu(double &x_road,double &y_road,double &x_enu,double &y_enu, double yaw);
};


#endif //MISSION_MISSION_CORE_H
