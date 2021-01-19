//
// Created by wzy on 2020/9/1.
//
#include <mission_core.h>
using namespace Eigen;

Vector3d delta_xyz;

void mission_core::px4_waypoint_calculate(Vector3d start_p,Vector3d end_p){
    waypoint_count = 0;
    step_x = (end_p[0] - start_p[0])/xyz_step_count*1.0;
    step_y = (end_p[1] - start_p[1])/xyz_step_count*1.0;
    step_z = (end_p[2] - start_p[2])/xyz_step_count*1.0;
    x_waypoints.clear();
    y_waypoints.clear();
    z_waypoints.clear();
    for (int i = 1; i <= xyz_step_count; ++i) {
        x_waypoints.push_back(start_p[0] + i*step_x);
        y_waypoints.push_back(start_p[1] + i*step_y);
        z_waypoints.push_back(start_p[2] + i*step_z);
    }

}

void mission_core::px4_traj_track_control(bool with_time_back_){

    // if(with_time_back_)
    // {
    //     //decide number of timecount_for_backhome
    //     if (get_traj){
    //         timecount_for_backhome = 0;
    //     }
    //     else{
    //         timecount_for_backhome = -150;
    //     }

    // }

    while(ros::ok()){
        ros::spinOnce();

        // if(with_time_back_){
        //     //add timecount_for_backhome for break the track while
        //     timecount_for_backhome += 1;
        //     ROS_WARN("timecount_for_backhome: %d", timecount_for_backhome);
        //     if (timecount_for_backhome > maxtime)
        //         break;
        // }
        px4_target_x = car_pos[0]-distance_car_drone[0]*cos(yaw_rotate) + x_pos_home_origin;
        px4_target_y = car_pos[1]-distance_car_drone[1]*sin(yaw_rotate) + y_pos_home_origin;
        ROS_WARN("x_pos_planehome_origin: %f", x_pos_home_origin);
        ROS_WARN("y_pos_planehome_origin: %f", y_pos_home_origin);
        ROS_WARN("car_pos0: %f",car_pos[0]);
        ROS_WARN("car_pos1: %f", car_pos[1]);
        ROS_WARN("px4 target x : %f" , px4_target_x);
        ROS_WARN("px4 target y : %f",  px4_target_y);
        ROS_WARN("odom curr x: %f", odom_msg.pose.pose.position.x);
        ROS_WARN("odom curr y: %f", odom_msg.pose.pose.position.y);
        px4_waypoint_calculate(curr_p, Vector3d(px4_target_x,px4_target_y,home_hover_height));
        if(fabs(x_waypoints[waypoint_count]-curr_p[0]) > 0.2 || fabs(y_waypoints[waypoint_count]-curr_p[1]) > 0.2 || fabs(z_waypoints[waypoint_count]-curr_p[2]) > 0.2)
        {
            ROS_INFO("reaching the %d trajectory point", waypoint_count);
            // ROS_INFO("3m_after_car_pos_x: %f", car_pos[0]);
            // ROS_INFO("3m_after_car_pos_y: %f", car_pos[1]);
            // ROS_INFO("3m_after_car_pos_z: %f", car_pos[2]);

            px4_position_msg.pose.position.x = x_waypoints[waypoint_count];
            px4_position_msg.pose.position.y = y_waypoints[waypoint_count];
            px4_position_msg.pose.position.z = z_waypoints[waypoint_count];
            // ROS_INFO("px4_position_msg.pose.position.x: %f",);
            // ROS_INFO("px4_position_msg.pose.position.y: %f",px4_position_msg.pose.position.y);
            // ROS_INFO("px4_position_msg.pose.position.z: %f",px4_position_msg.pose.position.z);
            ROS_WARN("error_x: %f",px4_position_msg.pose.position.x - odom_msg.pose.pose.position.x);
            ROS_WARN("error_y: %f",px4_position_msg.pose.position.y - odom_msg.pose.pose.position.y);
        }
        else{
            if (waypoint_count < (xyz_step_count-1)){
                waypoint_count += 1;
            }
            px4_position_msg.pose.position.x = x_waypoints[waypoint_count];
            px4_position_msg.pose.position.y = y_waypoints[waypoint_count];
            px4_position_msg.pose.position.z = z_waypoints[waypoint_count];
        }
        ROS_WARN("px4_position_msg.pose.position.x: %f",px4_position_msg.pose.position.x);
        ROS_WARN("px4_position_msg.pose.position.y: %f",px4_position_msg.pose.position.y);
        ROS_WARN("px4_position_msg.pose.position.z: %f",px4_position_msg.pose.position.z);
        px4_setpoint_position_pub.publish(px4_position_msg);

        rate.sleep();
    }
}