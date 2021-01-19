//
// Created by wzy on 2020/9/1.
//
#include <mission_core.h>
using namespace Eigen;


void mission_core::ocp_vision_follow_control(){

    /**
     * 第一次pnp结果会和GPS拿到的local_pos结果有较大差别。用 first_large_bias 一次性加到 gps_target_pos 上
     * 之后如果 pnp_target_pos 和 gps_target_pos 差别不大，就用 small_bias 去加到 gps_target_pos 上， pnp_gps_fuse_target_pos 使用 pnp_target_pos
     * 如果差别很大，那么认为 pnp_target_pos 错误， pnp_gps_fuse_target_pos 使用 gps_target_pos
     */

    /**
     * pnp 是相对坐标系，用来矫正车和飞机的local_positioin的漂移,高度z不做矫正，只使用激光定高,此处 gps_target_pos == gps_expect_pos
     * pnp_error = pnp_expect_pos - pnp_curr_pos , gps_error = gps_expect_pos - gps_curr_pos
     * pnp_gps_bias = pnp_error - gps_error
     * gps_expect_pos_fuse = gps_expect_pos + pnp_gps_bias
     * px4_position_msg = gps_expect_pos_fuse + xyz_pos_home_origin
     * publish(px4_position_msg)
     */

    while(ros::ok()){
        ros::spinOnce();


        cout << "car_pos_from_start[0]: " << car_pos_from_start[0] << endl;
        cout << "car_pos_from_start[1]: " << car_pos_from_start[1] << endl;
        cout << "carToDroneDistanceEnu[0]: " << carToDroneDistanceEnu[0] << endl;
        cout << "distance_car_drone[0] * cos(yaw_rotate): " << distance_car_drone[0] * cos(yaw_rotate) << endl;
        gps_target_pos = Vector3d(car_pos_from_start[0] + carToDroneDistanceEnu[0] - distance_car_drone[0] * cos(yaw_rotate),car_pos_from_start[1] + carToDroneDistanceEnu[1] - distance_car_drone[1] * sin(yaw_rotate),home_hover_height);
        car_curr_pos[0] = car_pos_from_start[0] + carToDroneDistanceEnu[0];
        car_curr_pos[1] = car_pos_from_start[1] + carToDroneDistanceEnu[1];
        cout << "car_pos_from_start[1]: " << car_pos_from_start[1] << "carToDroneDistanceEnu[1]" << carToDroneDistanceEnu[1] << endl;
        cout << "66666666666666666666666666666666666 pnp_z: " << pnp_z << endl;
        relative_pose_msg.pose.orientation.z = pnp_z;
        if(pnp_pose_is_good){
            ROS_WARN("///////////////-----pnp_pose_is_good-----////////////////////");
            pnp_pos_error[0] = - pnp_expect_pos[0] - drone_pos_vision_in_enu[0];
            pnp_pos_error[1] = - pnp_expect_pos[1] - drone_pos_vision_in_enu[1];

            gps_pos_error[0] = gps_target_pos[0] - drone_pos_from_start[0];
            gps_pos_error[1] = gps_target_pos[1] - drone_pos_from_start[1];
            //lowpass filter
            pnp_gps_bias[0] = pnp_gps_bias_prev[0] +  0.1*((pnp_pos_error[0] - gps_pos_error[0]) - pnp_gps_bias_prev[0]);
            pnp_gps_bias[1] = pnp_gps_bias_prev[1] +  0.1*((pnp_pos_error[1] - gps_pos_error[1]) - pnp_gps_bias_prev[1]);

            pnp_gps_bias_prev[0] = pnp_gps_bias[0] ;
            pnp_gps_bias_prev[1] = pnp_gps_bias[1] ;

            relative_pose_msg.header.stamp = ros::Time::now();
            relative_pose_msg.pose.position.x = drone_pos_vision_in_enu[0];
            relative_pose_msg.pose.position.y = drone_pos_vision_in_enu[1];
            relative_pose_msg.pose.position.z = drone_pos_from_start[2] - target_height;

            drone_pos_rect_from_start_prev[0] = relative_pose_msg.pose.position.x;
            drone_pos_rect_from_start_prev[1] = relative_pose_msg.pose.position.y;
            drone_pos_rect_from_start_prev[2] = relative_pose_msg.pose.position.z;
            pnp_last_time = relative_pose_msg.header.stamp.toSec();
        }
        else{
            cout << "----------------drone_pos_rect_from_start[0]:" << drone_pos_rect_from_start[0] << endl;
                drone_pos_rect_from_start[0] = drone_pos_from_start[0] - pnp_gps_bias[0];
                drone_pos_rect_from_start[1] = drone_pos_from_start[1] - pnp_gps_bias[1];
                if(ros::Time::now().toSec() - pnp_last_time > 0.4){
                    gps_relative_pose_msg.header.stamp = ros::Time::now();
                    gps_relative_pose_msg.pose.position.x = drone_pos_from_start[0] - car_curr_pos[0];
                    gps_relative_pose_msg.pose.position.y = drone_pos_from_start[1] - car_curr_pos[1];
                    gps_relative_pose_msg.pose.position.z = drone_pos_from_start[2] - target_height;

                    relative_pose_msg.header.stamp = ros::Time::now();
                    cout << "----------------drone_pos_rect_from_start[0]:" << drone_pos_rect_from_start[0] << endl;
                    cout << "----------------car_curr_pos[0]:" << car_curr_pos[0] << endl;
                    relative_pose_msg.pose.position.x = drone_pos_rect_from_start[0] - car_curr_pos[0];
                    cout << "--------------relative_pose_msg.pose.position.x :" << relative_pose_msg.pose.position.x  << endl;
                    relative_pose_msg.pose.position.y = drone_pos_rect_from_start[1] - car_curr_pos[1];
                    relative_pose_msg.pose.position.z = drone_pos_from_start[2] - target_height;
                }
                else{
                    relative_pose_msg.header.stamp = ros::Time::now();
                    relative_pose_msg.pose.position.x = drone_pos_rect_from_start_prev[0] ;
                    relative_pose_msg.pose.position.y = drone_pos_rect_from_start_prev[1];
                    relative_pose_msg.pose.position.z = drone_pos_rect_from_start_prev[2];
                }


        }




        cout << "drone_pos_from_start[0]: " << drone_pos_from_start[0] << endl;
        cout << "drone_pos_from_start[1]: " << drone_pos_from_start[1] << endl;
        cout << "drone_pos_rect_from_start[0]: " << drone_pos_rect_from_start[0] << endl;
        cout << "drone_pos_rect_from_start[1]: " << drone_pos_rect_from_start[1] << endl;
        cout << "car_curr_pos[0]: " << car_curr_pos[0] << endl;
        cout << "car_curr_pos[1]: " << car_curr_pos[1] << endl;
        cout << "relative_pose_msg.pose.position.x: " << relative_pose_msg.pose.position.x << endl;
        cout << "relative_pose_msg.pose.position.y: " << relative_pose_msg.pose.position.y << endl;
        cout << "pnp_expect_pos[0]: " << pnp_expect_pos[0] << endl;
        cout << "pnp_expect_pos[1]: " << pnp_expect_pos[1] << endl;
        cout << "drone_pos_vision_in_enu[0]: " << drone_pos_vision_in_enu[0] << endl;
        cout << "drone_pos_vision_in_enu[1]: " << drone_pos_vision_in_enu[1] << endl;
        cout << "pnp_pos_error[0]: " << pnp_pos_error[0] << endl;
        cout << "pnp_pos_error[1]: " << pnp_pos_error[1] << endl;
        cout << "gps_pos_error[0]: " << gps_pos_error[0] << endl;
        cout << "gps_pos_error[1]: " << gps_pos_error[1] << endl;
        cout << "pnp_gps_bias[0]: " << pnp_gps_bias[0] << endl;
        cout << "pnp_gps_bias[1]: " << pnp_gps_bias[1] << endl;
        cout << "gps_target_pos[0]: " << gps_target_pos[0] << endl;
        cout << "gps_target_pos[1]: " << gps_target_pos[1] << endl;
//        cout << "px4_position_msg.pose.position.x: " << px4_position_msg.pose.position.x << endl;
//        cout << "px4_position_msg.pose.position.y: " << px4_position_msg.pose.position.y << endl;

//        pnp_gps_bias[0] = min(max(pnp_gps_bias[0],-1.0),1.0);
//        pnp_gps_bias[1] = min(max(pnp_gps_bias[1],-1.0),1.0);
//
//        pnp_gps_fuse_pos[0] = gps_target_pos[0] + pnp_gps_bias[0];
//        pnp_gps_fuse_pos[1] = gps_target_pos[1] + pnp_gps_bias[1];


        cout << "x_pos_home_origin: " << x_pos_home_origin << endl;
        cout << "y_pos_home_origin: " << y_pos_home_origin << endl;

        cout << "curr_p[0]: " << curr_p[0] << endl;
        cout << "curr_p[1] " << curr_p[1] << endl;
        cout << "curr_p[2] " << curr_p[2] << endl;

        //!publish for debug
        pnp_expect_pos_msg.header.stamp = ros::Time::now();
        pnp_expect_pos_msg.pose.position.x = pnp_expect_pos[0];
        pnp_expect_pos_msg.pose.position.y = pnp_expect_pos[1];

        gps_target_pos_msg.header.stamp = ros::Time::now();
        gps_target_pos_msg.pose.position.x = gps_target_pos[0];
        gps_target_pos_msg.pose.position.y = gps_target_pos[1];

        pnp_gps_bias_msg.header.stamp = ros::Time::now();
        pnp_gps_bias_msg.pose.position.x = pnp_gps_bias[0];
        pnp_gps_bias_msg.pose.position.y = pnp_gps_bias[1];

        plane_pos_from_start_msg.header.stamp = ros::Time::now();
        plane_pos_from_start_msg.pose.position.x = drone_pos_from_start[0];
        plane_pos_from_start_msg.pose.position.y = drone_pos_from_start[1];
        plane_pos_from_start_msg.pose.position.z = drone_pos_from_start[2];

        car_pos_from_planeHome_msg.header.stamp = ros::Time::now();
        car_pos_from_planeHome_msg.pose.pose.position.x = car_curr_pos[0];
        car_pos_from_planeHome_msg.pose.pose.position.y = car_curr_pos[1];
        car_pos_from_planeHome_msg.pose.pose.position.z = target_height;
        car_pos_from_planeHome_msg.twist.twist.linear.x = car_vel_from_start[0];
        car_pos_from_planeHome_msg.twist.twist.linear.y = car_vel_from_start[1];
        cout << "car_pos_from_planeHome_msg.twist.twist.linear.x: " << car_pos_from_planeHome_msg.twist.twist.linear.x << endl;
        cout << "car_pos_from_planeHome_msg.twist.twist.linear.y: " << car_pos_from_planeHome_msg.twist.twist.linear.y << endl;

//        ROS_WARN("ocp_control_msg.orientation.x: %f",ocp_control_msg.orientation.x);
//        ROS_WARN("ocp_control_msg.orientation.y: %f",ocp_control_msg.orientation.y);
//        ROS_WARN("ocp_control_msg.orientation.z: %f",ocp_control_msg.orientation.z);
//        ROS_WARN("ocp_control_msg.orientation.w: %f",ocp_control_msg.orientation.w);
        ROS_WARN("ocp_control_msg.thrust: %f",ocp_control_msg.thrust);
        ocp_euler_angle =  quaternion2euler(ocp_control_msg.orientation.x, ocp_control_msg.orientation.y, ocp_control_msg.orientation.z, ocp_control_msg.orientation.w);
        ROS_WARN("ocp_euler_angle roll : %f", ocp_euler_angle.x);
        ROS_WARN("ocp_euler_angle pitch : %f", ocp_euler_angle.y);
        ROS_WARN("ocp_euler_angle yaw : %f", ocp_euler_angle.z);
        ocp_control_pub.publish(ocp_control_msg);


        plane_pos_from_start_pub.publish(plane_pos_from_start_msg);
        car_pos_from_planeHome_pub.publish(car_pos_from_planeHome_msg);
        pnp_expect_pos_pub.publish(pnp_expect_pos_msg);
        drone_pos_vision_in_enu_pub.publish(drone_pos_vision_in_enu_msg);
        gps_target_pos_pub.publish(gps_target_pos_msg);
        pnp_gps_bias_pub.publish(pnp_gps_bias_msg);

        cout << "66666666666666666666666666666666666 relative_pose_msg: " << relative_pose_msg.pose.orientation.z << endl;
        relative_pose_pub.publish(relative_pose_msg);



        rate.sleep();
    

    }
}