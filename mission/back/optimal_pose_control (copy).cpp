//
// Created by wzy on 2020/9/1.
//
#include <mission_core.h>
bool mission_core::ocp_control(){
    geometry_msgs::Vector3 ocp_euler_angle;
    while(ros::ok())
    {
        ros::spinOnce();
        plane_pos_from_start_msg.header.stamp = ros::Time::now();
        plane_pos_from_start_msg.pose.position.x = odom_msg.pose.pose.position.x - x_pos_home_origin;
        plane_pos_from_start_msg.pose.position.y = odom_msg.pose.pose.position.y - y_pos_home_origin;
        plane_pos_from_start_msg.pose.position.z = odom_msg.pose.pose.position.z - z_pos_home_origin;
        plane_pos_from_start_pub.publish(plane_pos_from_start_msg);
        ROS_WARN("odom_msg.pose.pose.position.x: %f", odom_msg.pose.pose.position.x);
        ROS_WARN("plane_pos_from_start_msg.pose.position.x: %f", plane_pos_from_start_msg.pose.position.x);
        ROS_WARN("pub plane_pos_from_start_msg ... ");
        // if(ocp_control_msg.type_mask)
        // {
            ROS_WARN("ocp_control_msg.orientation.x: %f",ocp_control_msg.orientation.x);
            ROS_WARN("ocp_control_msg.orientation.y: %f",ocp_control_msg.orientation.y);
            ROS_WARN("ocp_control_msg.orientation.z: %f",ocp_control_msg.orientation.z);
            ROS_WARN("ocp_control_msg.orientation.w: %f",ocp_control_msg.orientation.w);
            ROS_WARN("ocp_control_msg.thrust: %f",ocp_control_msg.thrust);
            ocp_euler_angle =  quaternion2euler(ocp_control_msg.orientation.x, ocp_control_msg.orientation.y, ocp_control_msg.orientation.z, ocp_control_msg.orientation.w);
            ROS_WARN("ocp_euler_angle roll : %f", ocp_euler_angle.x);
            ROS_WARN("ocp_euler_angle pitch : %f", ocp_euler_angle.y);
            ROS_WARN("ocp_euler_angle yaw : %f", ocp_euler_angle.z);
            ocp_control_pub.publish(ocp_control_msg);
            // record_failsafe_flag = false;
        // }
        // else
        // {
        //     ROS_ERROR("into failsafe mode, use curr_pos to hover");
        //     if(!record_failsafe_flag)
        //     {
        //         record_failsafe_flag = true;
        //         failsafe_pos_msg.pose.position.x = curr_p[0];
        //         failsafe_pos_msg.pose.position.y = curr_p[1];
        //         failsafe_pos_msg.pose.position.z = curr_p[2];
        //     }
        //     px4_position_msg.pose.orientation.x = failsafe_pos_msg.pose.position.x;
        //     px4_position_msg.pose.orientation.y = failsafe_pos_msg.pose.position.y;
        //     px4_position_msg.pose.orientation.z = failsafe_pos_msg.pose.position.z;
        //     ROS_WARN("px4_position_msg.pose.orientation.x: %f", px4_position_msg.pose.orientation.x);
        //     ROS_WARN("px4_position_msg.pose.orientation.y: %f", px4_position_msg.pose.orientation.y);
        //     ROS_WARN("px4_position_msg.pose.orientation.z: %f", px4_position_msg.pose.orientation.z);


        // }


        rate.sleep();
    }
}
