//
// Created by wzy on 2020/9/1.
//

#include "mission_core.h"

bool mission_core::auto_takeoff(){

    /**
     * acquire home_position and publish s stream of points
     */
    while(!got_attitude_init && ros::ok()){
        ROS_INFO("acquire home_yaw");
        ros::spinOnce();
        curr_euler = quaternion2euler(odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w);
        ROS_INFO("curr_yaw: %f", curr_euler.z);
        if(abs(curr_euler.z) > 0.0001){
            got_attitude_init = true;
        }
    }

    for (int i = 0; i < 100; ++i) {
        ROS_INFO("acquire home_position, %d", i);
        ros::spinOnce();
        px4_position_msg.pose.position.x += odom_msg.pose.pose.position.x;
        px4_position_msg.pose.position.y += odom_msg.pose.pose.position.y;
        px4_position_msg.pose.position.z += odom_msg.pose.pose.position.z;
        curr_euler = quaternion2euler(odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w);
        ROS_INFO("curr_yaw: %f", curr_euler.z);
        ROS_INFO("odom_msg.pose.pose.position.x: %f", odom_msg.pose.pose.position.x);
        ROS_INFO("odom_msg.pose.pose.position.y: %f", odom_msg.pose.pose.position.y);
        euler_sum += curr_euler.z;

        rate.sleep();
    }

    px4_position_msg.pose.position.x /= 100.0;
    px4_position_msg.pose.position.y /= 100.0;
    px4_position_msg.pose.position.z /= 100.0;
    curr_euler.z = euler_sum / 100.0;
    yaw_rotate = curr_euler.z;

    x_pos_home_origin = px4_position_msg.pose.position.x;
    y_pos_home_origin = px4_position_msg.pose.position.y;
    z_pos_home_origin = px4_position_msg.pose.position.z;

//    home_hover_height += z_pos_home_origin;
    px4_position_msg.header.stamp = ros::Time::now();
    px4_position_msg.pose.position.z = home_hover_height;
    takeoff_Quan = euler2quaternion(curr_euler.x,curr_euler.y,curr_euler.z);
    px4_position_msg.pose.orientation.x = takeoff_Quan.x;
    px4_position_msg.pose.orientation.y = takeoff_Quan.y;
    px4_position_msg.pose.orientation.z = takeoff_Quan.z;
    px4_position_msg.pose.orientation.w = takeoff_Quan.w;



    for (int j = 0; j < 100; ++j) {
        ROS_INFO("publish home_position setpoint, %d", j);
        px4_setpoint_position_pub.publish(px4_position_msg);
        rate.sleep();
    }

    //!车摆在飞机前3m处，测试时，计算车的home点在飞机home点的enu位置
    carToDroneDistance[0] = 3.0;
    carToDroneDistance[1] = 0.0;
    cs_road_to_enu(carToDroneDistance[0],carToDroneDistance[1],carToDroneDistanceEnu[0],carToDroneDistanceEnu[1]);
    //!计算pnp期望位置
    distance_car_drone[0] = 3.0;
    distance_car_drone[1] = 0.0;
    distance_car_drone[2] = 0.0;
    cs_road_to_enu( distance_car_drone[0], distance_car_drone[1], pnp_expect_pos[0], pnp_expect_pos[1]);
    pnp_expect_pos[2] = home_hover_height;
    /**
     * arm and offboard
     */
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        ros::spinOnce();
        if(current_state_msg.armed) {
            ROS_WARN("armed !");
        }else{
            ROS_INFO("not armed !");
        }

        if(current_state_msg.mode == "OFFBOARD")
        {
            ROS_WARN("OFFBOARD NOW !");
            ROS_WARN("ready to climb to home hover height");
            ROS_WARN("height now is %f", drone_pos_from_start[2]);
            ROS_WARN("require height is %f", home_hover_height);
            break;
        }




        // if( current_state_msg.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now(); 
        // }
        // else 
        // {
        //     if( !current_state_msg.armed  && (ros::Time::now() - last_request > ros::Duration(2.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }
        px4_position_msg.header.stamp = ros::Time::now();
        px4_position_msg.pose.position.x = curr_p[0];
        px4_position_msg.pose.position.y = curr_p[1];
        px4_position_msg.pose.position.z = curr_p[2];

//        px4_position_msg.pose.position.z =  home_hover_height + odom_msg.pose.pose.position.z - drone_pos_from_start[2];
        px4_setpoint_position_pub.publish(px4_position_msg);
        cout << "px4_position_msg.pose.position.x: " << px4_position_msg.pose.position.x << endl;

//        if (current_state_msg.mode == "OFFBOARD" && fabs(drone_pos_from_start[2] - home_hover_height) < 0.5 )
//        {
//            ROS_INFO("hover mode, height =  %f", odom_msg.pose.pose.position.z);
//            ROS_INFO("break hover mode, into yaw rotate mode");
//            break;
//
//        }

        


        rate.sleep();
    }

    return true;
}

bool mission_core::auto_takeoff_field(){

    /**
     * acquire home_position and publish s stream of points
     */
    while(!got_attitude_init && ros::ok()){
        ROS_INFO("acquire home_yaw");
        ros::spinOnce();
        curr_euler = quaternion2euler(odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w);
        ROS_INFO("curr_yaw: %f", curr_euler.z);
        if(abs(curr_euler.z) > 0.0001){
            got_attitude_init = true;
        }
    }

    for (int i = 0; i < 100; ++i) {
        ROS_INFO("acquire home_position, %d", i);
        ros::spinOnce();
        px4_position_msg.pose.position.x += odom_msg.pose.pose.position.x;
        px4_position_msg.pose.position.y += odom_msg.pose.pose.position.y;
        curr_euler = quaternion2euler(odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w);
        ROS_INFO("curr_yaw: %f", curr_euler.z);
        ROS_INFO("odom_msg.pose.pose.position.x: %f", odom_msg.pose.pose.position.x);
        ROS_INFO("odom_msg.pose.pose.position.y: %f", odom_msg.pose.pose.position.y);
        euler_sum += curr_euler.z;

        rate.sleep();
    }

    px4_position_msg.pose.position.x /= 100.0;
    px4_position_msg.pose.position.y /= 100.0;
    curr_euler.z = euler_sum / 100.0;
    yaw_rotate = curr_euler.z;

    x_pos_home_origin = px4_position_msg.pose.position.x;
    y_pos_home_origin = px4_position_msg.pose.position.y;

    px4_position_msg.header.stamp = ros::Time::now();
    px4_position_msg.pose.position.z = home_hover_height;
    takeoff_Quan = euler2quaternion(curr_euler.x,curr_euler.y,curr_euler.z);
    px4_position_msg.pose.orientation.x = takeoff_Quan.x;
    px4_position_msg.pose.orientation.y = takeoff_Quan.y;
    px4_position_msg.pose.orientation.z = takeoff_Quan.z;
    px4_position_msg.pose.orientation.w = takeoff_Quan.w;


    for (int j = 0; j < 100; ++j) {
        ROS_INFO("publish home_position setpoint, %d", j);
        px4_setpoint_position_pub.publish(px4_position_msg);
        rate.sleep();
    }

    //!车摆实际跑道上，计算车的home点在飞机home点的enu位置
    carToDroneDistance[0] = -8;
    carToDroneDistance[1] = 7.4;
    cs_road_to_enu(carToDroneDistance[0],carToDroneDistance[1],carToDroneDistanceEnu[0],carToDroneDistanceEnu[1]);
    //!计算pnp期望位置
    distance_car_drone[0] = 3.0;
    distance_car_drone[1] = 0.0;
    distance_car_drone[2] = 0.0;
    cs_road_to_enu( distance_car_drone[0], distance_car_drone[1], pnp_expect_pos[0], pnp_expect_pos[1]);
    pnp_expect_pos[2] = home_hover_height;
    /**
     * arm and offboard
     */
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        ros::spinOnce();
        if(current_state_msg.armed)
        {
            ROS_WARN("armed !");
            if(current_state_msg.mode == "OFFBOARD")
            {
                ROS_WARN("OFFBOARD NOW !");
                ROS_WARN("ready to climb to home hover height");
                ROS_WARN("height now is %f", odom_msg.pose.pose.position.z);
                ROS_WARN("require height is %f", home_hover_height);
            }

        }
        else
            ROS_INFO("not armed !");
        // if( current_state_msg.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // }
        // else
        // {
        //     if( !current_state_msg.armed  && (ros::Time::now() - last_request > ros::Duration(2.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }
        px4_position_msg.header.stamp = ros::Time::now();
        px4_setpoint_position_pub.publish(px4_position_msg);
        cout << "px4_position_msg.pose.position.x: " << px4_position_msg.pose.position.x << endl;

        if (current_state_msg.mode == "OFFBOARD" && fabs(odom_msg.pose.pose.position.z - home_hover_height) < 0.2 )
        {
            ROS_INFO("hover mode, height =  %f", odom_msg.pose.pose.position.z);
            ROS_INFO("break hover mode, into yaw rotate mode");
            break;

        }




        rate.sleep();
    }

    return true;
}

bool mission_core::manual_takeoff() {

    /**
* acquire home_position and publish s stream of points
*/
    while (!got_attitude_init && ros::ok()) {
        ROS_INFO("acquire home_yaw");
        ros::spinOnce();
        curr_euler = quaternion2euler(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                                      odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
        ROS_INFO("curr_yaw: %f", curr_euler.z);
        if (abs(curr_euler.z) > 0.0001) {
            got_attitude_init = true;
        }
    }

    for (int i = 0; i < 100; ++i) {
        ROS_INFO("acquire home_position, %d", i);
        ros::spinOnce();
        curr_euler = quaternion2euler(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                                      odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
        ROS_INFO("curr_yaw: %f", curr_euler.z);
        euler_sum += curr_euler.z;
        rate.sleep();
    }

    curr_euler.z = euler_sum / 100.0;
    yaw_rotate = curr_euler.z;


    while (ros::ok()) {
        ros::spinOnce();

        //yaw is prepared
        takeoff_Quan = euler2quaternion(curr_euler.x, curr_euler.y, curr_euler.z);
        px4_position_msg.pose.orientation.x = takeoff_Quan.x;
        px4_position_msg.pose.orientation.y = takeoff_Quan.y;
        px4_position_msg.pose.orientation.z = takeoff_Quan.z;
        px4_position_msg.pose.orientation.w = takeoff_Quan.w;

        x_pos_home_origin = curr_p[0];
        y_pos_home_origin = curr_p[1];
        home_hover_height = curr_p[2];
        px4_position_msg.pose.position.x = x_pos_home_origin;
        px4_position_msg.pose.position.y = y_pos_home_origin;
        px4_position_msg.pose.position.z = home_hover_height;


        //!车摆在飞机前3m处，测试时，计算车的home点在飞机home点的enu位置
        carToDroneDistance[0] = 3;
        carToDroneDistance[1] = 0;
        cs_road_to_enu(carToDroneDistance[0], carToDroneDistance[1], carToDroneDistanceEnu[0],
                       carToDroneDistanceEnu[1]);
        //!计算pnp期望位置
        distance_car_drone[0] = 3.0;
        distance_car_drone[1] = 0.0;
        distance_car_drone[2] = 0.0;
        cs_road_to_enu(distance_car_drone[0], distance_car_drone[1], pnp_expect_pos[0], pnp_expect_pos[1]);
        pnp_expect_pos[2] = home_hover_height;


        if (current_state_msg.armed) {
            ROS_WARN("armed !");
            if (current_state_msg.mode == "OFFBOARD") {
                ROS_WARN("OFFBOARD NOW !");
                ROS_WARN("ready to climb to home hover height");
                ROS_WARN("height now is %f", odom_msg.pose.pose.position.z);
                ROS_WARN("require height is %f", home_hover_height);
            }
            else{
                ROS_WARN("NOT OFFBOARD");
            }

        } else
            ROS_INFO("not armed !");
        // if( current_state_msg.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // }
        // else
        // {
        //     if( !current_state_msg.armed  && (ros::Time::now() - last_request > ros::Duration(2.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }
        px4_position_msg.header.stamp = ros::Time::now();
        px4_setpoint_position_pub.publish(px4_position_msg);
        cout << "px4_position_msg.pose.position.x: " << px4_position_msg.pose.position.x << endl;
        cout << "px4_position_msg.pose.position.y: " << px4_position_msg.pose.position.y << endl;
        cout << "px4_position_msg.pose.position.z: " << px4_position_msg.pose.position.z << endl;

        if (current_state_msg.mode == "OFFBOARD" && fabs(odom_msg.pose.pose.position.z - home_hover_height) < 0.2) {
            ROS_INFO("hover mode, height =  %f", odom_msg.pose.pose.position.z);
            ROS_INFO("break hover mode, into yaw rotate mode");
            break;
        }

    }
}

bool mission_core::yaw_rotate_process(){

    geometry_msgs::Vector3 curr_yaw_msg;
    std_msgs::Bool start_velocity_esti_flag;
    start_velocity_esti_flag.data = false;

    px4_position_msg.pose.orientation = euler2quaternion(0,0,yaw_rotate);
    px4_position_msg.pose.position.x = x_pos_home_origin;
    px4_position_msg.pose.position.y = y_pos_home_origin;
    px4_position_msg.pose.position.z = home_hover_height;

    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        px4_position_msg.header.stamp = ros::Time::now();
        px4_setpoint_position_pub.publish(px4_position_msg);
        curr_yaw_msg = quaternion2euler(odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w);
        ROS_WARN("yaw now: %f", curr_yaw_msg.z);
        ROS_INFO("yaw_rotate: %f", yaw_rotate);
        if (fabs(curr_yaw_msg.z - yaw_rotate) < 0.1)
        {
            start_velocity_esti_flag.data = true;
            start_trace_msg.data = true;
            ROS_INFO("reached yaw_target, wait for car over the start line");
//            start_velocity_esti_pub.publish(start_velocity_esti_flag);
//            start_trace_pub.publish(start_trace_msg);
            ROS_INFO("pub trace msg for %d", count);
            //get attitude init
            drone_euler_init = quaternion2euler(odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w);
            got_attitude_init = true;
            count += 1;
        }
        if (count > 5){
            ROS_WARN("stop pub trace msg");
            break;
        }
        rate.sleep();
    }
}

void mission_core::esti_car_velocity() {
//    x_velocity_start = car_pos_from_start[0] + carToDroneDistanceEnu[0] - length_of_velocity_esti*cos(yaw_rotate);
//    y_velocity_start = car_pos_from_start[1] + carToDroneDistanceEnu[1] - length_of_velocity_esti*sin(yaw_rotate);
//    bool have_started = false;
//    start_trace_msg.data = true;
//    std::chrono::time_point<std::chrono::steady_clock> start_t;
//    std::chrono::time_point<std::chrono::steady_clock> end_t;
//    while (ros::ok()){
//        ROS_INFO("start_vel_error_x: %f", x_velocity_start-car_pos[0]);
//        ROS_INFO("start_vel_error_y: %f", y_velocity_start-car_pos[1]);
//        ros::spinOnce();
//        start_trace_pub.publish(start_trace_msg);//小车开始向前跑
//        px4_setpoint_position_pub.publish(px4_position_msg); //不需要更新，继续按旧的值去发布
//        if (!have_started && fabs(x_velocity_start-car_pos[0]) < 1 && fabs(y_velocity_start - car_pos[1]) < 1) //当车辆抵达速度估计的起始点
//        {   ROS_INFO("start time calculating ... ");
//            start_t = chrono::steady_clock::now();
//            have_started = true;
//        }
//        if (have_started && fabs(x_pos_road_origin - car_pos[0]) < 1 && fabs(y_pos_road_origin - car_pos[1]) < 1) //当车辆抵达速度估计的终止点，即车辆起跑线
//        {   ROS_INFO("end time calculating ..., break ");
//            end_t = chrono::steady_clock::now();
//            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t);
//            vel_time = duration.count()/1000000.0;
//            car_esti_vel[0] = length_of_velocity_esti*cos(yaw_rotate)/vel_time;
//            cout << "vel_time: " << vel_time << endl;
//            cout << "car_esti_vel[0]: " << car_esti_vel[0] << endl;
//            car_esti_vel[1] = length_of_velocity_esti*sin(yaw_rotate)/vel_time;
//            break;
//        }
//        rate.sleep();
//    }
}