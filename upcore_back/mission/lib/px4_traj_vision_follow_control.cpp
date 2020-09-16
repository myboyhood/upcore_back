//
// Created by wzy on 2020/9/1.
//
#include <mission_core.h>
using namespace Eigen;


void mission_core::px4_traj_vision_follow_control(){

    if(vision_prepare_ok)
    {
    /**
     * 第一次pnp结果会和GPS拿到的local_pos结果有较大差别。用 first_large_bias 一次性加到 gps_target_pos 上
     * 之后如果 pnp_target_pos 和 gps_target_pos 差别不大，就用 small_bias 去加到 gps_target_pos 上， pnp_gps_fuse_target_pos 使用 pnp_target_pos
     * 如果差别很大，那么认为 pnp_target_pos 错误， pnp_gps_fuse_target_pos 使用 gps_target_pos
     */
    first_bias_flag = true;
    while(ros::ok()){
        ros::spinOnce();
        vision_pose_acqurie();

        gps_target_pos = Vector3d(car_pos[0]-distance_car_drone*cos(yaw_rotate),car_pos[1]-distance_car_drone*sin(yaw_rotate),home_hover_height);

        if(first_bias_flag && pnp_pose_is_good)
        {
            first_bias_flag = false;
            pnp_target_pos = Vector3d(car_pos[0]-drone_pos_vision_in_enu.x(),car_pos[1]-drone_pos_vision_in_enu.y(),home_hover_height);
            first_large_bias[0] = pnp_target_pos[0] - gps_target_pos[0];
            first_large_bias[1] = pnp_target_pos[1] - gps_target_pos[1];
        }
        ROS_INFO("first_bias [0]: %f",small_bias[0]);
        ROS_INFO("first_bias [1]: %f",small_bias[1]);
        gps_target_pos[0] += first_large_bias[0];
        gps_target_pos[1] += first_large_bias[1];

        if(pnp_pose_is_good)
        {
            pnp_target_pos = Vector3d(car_pos[0]-drone_pos_vision_in_enu.x(),car_pos[1]-drone_pos_vision_in_enu.y(),home_hover_height);
            small_bias[0] = pnp_target_pos[0] - gps_target_pos[0];
            small_bias[1] = pnp_target_pos[1] - gps_target_pos[1];
            ROS_WARN("small_bias [0]: %f",small_bias[0]);
            ROS_WARN("small_bias [1]: %f",small_bias[1]);
            gps_target_pos[0] += small_bias[0];
            gps_target_pos[1] += small_bias[1];

            if (fabs(gps_target_pos.x() - pnp_target_pos.x()) < 0.2 && fabs(gps_target_pos.y() - pnp_target_pos.y()) < 0.2)
            {
                pnp_gps_fuse_target_pos = pnp_target_pos;
            }
            else{
                pnp_gps_fuse_target_pos = gps_target_pos;
            }
        }
        
        else{
            ROS_WARN("small_bias [0]: %f",small_bias[0]);
            ROS_WARN("small_bias [1]: %f",small_bias[1]);
            gps_target_pos[0] += small_bias[0];
            gps_target_pos[1] += small_bias[1];
            ROS_WARN("pnp failed , use gps_target_pos");
            pnp_gps_fuse_target_pos = gps_target_pos;
        }

            px4_waypoint_calculate(curr_p, pnp_gps_fuse_target_pos);
            if(fabs(x_waypoints[waypoint_count]-curr_p[0]) > 0.2 || fabs(y_waypoints[waypoint_count]-curr_p[1]) > 0.2 || fabs(z_waypoints[waypoint_count]-curr_p[2]) > 0.2)
            {
                ROS_INFO("reaching the %d trajectory point", waypoint_count);
                px4_position_msg.pose.position.x = x_waypoints[waypoint_count];
                px4_position_msg.pose.position.y = y_waypoints[waypoint_count];
                px4_position_msg.pose.position.z = z_waypoints[waypoint_count];
            }
            else{
                ROS_INFO("the error between curr and next point is within 0.2m ");
                if (waypoint_count < (xyz_step_count-1)){
                    waypoint_count += 1;
                    ROS_INFO("count + 1");
                }
                px4_position_msg.pose.position.x = x_waypoints[waypoint_count];
                px4_position_msg.pose.position.y = y_waypoints[waypoint_count];
                px4_position_msg.pose.position.z = z_waypoints[waypoint_count];
            }
            ROS_INFO("car_pos_x: %f", car_pos[0]);
            ROS_INFO("car_pos_y: %f", car_pos[1]);
            ROS_INFO("car_pos_z: %f", car_pos[2]);
            ROS_INFO("px4_position_msg.pose.position.x: %f",px4_position_msg.pose.position.x);
            ROS_INFO("px4_position_msg.pose.position.y: %f",px4_position_msg.pose.position.y);
            ROS_INFO("px4_position_msg.pose.position.z: %f",px4_position_msg.pose.position.z);
            ROS_WARN("error_x: %f",car_pos[0]-distance_car_drone*cos(yaw_rotate) - px4_position_msg.pose.position.x);
            ROS_WARN("error_y: %f",car_pos[1]-distance_car_drone*sin(yaw_rotate) - px4_position_msg.pose.position.y);
            px4_setpoint_position_pub.publish(px4_position_msg);
        

        
        


        rate.sleep();
    

    }
    }
}