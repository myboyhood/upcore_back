//
// Created by wzy on 2020/9/1.
//
#include <mission_core.h>
using namespace Eigen;

void mission_core::backhome() {

    //back home position above
    Vector3d home_position_above(x_pos_home_origin,y_pos_home_origin,home_hover_height);
    xyz_step_count = 10; // 分为100个点返回到home上方
    px4_waypoint_calculate(curr_p,home_position_above);
    for (int i = 0; i < x_waypoints.size(); ++i) {
        cout << x_waypoints[i] << endl;
        cout << y_waypoints[i] << endl;
        cout << z_waypoints[i] << endl;
    }

    waypoint_count = 0;
    while(ros::ok()){
        ros::spinOnce();

        ROS_INFO("home_x: %f",x_pos_home_origin);
        ROS_INFO("home_y: %f",y_pos_home_origin);
        ROS_INFO("home_z: %f",z_pos_home_origin);
        ROS_INFO("x_waypoints [ %d ]: %f", waypoint_count, x_waypoints[waypoint_count]);
        ROS_INFO("curr_p_xyz: %f, %f, %f", curr_p[0], curr_p[1],curr_p[2]);
        cout << "x_error: " << x_waypoints[waypoint_count]-curr_p[0] << endl;
        cout << "y_error: " << y_waypoints[waypoint_count]-curr_p[1] << endl;
        cout << "z_error: " << z_waypoints[waypoint_count]-curr_p[2] << endl;
        if(fabs(x_waypoints[waypoint_count]-curr_p[0]) > 1 || fabs(y_waypoints[waypoint_count]-curr_p[1]) > 1 || fabs(z_waypoints[waypoint_count]-curr_p[2]) > 1)
        {
            ROS_INFO("reaching the %d trajectory point", waypoint_count);
            px4_position_msg.pose.position.x = x_waypoints[waypoint_count];
            px4_position_msg.pose.position.y = y_waypoints[waypoint_count];
            px4_position_msg.pose.position.z = z_waypoints[waypoint_count];
            ROS_INFO("px4_position_msg.pose.position.x: %f",px4_position_msg.pose.position.x);
            ROS_INFO("px4_position_msg.pose.position.y: %f",px4_position_msg.pose.position.y);
            ROS_INFO("px4_position_msg.pose.position.z: %f",px4_position_msg.pose.position.z);
        }
        else{
            if (waypoint_count < (xyz_step_count-1)){
                waypoint_count += 1;
                ROS_WARN("COUNT ADD 1");
            }

            else{
                break;
            }
            px4_position_msg.pose.position.x = x_waypoints[waypoint_count];
            px4_position_msg.pose.position.y = y_waypoints[waypoint_count];
            px4_position_msg.pose.position.z = z_waypoints[waypoint_count];

        }

        px4_setpoint_position_pub.publish(px4_position_msg);
        rate.sleep();
    }

    //land at home_position
    Vector3d home_position_land(x_pos_home_origin,y_pos_home_origin,z_pos_home_origin);
    xyz_step_count = 5;
    px4_waypoint_calculate(curr_p,home_position_land);

    for (int i = 0; i < x_waypoints.size(); ++i) {
        cout << x_waypoints[i] << endl;
        cout << y_waypoints[i] << endl;
        cout << z_waypoints[i] << endl;
    }


    offb_set_mode.request.custom_mode = "MANUAL";
    arm_cmd.request.value = false;
    waypoint_count = 0;
    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        ros::spinOnce();

            if(fabs(x_waypoints[waypoint_count]-curr_p[0]) > 0.2 || fabs(y_waypoints[waypoint_count]-curr_p[1]) > 0.2 || fabs(z_waypoints[waypoint_count]-curr_p[2]) > 0.2)
            {
                ROS_INFO("reaching the %d trajectory point", waypoint_count);
                px4_position_msg.pose.position.x = x_waypoints[waypoint_count];
                px4_position_msg.pose.position.y = y_waypoints[waypoint_count];
                px4_position_msg.pose.position.z = z_waypoints[waypoint_count];
                ROS_INFO("px4_position_msg.pose.position.x: %f",px4_position_msg.pose.position.x);
                ROS_INFO("px4_position_msg.pose.position.y: %f",px4_position_msg.pose.position.y);
                ROS_INFO("px4_position_msg.pose.position.z: %f",px4_position_msg.pose.position.z);
            }
            else{
                if (waypoint_count < (xyz_step_count-1)){
                    waypoint_count += 1;
                }
                else{

                    if( current_state_msg.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                            ROS_INFO("Vehicle Disarmed");
                        }
                        last_request = ros::Time::now();
                    }

                }
                px4_position_msg.pose.position.x = x_waypoints[waypoint_count];
                px4_position_msg.pose.position.y = y_waypoints[waypoint_count];
                px4_position_msg.pose.position.z = z_waypoints[waypoint_count];

            }

            px4_setpoint_position_pub.publish(px4_position_msg);
            rate.sleep();
    }


}