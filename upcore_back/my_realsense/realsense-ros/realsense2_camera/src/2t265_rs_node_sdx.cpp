// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
// #include "../include/realsense_node_factory.h"
#include <librealsense2/rs.hpp> 
#include <nav_msgs/Odometry.h>
#include <ros/ros.h> 
#include <iostream>
#include <iomanip>

#define        OFFSET_SWITCH true

//安装位置 机体坐标系
#define        OFFSET_X1  0.120
#define        OFFSET_Y1  0.0
#define        OFFSET_Z1  0.0

int main(int argc, char * argv[]) try
{
    ros::init(argc, argv, "t265_2_sdx");
    ros::NodeHandle nh; 
    //发布主题 
    ros::Publisher t265_2_sdx_pub = nh.advertise<nav_msgs::Odometry>("/camera_2/odom/sample", 100);
    ros::Rate loop_rate(210);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::context ctx;            // Create librealsense context for managing devices

    rs2::pipeline pipe(ctx);
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    // cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    std::string t265_2_1_serial = "908412110262";
    cfg.enable_device(t265_2_1_serial);
    // Start pipeline with chosen configuration
    pipe.start(cfg);
    nav_msgs::Odometry pose_msg;

        //record fist boot angle 
    bool t2652_first_boot = true;
    float t2652_first_ang[3];

    float compensate_data1[4];
    // Main loop
    while (ros::ok())
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose = f.as<rs2::pose_frame>().get_pose_data();

        // if(pose.translation.z==pose.translation.z){
        //     ROS_WARN("CAMERA 2 STOPPED !!!!! RESTART....");
        //     pipe.start(cfg);
        // }
        
        pose_msg.header.stamp = ros::Time::now ();
        pose_msg.header.frame_id = "camera_odom_frame";
        pose_msg.child_frame_id = "camera_pose_frame";
        pose_msg.pose.pose.position.x = pose.translation.z;
        pose_msg.pose.pose.position.y = -pose.translation.x;
        pose_msg.pose.pose.position.z = -pose.translation.y;
        pose_msg.pose.pose.orientation.x = pose.rotation.z;
        pose_msg.pose.pose.orientation.y = -pose.rotation.x;
        pose_msg.pose.pose.orientation.z = -pose.rotation.y;
        pose_msg.pose.pose.orientation.w = pose.rotation.w;
        pose_msg.twist.twist.angular.x = pose.angular_velocity.z;
        pose_msg.twist.twist.angular.y = -pose.angular_velocity.x;
        pose_msg.twist.twist.angular.z = -pose.angular_velocity.y;
        pose_msg.twist.twist.linear.x = pose.velocity.z;
        pose_msg.twist.twist.linear.y = -pose.velocity.x;
        pose_msg.twist.twist.linear.z = -pose.velocity.y;

        if(t2652_first_boot){
            t2652_first_boot = false;
            float w1,x1,y1,z1;
            x1 = pose_msg.pose.pose.orientation.x;
            y1 = pose_msg.pose.pose.orientation.y;
            z1 = pose_msg.pose.pose.orientation.z;
            w1 = pose_msg.pose.pose.orientation.w;
            t2652_first_ang[0] = (float)atan2(2*(w1*x1+y1*z1),1-2*(x1*x1+y1*y1));
            t2652_first_ang[1] = (float)asin(2*(w1*y1-x1*z1));
            t2652_first_ang[2] = (float)atan2(2*(w1*z1+x1*y1),1-2*(z1*z1+y1*y1));
        }

        if(OFFSET_SWITCH){
            float t2652_T[4][4];
            float current_angle1[3];
            float offset[4];
            float cw1,cx1,cy1,cz1;
            cx1 = pose_msg.pose.pose.orientation.x;
            cy1 = pose_msg.pose.pose.orientation.y;
            cz1 = pose_msg.pose.pose.orientation.z;
            cw1 = pose_msg.pose.pose.orientation.w;

            current_angle1[0] = (float)atan2(2*(cw1*cx1+cy1*cz1),1-2*(cx1*cx1+cy1*cy1));
            current_angle1[1] = (float)asin(2*(cw1*cy1-cx1*cz1));
            current_angle1[2] = (float)atan2(2*(cw1*cz1+cx1*cy1),1-2*(cz1*cz1+cy1*cy1));

            for (int i = 0;i<3;i++){
                current_angle1[i] -= t2652_first_ang[i];
            }

            t2652_T[0][0] = cos(current_angle1[1])*cos(current_angle1[2]);
            t2652_T[0][1] = -cos(current_angle1[1])*sin(current_angle1[2]);
            t2652_T[0][2] = sin(current_angle1[1]);
            t2652_T[1][0] = cos(current_angle1[0])*sin(current_angle1[2])+sin(current_angle1[0])*sin(current_angle1[1])*cos(current_angle1[2]);
            t2652_T[1][1] = cos(current_angle1[0])*cos(current_angle1[2])-sin(current_angle1[0])*sin(current_angle1[1])*sin(current_angle1[2]);
            t2652_T[1][2] = -sin(current_angle1[0])*cos(current_angle1[1]);
            t2652_T[2][0] = sin(current_angle1[0])*sin(current_angle1[2])-cos(current_angle1[0])*sin(current_angle1[1])*cos(current_angle1[2]);
            t2652_T[2][1] = sin(current_angle1[0])*cos(current_angle1[2])+cos(current_angle1[0])*sin(current_angle1[1])*sin(current_angle1[2]);
            t2652_T[2][2] = cos(current_angle1[0])*cos(current_angle1[1]);
            t2652_T[0][3] = -OFFSET_X1;
            t2652_T[1][3] = -OFFSET_Y1;
            t2652_T[2][3] = -OFFSET_Z1;
            t2652_T[3][3] = 1;

            offset[0] = OFFSET_X1;
            offset[1] = OFFSET_Y1;
            offset[2] = OFFSET_Z1;
            offset[3] = 1.0f;

            for(int i=0;i<4;i++){
                //reset
                compensate_data1[i] = 0;
                for(int j = 0;j<4;j++){
                compensate_data1[i] += t2652_T[i][j]*offset[j]; 
                }          
            }
            pose_msg.pose.pose.position.x -= compensate_data1[0];
            pose_msg.pose.pose.position.y -= compensate_data1[1];
            pose_msg.pose.pose.position.z -= compensate_data1[2];
        }

        double cov_pose(0.01 * pow(10, 3-(int)pose.tracker_confidence));
        double cov_twist(0.01 * pow(10, 1-(int)pose.tracker_confidence));
        double track_con(-1.0);
        double map_con(-1.0);

        switch((int)(pose.tracker_confidence)){
            case 0:
                track_con = 0.0;
                break;
            case 1:
                track_con = 1.0;
                break;
            case 2:
                track_con = 2.0;
                break;
            case 3:
                track_con = 3.0;
                break;
            default:
                break;
        }

        switch((int)(pose.mapper_confidence)){
            case 0:
                map_con = 0.0;
                break;
            case 1:
                map_con = 1.0;
                break;
            case 2:
                map_con = 2.0;
                break;
            case 3:
                map_con = 3.0;
                break;
            default:
                break;
        }

        // 用于保存地图可信度，追踪可信度
        pose_msg.pose.covariance = {track_con, 0, 0, 0, 0, 0,
                            0, track_con, 0, 0, 0, 0,
                            0, 0, track_con, 0, 0, 0,
                            0, 0, 0, map_con, 0, 0,
                            0, 0, 0, 0, map_con, 0,
                            0, 0, 0, 0, 0, map_con};

        pose_msg.twist.covariance = {cov_pose, 0, 0, 0, 0, 0,
                            0, cov_pose, 0, 0, 0, 0,
                            0, 0, cov_pose, 0, 0, 0,
                            0, 0, 0, cov_twist, 0, 0,
                            0, 0, 0, 0, cov_twist, 0,
                            0, 0, 0, 0, 0, cov_twist};

        ROS_INFO(
            "Device Position: %2.3f,%2.3f,%2.3f",
            pose_msg.pose.pose.position.x,pose_msg.pose.pose.position.y,pose_msg.pose.pose.position.z
            );
        t265_2_sdx_pub.publish(pose_msg);
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
