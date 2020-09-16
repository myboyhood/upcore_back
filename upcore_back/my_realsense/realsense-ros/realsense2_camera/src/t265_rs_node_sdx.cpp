// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include "../include/realsense_node_factory.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h> 
#include <iostream>
#include <iomanip>

int main(int argc, char * argv[]) try
{
    ros::init(argc, argv, "t265_sdx");
    ros::NodeHandle nh; 
    //发布主题 
    ros::Publisher t265_sdx_pub = nh.advertise<nav_msgs::Odometry>("/camera/odom/sample", 100);
    ros::Rate loop_rate(210);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // std::string T265_1_serial = "908412110262";
    // cfg.enable_device(T265_1_serial);
    // Start pipeline with chosen configuration
    pipe.start(cfg);
    nav_msgs::Odometry pose_msg;
    // Main loop
    while (ros::ok())
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose = f.as<rs2::pose_frame>().get_pose_data();
        
        pose_msg.header.stamp = ros::Time::now ();
        pose_msg.header.frame_id = "camera_odom_frame";
        pose_msg.child_frame_id = "camera_pose_frame";
        pose_msg.pose.pose.position.x = -pose.translation.z;
        pose_msg.pose.pose.position.y = -pose.translation.x;
        pose_msg.pose.pose.position.z = pose.translation.y;
        pose_msg.pose.pose.orientation.x = -pose.rotation.z;
        pose_msg.pose.pose.orientation.y = -pose.rotation.x;
        pose_msg.pose.pose.orientation.z = pose.rotation.y;
        pose_msg.pose.pose.orientation.w = pose.rotation.w;
        pose_msg.twist.twist.angular.x = -pose.angular_velocity.z;
        pose_msg.twist.twist.angular.y = -pose.angular_velocity.x;
        pose_msg.twist.twist.angular.z = pose.angular_velocity.y;
        pose_msg.twist.twist.linear.x = -pose.velocity.z;
        pose_msg.twist.twist.linear.y = -pose.velocity.x;
        pose_msg.twist.twist.linear.z = pose.velocity.y;

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
        t265_sdx_pub.publish(pose_msg);
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
