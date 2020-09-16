// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp> 
#include <nav_msgs/Odometry.h>
#include <ros/ros.h> 
#include <iostream>
#include <iomanip>

#define        OFFSET_SWITCH true

//安装位置 机体坐标系
#define        OFFSET_X2  -0.120
#define        OFFSET_Y2  0.0
#define        OFFSET_Z2  0.0

int main(int argc, char * argv[]) try
{
    ros::init(argc, argv, "t265_1_sdx");
    ros::NodeHandle nh; 
    //发布主题 
    ros::Publisher t265_1_sdx_pub = nh.advertise<nav_msgs::Odometry>("/camera_1/odom/sample", 100);
    ros::Rate loop_rate(210);
    rs2::context                ctx;            // Create librealsense context for managing devices

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe(ctx);
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    std::string t265_1_1_serial = "908412110257";
    cfg.enable_device(t265_1_1_serial);
    // cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);
    nav_msgs::Odometry pose_msg;

    bool t2651_first_boot = true;
    float t2651_first_ang[3];
    float compensate_data2[4];
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
        //     ROS_WARN("CAMERA 1 STOPPED !!!!! RESTART....");
        //     pipe.start(cfg);
        // }
        
        pose_msg.header.stamp = ros::Time::now ();
        pose_msg.header.frame_id = "camera_odom_frame";
        pose_msg.child_frame_id = "camera_pose_frame";
        pose_msg.pose.pose.position.x = -pose.translation.z;
        pose_msg.pose.pose.position.y = pose.translation.x;
        pose_msg.pose.pose.position.z = -pose.translation.y;
        pose_msg.pose.pose.orientation.x = -pose.rotation.z;
        pose_msg.pose.pose.orientation.y = pose.rotation.x;
        pose_msg.pose.pose.orientation.z = -pose.rotation.y;
        pose_msg.pose.pose.orientation.w = pose.rotation.w;
        pose_msg.twist.twist.angular.x = -pose.angular_velocity.z;
        pose_msg.twist.twist.angular.y = pose.angular_velocity.x;
        pose_msg.twist.twist.angular.z = -pose.angular_velocity.y;
        pose_msg.twist.twist.linear.x = -pose.velocity.z;
        pose_msg.twist.twist.linear.y = pose.velocity.x;
        pose_msg.twist.twist.linear.z = -pose.velocity.y;

                // get first boot angular
        if(t2651_first_boot){
            t2651_first_boot = false;
            float w2,x2,y2,z2;
            x2 = pose_msg.pose.pose.orientation.x;
            y2 = pose_msg.pose.pose.orientation.y;
            z2 = pose_msg.pose.pose.orientation.z;
            w2 = pose_msg.pose.pose.orientation.w;
            t2651_first_ang[0] = (float)atan2(2*(w2*x2+y2*z2),1-2*(x2*x2+y2*y2));
            t2651_first_ang[1] = (float)asin(2*(w2*y2-x2*z2));
            t2651_first_ang[2] = (float)atan2(2*(w2*z2+x2*y2),1-2*(z2*z2+y2*y2));
        }

        if(OFFSET_SWITCH){
        float t2651_T[4][4];
        float current_angle2[3];
        float offset[4];
        float cw2,cx2,cy2,cz2;

        cx2 = pose_msg.pose.pose.orientation.x;
        cy2 = pose_msg.pose.pose.orientation.y;
        cz2 = pose_msg.pose.pose.orientation.z;
        cw2 = pose_msg.pose.pose.orientation.w;

        current_angle2[0] = (float)atan2(2*(cw2*cx2+cy2*cz2),1-2*(cx2*cx2+cy2*cy2));
        current_angle2[1] = (float)asin(2*(cw2*cy2-cx2*cz2));
        current_angle2[2] = (float)atan2(2*(cw2*cz2+cx2*cy2),1-2*(cz2*cz2+cy2*cy2));

        for (int i = 0;i<3;i++){
            current_angle2[i] -= t2651_first_ang[i];
        }

        t2651_T[0][0] = cos(current_angle2[1])*cos(current_angle2[2]);
        t2651_T[0][1] = -cos(current_angle2[1])*sin(current_angle2[2]);
        t2651_T[0][2] = sin(current_angle2[1]);
        t2651_T[1][0] = cos(current_angle2[0])*sin(current_angle2[2])+sin(current_angle2[0])*sin(current_angle2[1])*cos(current_angle2[2]);
        t2651_T[1][1] = cos(current_angle2[0])*cos(current_angle2[2])-sin(current_angle2[0])*sin(current_angle2[1])*sin(current_angle2[2]);
        t2651_T[1][2] = -sin(current_angle2[0])*cos(current_angle2[1]);
        t2651_T[2][0] = sin(current_angle2[0])*sin(current_angle2[2])-cos(current_angle2[0])*sin(current_angle2[1])*cos(current_angle2[2]);
        t2651_T[2][1] = sin(current_angle2[0])*cos(current_angle2[2])+cos(current_angle2[0])*sin(current_angle2[1])*sin(current_angle2[2]);
        t2651_T[2][2] = cos(current_angle2[0])*cos(current_angle2[1]);
        t2651_T[0][3] = -OFFSET_X2;
        t2651_T[1][3] = -OFFSET_Y2;
        t2651_T[2][3] = -OFFSET_Z2;
        t2651_T[3][3] = 1;

        offset[0] = OFFSET_X2;
        offset[1] = OFFSET_Y2;
        offset[2] = OFFSET_Z2;
        offset[3] = 1.0f;

        for(int i=0;i<4;i++){
            //reset
            compensate_data2[i] = 0;
            for(int j = 0;j<4;j++){
               compensate_data2[i] += t2651_T[i][j]*offset[j]; 
            }          
        }
        pose_msg.pose.pose.position.x -= compensate_data2[0];
        pose_msg.pose.pose.position.y -= compensate_data2[1];
        pose_msg.pose.pose.position.z -= compensate_data2[2];
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
        t265_1_sdx_pub.publish(pose_msg);
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
