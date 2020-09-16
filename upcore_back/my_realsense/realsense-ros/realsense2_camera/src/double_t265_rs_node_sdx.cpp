// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp> 
#include "realsense2_camera/Double_Odometry.h"
#include <ros/ros.h> 
#include <iostream>
#include <iomanip>
#include <cmath>
/**********************************************************
 * 
 * 启动双T265模块的ros节点
 * 全局采用飞机飞控坐标系FRD，前置双目相机向前，X为向前，Y向右，Z向下
 * 全局采用飞机飞控坐标系FRD，后置双目相机向前，X为向后，Y向左，Z向下
 * 
 * 2019.8.13
 * 
 **********************************************************/

#define        OFFSET_SWITCH false

//安装位置 机体坐标系
#define        OFFSET_X1  0.120f
#define        OFFSET_Y1  0.0f
#define        OFFSET_Z1  0.0f
#define        OFFSET_X2  -0.120f
#define        OFFSET_Y2  0.0f
#define        OFFSET_Z2  0.0f



int main(int argc, char * argv[]) try
{
    ros::init(argc, argv, "double_t265");
    ros::NodeHandle nh; 
    //发布主题 
    ros::Publisher _1_t265_sdx_pub = nh.advertise<realsense2_camera::Double_Odometry>("/double_camera/odom/sample", 1);
    ros::Rate loop_rate(50);
    rs2::context                ctx;            // Create librealsense context for managing devices
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe1(ctx);
    rs2::pipeline pipe2(ctx);
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg1;
    rs2::config cfg2;
    // front t265    
    std::string _1_t265_serial = "908412110273";
    // back t265
    std::string _2_t265_serial = "908412110262";
    cfg1.enable_device(_1_t265_serial);
    // don't fix this bug!!!!
    cfg2.enable_device(_2_t265_serial);
    // Start pipeline with chosen configuration
    pipe1.start(cfg1);
    pipe2.start(cfg2);
    realsense2_camera::Double_Odometry pose_msg;
    rs2_pose pose_1;
    rs2_pose pose_2;
    
    //record fist boot angle 
    bool t2651_first_boot = true;
    float t2651_first_ang[3];
    bool t2652_first_boot = true;
    float t2652_first_ang[3];
    //compensate data 
    float compensate_data1[4];
    float compensate_data2[4];
    // float last_pose_x1 = 10000.0;
    // float last_pose_x2 = 10000.0;

    // bool data1_valid;
    // bool data2_valid;

    // Main loop
    ros::Duration(2).sleep();
    rs2::frameset fs1;
    rs2::frameset fs2;

    while (ros::ok())
    {

        if (pipe1.poll_for_frames(&fs1)) {
            auto f_1 = fs1.first_or_default(RS2_STREAM_POSE);
            pose_1 = f_1.as<rs2::pose_frame>().get_pose_data();
            // std::cout << "\n" << "Device 1 Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
            //           pose_data.translation.y << " " << pose_data.translation.z << "";
        }
        ros::Duration(0.001).sleep();
        
        if (pipe2.poll_for_frames(&fs2)) {
            auto f_2 = fs2.first_or_default(RS2_STREAM_POSE);
            pose_2 = f_2.as<rs2::pose_frame>().get_pose_data();
            // std::cout << "\n" << "Device 2 Position: " << std::setprecision(3) << std::fixed << pose_data2.translation.x << " " <<
            //           pose_data2.translation.y << " " << pose_data2.translation.z << "";
        }
        ros::Duration(0.001).sleep();
        
        
        // if(pose_1.translation.z==last_pose_x1){
        //     ROS_WARN("CAMERA 1 STOPPED !!!!! RESTART....");
        //     pipe1.stop();
        //     pipe1.start(cfg1);
        // }
        // last_pose_x1 = pose_1.translation.z;
        // if(pose_2.translation.z==last_pose_x2){
        //     ROS_WARN("CAMERA 2 STOPPED !!!!! RESTART....");
        //     pipe2.stop();
        //     pipe2.start(cfg2);
        // }
        // last_pose_x2 = pose_2.translation.z;

        pose_msg.header.stamp = ros::Time::now ();
        pose_msg.header.frame_id = "camera_odom_frame";
        pose_msg.child_frame_id = "camera_pose_frame";
        pose_msg.pose_1.pose.position.x = -pose_1.translation.z;
        pose_msg.pose_1.pose.position.y = pose_1.translation.x;
        pose_msg.pose_1.pose.position.z = -pose_1.translation.y;
        pose_msg.pose_1.pose.orientation.x = -pose_1.rotation.z;
        pose_msg.pose_1.pose.orientation.y = pose_1.rotation.x;
        pose_msg.pose_1.pose.orientation.z = -pose_1.rotation.y;
        pose_msg.pose_1.pose.orientation.w = pose_1.rotation.w;
        pose_msg.twist_1.twist.angular.x = -pose_1.angular_velocity.z;
        pose_msg.twist_1.twist.angular.y = pose_1.angular_velocity.x;
        pose_msg.twist_1.twist.angular.z = -pose_1.angular_velocity.y;
        pose_msg.twist_1.twist.linear.x = -pose_1.velocity.z;
        pose_msg.twist_1.twist.linear.y = pose_1.velocity.x;
        pose_msg.twist_1.twist.linear.z = -pose_1.velocity.y;
        // data1_valid =( 
        //     (pose_msg.pose_1.pose.position.x==pose_msg.pose_1.pose.position.x)&&
        //     (pose_msg.pose_1.pose.position.y==pose_msg.pose_1.pose.position.y)&&
        //     (pose_msg.pose_1.pose.position.z==pose_msg.pose_1.pose.position.z)&&
        //     (pose_msg.pose_1.pose.orientation.x==pose_msg.pose_1.pose.orientation.x)&&
        //     (pose_msg.pose_1.pose.orientation.y==pose_msg.pose_1.pose.orientation.y)&&
        //     (pose_msg.pose_1.pose.orientation.z==pose_msg.pose_1.pose.orientation.z)&&
        //     (pose_msg.pose_1.pose.orientation.w==pose_msg.pose_1.pose.orientation.w)&&
        //     (pose_msg.twist_1.twist.angular.x==pose_msg.twist_1.twist.angular.x)&&
        //     (pose_msg.twist_1.twist.angular.y==pose_msg.twist_1.twist.angular.y)&&
        //     (pose_msg.twist_1.twist.angular.z==pose_msg.twist_1.twist.angular.z)&&
        //     (pose_msg.twist_1.twist.linear.x==pose_msg.twist_1.twist.linear.x)&&
        //     (pose_msg.twist_1.twist.linear.y==pose_msg.twist_1.twist.linear.y)&&
        //     (pose_msg.twist_1.twist.linear.z==pose_msg.twist_1.twist.linear.z)
        // );
        // if(!data1_valid){
        //     ROS_WARN("Invalid data!");
        //     continue;
        // }
        // get first boot angular
        if(t2651_first_boot){
            t2651_first_boot = false;
            float w1,x1,y1,z1;
            x1 = pose_msg.pose_1.pose.orientation.x;
            y1 = pose_msg.pose_1.pose.orientation.y;
            z1 = pose_msg.pose_1.pose.orientation.z;
            w1 = pose_msg.pose_1.pose.orientation.w;
            t2651_first_ang[0] = (float)atan2(2*(w1*x1+y1*z1),1-2*(x1*x1+y1*y1));
            t2651_first_ang[1] = (float)asin(2*(w1*y1-x1*z1));
            t2651_first_ang[2] = (float)atan2(2*(w1*z1+x1*y1),1-2*(z1*z1+y1*y1));
            
        }

        //compensate offset
        // OFFSET_SWITCH = 1 
        if(OFFSET_SWITCH){
            float t2651_T[4][4];
            float current_angle1[3];
            float offset1[4];
            float cw1,cx1,cy1,cz1;

            cx1 = pose_msg.pose_1.pose.orientation.x;
            cy1 = pose_msg.pose_1.pose.orientation.y;
            cz1 = pose_msg.pose_1.pose.orientation.z;
            cw1 = pose_msg.pose_1.pose.orientation.w;

            current_angle1[0] = (float)atan2(2*(cw1*cx1+cy1*cz1),1-2*(cx1*cx1+cy1*cy1));
            current_angle1[1] = (float)asin(2*(cw1*cy1-cx1*cz1));
            current_angle1[2] = (float)atan2(2*(cw1*cz1+cx1*cy1),1-2*(cz1*cz1+cy1*cy1));

            for (int i = 0;i<3;i++){
                current_angle1[i] -= t2651_first_ang[i];
            }

            t2651_T[0][0] = cos(current_angle1[1])*cos(current_angle1[2]);
            t2651_T[0][1] = -cos(current_angle1[1])*sin(current_angle1[2]);
            t2651_T[0][2] = sin(current_angle1[1]);
            t2651_T[1][0] = cos(current_angle1[0])*sin(current_angle1[2])+sin(current_angle1[0])*sin(current_angle1[1])*cos(current_angle1[2]);
            t2651_T[1][1] = cos(current_angle1[0])*cos(current_angle1[2])-sin(current_angle1[0])*sin(current_angle1[1])*sin(current_angle1[2]);
            t2651_T[1][2] = -sin(current_angle1[0])*cos(current_angle1[1]);
            t2651_T[2][0] = sin(current_angle1[0])*sin(current_angle1[2])-cos(current_angle1[0])*sin(current_angle1[1])*cos(current_angle1[2]);
            t2651_T[2][1] = sin(current_angle1[0])*cos(current_angle1[2])+cos(current_angle1[0])*sin(current_angle1[1])*sin(current_angle1[2]);
            t2651_T[2][2] = cos(current_angle1[0])*cos(current_angle1[1]);
            t2651_T[0][3] = -OFFSET_X1;
            t2651_T[1][3] = -OFFSET_Y1;
            t2651_T[2][3] = -OFFSET_Z1;
            t2651_T[3][3] = 1;

            offset1[0] = OFFSET_X1;
            offset1[1] = OFFSET_Y1;
            offset1[2] = OFFSET_Z1;
            offset1[3] = 1.0f;

            for(int i=0;i<4;i++){
                //reset
                compensate_data1[i] = 0.0f;
                for(int j = 0;j<4;j++){
                compensate_data1[i] += t2651_T[i][j]*offset1[j];
                }          
            }
            // ROS_INFO("compensate data  %2.3f,%2.3f,%2.3f",current_angle1[1],t2651_first_ang[0],t2651_T[0][3]); 
            pose_msg.pose_1.pose.position.x -= compensate_data1[0];
            pose_msg.pose_1.pose.position.y -= compensate_data1[1];
            pose_msg.pose_1.pose.position.z -= compensate_data1[2];
        }

        double cov_pose(0.01 * pow(10, 3-(int)pose_1.tracker_confidence));
        double cov_twist(0.01 * pow(10, 1-(int)pose_1.tracker_confidence));
        double track_con(-1.0);
        double map_con(-1.0);

        switch((int)(pose_1.tracker_confidence)){
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

        switch((int)(pose_1.mapper_confidence)){
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
        pose_msg.pose_1.covariance = {track_con, 0, 0, 0, 0, 0,
                            0, track_con, 0, 0, 0, 0,
                            0, 0, track_con, 0, 0, 0,
                            0, 0, 0, map_con, 0, 0,
                            0, 0, 0, 0, map_con, 0,
                            0, 0, 0, 0, 0, map_con};

        pose_msg.twist_1.covariance = {cov_pose, 0, 0, 0, 0, 0,
                            0, cov_pose, 0, 0, 0, 0,
                            0, 0, cov_pose, 0, 0, 0,
                            0, 0, 0, cov_twist, 0, 0,
                            0, 0, 0, 0, cov_twist, 0,
                            0, 0, 0, 0, 0, cov_twist};

        pose_msg.pose_2.pose.position.x = pose_2.translation.z;
        pose_msg.pose_2.pose.position.y = -pose_2.translation.x;
        pose_msg.pose_2.pose.position.z = -pose_2.translation.y;
        pose_msg.pose_2.pose.orientation.x = pose_2.rotation.z;
        pose_msg.pose_2.pose.orientation.y = -pose_2.rotation.x;
        pose_msg.pose_2.pose.orientation.z = -pose_2.rotation.y;
        pose_msg.pose_2.pose.orientation.w = pose_2.rotation.w;
        pose_msg.twist_2.twist.angular.x = pose_2.angular_velocity.z;
        pose_msg.twist_2.twist.angular.y = -pose_2.angular_velocity.x;
        pose_msg.twist_2.twist.angular.z = -pose_2.angular_velocity.y;
        pose_msg.twist_2.twist.linear.x = pose_2.velocity.z;
        pose_msg.twist_2.twist.linear.y = -pose_2.velocity.x;
        pose_msg.twist_2.twist.linear.z = -pose_2.velocity.y;

        // data2_valid =( 
        //     (pose_msg.pose_2.pose.position.x==pose_msg.pose_2.pose.position.x)&&
        //     (pose_msg.pose_2.pose.position.y==pose_msg.pose_2.pose.position.y)&&
        //     (pose_msg.pose_2.pose.position.z==pose_msg.pose_2.pose.position.z)&&
        //     (pose_msg.pose_2.pose.orientation.x==pose_msg.pose_2.pose.orientation.x)&&
        //     (pose_msg.pose_2.pose.orientation.y==pose_msg.pose_2.pose.orientation.y)&&
        //     (pose_msg.pose_2.pose.orientation.z==pose_msg.pose_2.pose.orientation.z)&&
        //     (pose_msg.pose_2.pose.orientation.w==pose_msg.pose_2.pose.orientation.w)&&
        //     (pose_msg.twist_2.twist.angular.x==pose_msg.twist_2.twist.angular.x)&&
        //     (pose_msg.twist_2.twist.angular.y==pose_msg.twist_2.twist.angular.y)&&
        //     (pose_msg.twist_2.twist.angular.z==pose_msg.twist_2.twist.angular.z)&&
        //     (pose_msg.twist_2.twist.linear.x==pose_msg.twist_2.twist.linear.x)&&
        //     (pose_msg.twist_2.twist.linear.y==pose_msg.twist_2.twist.linear.y)&&
        //     (pose_msg.twist_2.twist.linear.z==pose_msg.twist_2.twist.linear.z)
        // );
        // if(!data2_valid){
        //     continue;
        // }

        // get first boot angular
        if(t2652_first_boot){
            t2652_first_boot = false;
            float w2,x2,y2,z2;
            x2 = pose_msg.pose_2.pose.orientation.x;
            y2 = pose_msg.pose_2.pose.orientation.y;
            z2 = pose_msg.pose_2.pose.orientation.z;
            w2 = pose_msg.pose_2.pose.orientation.w;
            t2652_first_ang[0] = (float)atan2(2*(w2*x2+y2*z2),1-2*(x2*x2+y2*y2));
            t2652_first_ang[1] = (float)asin(2*(w2*y2-x2*z2));
            t2652_first_ang[2] = (float)atan2(2*(w2*z2+x2*y2),1-2*(z2*z2+y2*y2));
        }

        if(OFFSET_SWITCH){
        float t2652_T[4][4];
        float current_angle2[3];
        float offset[4];
        float cw2,cx2,cy2,cz2;

        cx2 = pose_msg.pose_2.pose.orientation.x;
        cy2 = pose_msg.pose_2.pose.orientation.y;
        cz2 = pose_msg.pose_2.pose.orientation.z;
        cw2 = pose_msg.pose_2.pose.orientation.w;

        current_angle2[0] = (float)atan2(2*(cw2*cx2+cy2*cz2),1-2*(cx2*cx2+cy2*cy2));
        current_angle2[1] = (float)asin(2*(cw2*cy2-cx2*cz2));
        current_angle2[2] = (float)atan2(2*(cw2*cz2+cx2*cy2),1-2*(cz2*cz2+cy2*cy2));

        for (int i = 0;i<3;i++){
            current_angle2[i] -= t2652_first_ang[i];
        }

        t2652_T[0][0] = cos(current_angle2[1])*cos(current_angle2[2]);
        t2652_T[0][1] = -cos(current_angle2[1])*sin(current_angle2[2]);
        t2652_T[0][2] = sin(current_angle2[1]);
        t2652_T[1][0] = cos(current_angle2[0])*sin(current_angle2[2])+sin(current_angle2[0])*sin(current_angle2[1])*cos(current_angle2[2]);
        t2652_T[1][1] = cos(current_angle2[0])*cos(current_angle2[2])-sin(current_angle2[0])*sin(current_angle2[1])*sin(current_angle2[2]);
        t2652_T[1][2] = -sin(current_angle2[0])*cos(current_angle2[1]);
        t2652_T[2][0] = sin(current_angle2[0])*sin(current_angle2[2])-cos(current_angle2[0])*sin(current_angle2[1])*cos(current_angle2[2]);
        t2652_T[2][1] = sin(current_angle2[0])*cos(current_angle2[2])+cos(current_angle2[0])*sin(current_angle2[1])*sin(current_angle2[2]);
        t2652_T[2][2] = cos(current_angle2[0])*cos(current_angle2[1]);
        t2652_T[0][3] = -OFFSET_X2;
        t2652_T[1][3] = -OFFSET_Y2;
        t2652_T[2][3] = -OFFSET_Z2;
        t2652_T[3][3] = 1;

        offset[0] = OFFSET_X2;
        offset[1] = OFFSET_Y2;
        offset[2] = OFFSET_Z2;
        offset[3] = 1.0f;

        for(int i=0;i<4;i++){
            //reset
            compensate_data2[i] = 0;
            for(int j = 0;j<4;j++){
               compensate_data2[i] += t2652_T[i][j]*offset[j]; 
            }          
        }
        pose_msg.pose_2.pose.position.x -= compensate_data2[0];
        pose_msg.pose_2.pose.position.y -= compensate_data2[1];
        pose_msg.pose_2.pose.position.z -= compensate_data2[2];
        }

        double cov_pose2(0.01 * pow(10, 3-(int)pose_2.tracker_confidence));
        double cov_twist2(0.01 * pow(10, 1-(int)pose_2.tracker_confidence));
        double track_con2(-1.0);
        double map_con2(-1.0);

        switch((int)(pose_2.tracker_confidence)){
            case 0:
                track_con2 = 0.0;
                break;
            case 1:
                track_con2 = 1.0;
                break;
            case 2:
                track_con2 = 2.0;
                break;
            case 3:
                track_con2 = 3.0;
                break;
            default:
                break;
        }

        switch((int)(pose_2.mapper_confidence)){
            case 0:
                map_con2 = 0.0;
                break;
            case 1:
                map_con2 = 1.0;
                break;
            case 2:
                map_con2 = 2.0;
                break;
            case 3:
                map_con2 = 3.0;
                break;
            default:
                break;
        }

        // 用于保存地图可信度，追踪可信度
        pose_msg.pose_2.covariance = {track_con2, 0, 0, 0, 0, 0,
                            0, track_con2, 0, 0, 0, 0,
                            0, 0, track_con2, 0, 0, 0,
                            0, 0, 0, map_con2, 0, 0,
                            0, 0, 0, 0, map_con2, 0,
                            0, 0, 0, 0, 0, map_con2};

        pose_msg.twist_2.covariance = {cov_pose2, 0, 0, 0, 0, 0,
                            0, cov_pose2, 0, 0, 0, 0,
                            0, 0, cov_pose2, 0, 0, 0,
                            0, 0, 0, cov_twist2, 0, 0,
                            0, 0, 0, 0, cov_twist2, 0,
                            0, 0, 0, 0, 0, cov_twist2};

        // ROS_INFO(
        //     "Device Position: %2.3f,%2.3f,%2.3f",
        //     pose_msg.pose_1.pose_1.position.x,pose_msg.pose_1.pose_1.position.y,pose_msg.pose_1.pose_1.position.z
        //     );
        _1_t265_sdx_pub.publish(pose_msg);
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
