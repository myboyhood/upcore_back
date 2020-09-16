//
// Created by cc on 2020/8/4.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>
//#include <dynamic_reconfigure/server.h>
//#include <pva_tracker/PVA_TrackerConfig.h>
#include <nav_msgs/Odometry.h>
//#include <mav_msgs/conversions.h>
//#include <mav_msgs/default_topics.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <gps_control/Parameter.h>


#define GRAVITATIONAL_ACC 9.81

using namespace Eigen;

// Coefficients
Vector3d position_error_p;
Vector3d position_error_d;
Vector3d position_error_i;
Vector3d velocity_error_p;
Vector3d velocity_error_d;
Vector3d velocity_error_i;

double p_i_acc_error_limit = 0.6;
double v_i_acc_error_limit = 0.5;

// Global Variables
Vector3d planned_p;
Vector3d planned_v;
Vector3d planned_a;
double planned_yaw;
Vector3d current_p;
Vector3d current_v;
Quaterniond current_att;
ros::Publisher att_ctrl_pub, odom_sp_ned_pub;
double thrust_factor = 0.05;
std_msgs::Bool is_attitude_mode;
std_msgs::Bool is_follow_mode;
int pva_count = 0;
Parameter param;
// bool coord_test_flag = false;

Vector3d vectorElementMultiply(Vector3d v1, Vector3f v2)
{
    Vector3d result;
    result << v1(0)*v2(0), v1(1)*v2(1), v1(2)*v2(2);
    return result;
}

void vector3dLimit(Vector3d &v, double limit)  ///limit should be positive
{
    if(limit > 0){
        for(int i=0; i<3; i++){
            v(i) = fabs(v(i)) > limit ? (v(i) > 0 ? limit : -limit) : v(i);
        }
    }
}

void pvaCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
{   pva_count += 1;
    // mavros_msgs::AttitudeTarget att_setpoint;

    // /// NWU frame to ENU frame
    // planned_p << -msg->positions[1], msg->positions[0], msg->positions[2];
    // planned_yaw = msg->positions[3];
    // planned_v << -msg->velocities[1], msg->velocities[0], msg->velocities[2];
    // planned_a << -msg->accelerations[1], msg->accelerations[0], msg->accelerations[2];

    // No need to transform coordinates in Gazebo
    planned_p << msg->positions[0], msg->positions[1], msg->positions[2];
    planned_yaw = msg->positions[3];
    planned_v << msg->velocities[0], msg->velocities[1], msg->velocities[2];
    planned_a << msg->accelerations[0], msg->accelerations[1], msg->accelerations[2];

    ROS_INFO("Planned_yaw: %lf", planned_yaw);

    // ROS_WARN("Position x: %lf\n", msg->positions[0]);

    nav_msgs::Odometry odom_sp_enu;
    odom_sp_enu.header.stamp = ros::Time::now();
    odom_sp_enu.pose.pose.position.x = planned_p(0);
    odom_sp_enu.pose.pose.position.y = planned_p(1);
    odom_sp_enu.pose.pose.position.z = planned_p(2);
    odom_sp_enu.twist.twist.linear.x = planned_v(0);
    odom_sp_enu.twist.twist.linear.y = planned_v(1);
    odom_sp_enu.twist.twist.linear.z = planned_v(2);
    odom_sp_ned_pub.publish(odom_sp_enu);

    /// Calculate desired thrust and attitude
    Vector3d p_error = planned_p - current_p;
    Vector3d v_error = planned_v - current_v;
    std::cout << "planned_p: " << planned_p[0] << " " << planned_p[1] << " "<< planned_p[2] << std::endl;
    std::cout << "planned_v: " << planned_v[0] << " " << planned_v[1] << " "<< planned_v[2] << std::endl;
    std::cout << "planned_a: " << planned_a[0] << " " << planned_a[1] << " "<< planned_a[2] << std::endl;
    std::cout << "current_p: " << current_p[0] << " " << current_p[1] << " "<< current_p[2] << std::endl;
    static Vector3d p_error_last;
    static Vector3d v_error_last;
    static Vector3d p_error_accumulate;
    static Vector3d v_error_accumulate;
    static bool if_init = true;

    if(if_init){
        if_init = false;
        p_error_last = p_error;
        v_error_last = v_error;
        p_error_accumulate = p_error;
        v_error_accumulate = v_error;
        return;
    }

    /**Core code**/
    Vector3d delt_p_error = p_error - p_error_last;
    Vector3d delt_v_error = v_error - v_error_last;

    p_error_accumulate += p_error;
    v_error_accumulate += v_error;
    vector3dLimit(p_error_accumulate, p_i_acc_error_limit);
    vector3dLimit(v_error_accumulate, v_i_acc_error_limit);

    Vector3d a_fb =   /// PID
            vectorElementMultiply(p_error, param.pp) + vectorElementMultiply(v_error, param.vp) +
            vectorElementMultiply(delt_p_error, param.pd) + vectorElementMultiply(delt_v_error, param.vd) +
            vectorElementMultiply(p_error_accumulate, param.pi) + vectorElementMultiply(v_error_accumulate, param.vi);

    p_error_last = p_error;
    v_error_last = v_error;
    std::cout << "a_fb: " << a_fb[0] << a_fb[1] << a_fb[2] << std::endl;
    Vector3d z_w_norm(0, 0, 1.0);
    Vector3d a_des = a_fb + planned_a + GRAVITATIONAL_ACC * z_w_norm;
    ROS_INFO("a_des[0]: %lf",a_des[0]);
    ROS_INFO("a_des[1]: %lf",a_des[1]);
    ROS_INFO("a_des[2]: %lf",a_des[2]);
    Vector3d att_des_norm = a_des / a_des.norm();
    ROS_INFO("a_des.norm: %lf",a_des.norm());

    // In RotorS, the coordinate is fixed with the drone (tentative?)
    // Quaterniond att_des = Quaterniond::FromTwoVectors(z_w_norm, att_des_norm);

    //add yaw
    // Quaterniond yaw_quat(cos(planned_yaw/2.0), att_des_norm(0)*sin(planned_yaw/2.0),
    //         att_des_norm(1)*sin(planned_yaw/2.0),att_des_norm(2)*sin(planned_yaw/2.0));
    // att_des = yaw_quat * att_des;

    //Calculate thrust
    Quaterniond z_w_quat(0, 0, 0, 1.0);
    Quaterniond att_current_vector_quat = current_att * z_w_quat * current_att.inverse();
    Vector3d att_current_vector(att_current_vector_quat.x(), att_current_vector_quat.y(),
                                att_current_vector_quat.z());

    Quaterniond att_des = Quaterniond::FromTwoVectors(att_current_vector, att_des_norm);

    double thrust_des = a_des.norm() * thrust_factor;  //a_des.dot(att_current_vector) * THRUST_FACTOR

    /**End of Core code**/

//    mav_msgs::RollPitchYawrateThrust att_setpoint;
    mavros_msgs::AttitudeTarget att_setpoint;

    tf::Quaternion q(att_des.x(), att_des.y(), att_des.z(), att_des.w());
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << "pva_count: " << pva_count << "roll: " << roll << "pitch: " << pitch << "yaw: " << yaw << std::endl;
    // Compute current yaw
    tf::Quaternion c_q(current_att.x(), current_att.y(), current_att.z(), current_att.w());
    tf::Matrix3x3 c_m(c_q);
    double c_roll, c_pitch, c_yaw;
    c_m.getRPY(c_roll, c_pitch, c_yaw);

    ROS_WARN("yaw & current yaw: %lf, %lf", yaw, c_yaw);

    double yaw_rate = planned_yaw - c_yaw;
    while (yaw_rate > M_PI) yaw_rate -= 2*M_PI;
    while (yaw_rate < -M_PI) yaw_rate += 2*M_PI;

    ROS_WARN("yaw rate: %lf", yaw_rate);

    // Normalized yaw_rate --> (-1, 1)
    yaw_rate /= M_PI;
    double yaw_rate_amp = 2.0;

//    att_setpoint.header.stamp = ros::Time::now();
//    att_setpoint.roll = roll;
//    att_setpoint.pitch = pitch;
//    // att_setpoint.roll = 0.0;
//    // att_setpoint.pitch = 0.0;
//    // att_setpoint.yaw_rate = yaw_rate * yaw_rate_amp;
//    att_setpoint.yaw_rate = 0.0;
//    att_setpoint.thrust.x = 0.0;
//    att_setpoint.thrust.y = 0.0;
//    att_setpoint.thrust.z = thrust_des*30;

    // if (c_yaw > M_PI/2.0) coord_test_flag = true;
    // coord_test_flag = true;

    // if (coord_test_flag) {
    //     att_setpoint.roll = roll;
    //     att_setpoint.pitch = pitch;
    //     att_setpoint.yaw_rate = 0.0;
    // }

     att_setpoint.header.stamp = ros::Time::now();
     att_setpoint.orientation.w = att_des.w();
     att_setpoint.orientation.x = att_des.x();
     att_setpoint.orientation.y = att_des.y();
     att_setpoint.orientation.z = att_des.z();
     att_setpoint.thrust = thrust_des;

    // ROS_INFO_THROTTLE(1.0, "Attitude Quaternion Setpoint is w=%f, x=%f, y=%f, z=%f, thrust=%f", att_setpoint.orientation.w,
    //         att_setpoint.orientation.x, att_setpoint.orientation.y, att_setpoint.orientation.z, att_setpoint.thrust);

    att_ctrl_pub.publish(att_setpoint);
}


//void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
//{
//    /// ENU frame
//    current_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//    current_att.w() = msg->pose.orientation.w;
//    current_att.x() = msg->pose.orientation.x;
//    current_att.y() = msg->pose.orientation.y;
//    current_att.z() = msg->pose.orientation.z;
//}
//
//void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
//{
//    /// ENU frame
//    current_v << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
//}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    /// ENU frame
    current_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    current_att.w() = msg->pose.pose.orientation.w;
    current_att.x() = msg->pose.pose.orientation.x;
    current_att.y() = msg->pose.pose.orientation.y;
    current_att.z() = msg->pose.pose.orientation.z;
    current_v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
}


//void configureCallback(tracker::PVA_TrackerConfig &config, uint32_t level) {
//    position_error_p << config.position_p_xy, config.position_p_xy, config.position_p_z;
//    position_error_d << config.position_d_xy, config.position_d_xy, config.position_d_z;
//    position_error_i << config.position_i_xy, config.position_i_xy, config.position_i_z;
//    p_i_acc_error_limit = config.p_i_acc_error_limit;
//
//    velocity_error_p << config.velocity_p_xy, config.velocity_p_xy, config.velocity_p_z;
//    velocity_error_d << config.velocity_d_xy, config.velocity_d_xy, config.velocity_d_z;
//    velocity_error_i << config.velocity_i_xy, config.velocity_i_xy, config.velocity_i_z;
//    v_i_acc_error_limit = config.v_i_acc_error_limit;
//    thrust_factor = config.hover_thrust_factor;
//}

void mode_cb(const std_msgs::Bool::ConstPtr &msg)
{
    is_attitude_mode.data = msg->data;
}


void change_to_follow_cb(const std_msgs::Bool::ConstPtr &msg)
{
    is_follow_mode.data = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracker");

//    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig> server;
//    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig>::CallbackType f;
//    f = boost::bind(&configureCallback, _1, _2);
//    server.setCallback(f);

    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    ros::Subscriber pva_sub = nh.subscribe("/pva_setpoint", 1, pvaCallback);
//    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/firefly/ground_truth/odometry", 1, odomCallback);
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, odomCallback);
    ros::Subscriber change_attitude_mode = nh.subscribe<std_msgs::Bool>("change_to_attitude",1,mode_cb);
    ros::Subscriber change_to_follow = nh.subscribe<std_msgs::Bool>("change_to_follow_topic",1,change_to_follow_cb);

    att_ctrl_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);

    // att_ctrl_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);
//    att_ctrl_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>("/firefly/command/roll_pitch_yawrate_thrust", 1);
    odom_sp_ned_pub = nh.advertise<nav_msgs::Odometry>("/odom_sp_ned", 1);

    //! read param file
    std::string paramadr("/home/up/catkin_ws/src/gps_control/param/param2.txt");
    if(param.readParam(paramadr.c_str()) == 0)
    {
        std::cout<<"read config file error!"<<std::endl;
        return 0;
    }

    is_attitude_mode.data = false;
    while (ros::ok() && !is_attitude_mode.data)
    {
        std::cout << "position control , computing process: wait for attitude mode" << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }
    is_follow_mode.data = false;
    while (ros::ok() && !is_follow_mode.data){
        std::cout << "attitude control " << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }

    while (ros::ok()){
        std::cout << "follow control ,tracker enter idle..." << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }
//    ros::spin();
    return 0;
}
