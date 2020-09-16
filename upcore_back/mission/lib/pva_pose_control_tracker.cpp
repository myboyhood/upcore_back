//
// Created by wzy on 2020/9/2.
//

#include <mission_core.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define GRAVITATIONAL_ACC 9.81

using namespace Eigen;
// Global Variables
double p_i_acc_error_limit = 0.25;
double v_i_acc_error_limit = 0.2;
double thrust_factor = 0.08;
//double thrust_factor = 0.025;


Vector3d vectorElementMultiply(Vector3d v1, Vector3d v2)
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

void mission_core::pvaTrack(trajectory_msgs::JointTrajectoryPoint &msg)
{
    Vector3d planned_p;
    Vector3d planned_v;
    Vector3d planned_a;
    double planned_yaw;
    Quaterniond current_att;
//    /// NWU frame to ENU frame
//    planned_p << -msg->positions[1], msg->positions[0], msg->positions[2];
//    planned_yaw = msg->positions[3] + M_PI/2.0;
//    planned_v << -msg->velocities[1], msg->velocities[0], msg->velocities[2];
//    planned_a << -msg->accelerations[1], msg->accelerations[0], msg->accelerations[2];

    planned_p << msg.positions[0], msg.positions[1], msg.positions[2];
    planned_yaw = msg.positions[3];
    planned_v << msg.velocities[0], msg.velocities[1], msg.velocities[2];
    planned_a << msg.accelerations[0], msg.accelerations[1], msg.accelerations[2];
    ROS_INFO("P: %f, %f, %f",msg.positions[0],msg.positions[1],msg.positions[2]);

    /// Publish to record in rosbag

    odom_sp_enu.header.stamp = ros::Time::now();
    odom_sp_enu.pose.pose.position.x = planned_p(0);
    odom_sp_enu.pose.pose.position.y = planned_p(1);
    odom_sp_enu.pose.pose.position.z = planned_p(2);
    odom_sp_enu.twist.twist.linear.x = planned_v(0);
    odom_sp_enu.twist.twist.linear.y = planned_v(1);
    odom_sp_enu.twist.twist.linear.z = planned_v(2);
    pva_odom_sp_enu_pub.publish(odom_sp_enu);

    /// Calculate desired thrust and attitude
    Vector3d p_error = planned_p - curr_p;
    Vector3d v_error = planned_v - curr_v;

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
            vectorElementMultiply(p_error, pp) + vectorElementMultiply(v_error, vp) +
            vectorElementMultiply(delt_p_error, pd) + vectorElementMultiply(delt_v_error, vd) +
            vectorElementMultiply(p_error_accumulate, pi) + vectorElementMultiply(v_error_accumulate, vi);

    // Set a maximum acceleration feedforward value given by position and velocity error.
    for(int i=0; i<3; i++){
        if(fabs(a_fb(i)) > 8.0) a_fb(i) = 8.0 * a_fb(i) / fabs(a_fb(i));
    }

    p_error_last = p_error;
    v_error_last = v_error;

    Vector3d z_w_norm(0, 0, 1.0);
    Vector3d a_des = a_fb + planned_a + GRAVITATIONAL_ACC * z_w_norm;

    Vector3d att_des_norm = a_des / a_des.norm();

    Quaterniond z_w_quat(0, 0, 0, 1.0);
    Quaterniond att_current_vector_quat = current_att * z_w_quat * current_att.inverse();
    Vector3d current_att_vector(att_current_vector_quat.x(), att_current_vector_quat.y(), att_current_vector_quat.z());

//    Quaterniond att_des_q = Quaterniond::FromTwoVectors(current_att_vector, att_des_norm);
    Quaterniond att_des_q = Quaterniond::FromTwoVectors(z_w_norm, att_des_norm);

    //add yaw
    Quaterniond yaw_quat(cos(planned_yaw/2.0), att_des_norm(0)*sin(planned_yaw/2.0),
                         att_des_norm(1)*sin(planned_yaw/2.0),att_des_norm(2)*sin(planned_yaw/2.0));
    att_des_q = yaw_quat * att_des_q;

    //Calculate thrust
    double thrust_des = a_des.norm() * thrust_factor;  //a_des.dot(att_current_vector) * THRUST_FACTOR

    /**End of Core code**/

    att_setpoint.header.stamp = ros::Time::now();
    att_setpoint.orientation.w = att_des_q.w();
    att_setpoint.orientation.x = att_des_q.x();
    att_setpoint.orientation.y = att_des_q.y();
    att_setpoint.orientation.z = att_des_q.z();
    att_setpoint.thrust = thrust_des;
    ROS_INFO_THROTTLE(1.0, "Attitude Quaternion Setpoint is w=%f, x=%f, y=%f, z=%f, thrust=%f", att_setpoint.orientation.w,
                      att_setpoint.orientation.x, att_setpoint.orientation.y, att_setpoint.orientation.z, att_setpoint.thrust);

    pva_setpoint_attitude_pub.publish(att_setpoint);
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


//int main(int argc, char** argv) {
//    ros::init(argc, argv, "tracker");
//
//    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig> server;
//    dynamic_reconfigure::Server<tracker::PVA_TrackerConfig>::CallbackType f;
//    f = boost::bind(&configureCallback, _1, _2);
//    server.setCallback(f);
//
//    ros::NodeHandle nh;
//
//    ros::Subscriber pva_sub = nh.subscribe("/pva_setpoint", 1, pvaCallback);
//    ros::Subscriber position_sub = nh.subscribe("/mavros/local_position/pose", 1, positionCallback);
//    ros::Subscriber velocity_sub = nh.subscribe("/mavros/local_position/velocity_local", 1, velocityCallback);
//
//    att_ctrl_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
//    odom_sp_enu_pub = nh.advertise<nav_msgs::Odometry>("/odom_sp_enu", 1);
//
//    ros::spin();
//    return 0;
//}