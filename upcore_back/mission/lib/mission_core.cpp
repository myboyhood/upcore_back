//
// Created by wzy on 2020/9/1.
//

#include "mission_core.h"


mission_core::mission_core() : rate(LOOPRATE){

    //! >>>>>>>> service <<<<<<<<//
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");



    //! >>>>>>>> subscriber <<<<<<<<//
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &mission_core::state_cb ,this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom",1, &mission_core::odom_cb, this);
    start_trace_sub = nh.subscribe<std_msgs::Bool>("start_trace_topic",1, &mission_core::start_trace_cb, this);
    car_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/car_local_position",10,&mission_core::car_pos_cb,this);
    img_ir_sub = nh.subscribe<sensor_msgs::Image>("/camera/infra1/image_rect_raw",1,&mission_core::img_ir_cb,this);
    img_bgr_sub = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw",1,&mission_core::img_bgr_cb,this);

    //! >>>>>>>> ros publisher <<<<<<<< //
    px4_setpoint_position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    pva_setpoint_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    start_velocity_esti_pub = nh.advertise<std_msgs::Bool>("/start_velocity_esti_topic",1);
    pva_odom_sp_enu_pub = nh.advertise<nav_msgs::Odometry>("drone_pva_sp_enu",1);
    drone_vision_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("drone_vision_pos",1);
    start_trace_pub = nh.advertise<std_msgs::Bool>("start_trace_topic",1);
    //!>>>>>>>> test publisher <<<<<<<< //
    drone_euler_pub = nh.advertise<geometry_msgs::PoseStamped>("drone_euler_topic",1);
}

/**
 * callback function
 */
void mission_core::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state_msg = *msg;
}

void mission_core::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom_msg = *msg;
    curr_p << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z;
    curr_v << odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z;
}

void mission_core::car_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    car_pos[0] = msg->pose.position.x;
    car_pos[1] = msg->pose.position.y;
    car_pos[2] = msg->pose.position.z;
}

void mission_core::start_trace_cb(const std_msgs::Bool::ConstPtr& msg){
    start_trace_flag = msg->data;
}

void mission_core::img_ir_cb(const sensor_msgs::Image::ConstPtr& msg){

    img_ir_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
    img_ir = img_ir_ptr->image;


}

void mission_core::img_bgr_cb(const sensor_msgs::Image::ConstPtr& msg){
    img_bgr_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    img_bgr = img_bgr_ptr->image;
}

/**
 * process function
 */
void mission_core::cs_road_to_enu(double &x_road,double &y_road,double &x_enu,double &y_enu){
    x_enu = x_road * cos(yaw_rotate) - y_road * sin(yaw_rotate);
    y_enu = x_road * sin(yaw_rotate) + y_road * cos(yaw_rotate);
}

/**
 * 将欧拉角转化为四元数
 * @param roll
 * @param pitch
 * @param yaw
 * @return 返回四元数
 */
geometry_msgs::Quaternion mission_core::euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}
Eigen::Quaterniond mission_core::euler2quaternion_eigen(float roll, float pitch, float yaw)
{
    Eigen::Quaterniond temp;
    temp.w() = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x() = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y() = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z() = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

/**
 * 将四元数转化为欧拉角形式
 * @param x
 * @param y
 * @param z
 * @param w
 * @return 返回Vector3的欧拉角
 */
geometry_msgs::Vector3 mission_core::quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    // I use ENU coordinate system , so I plus ' - '
    temp.y = - asin(2.0 * (z * x - w * y));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}
