#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <Eigen/Eigen>
#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

//! ros
#define LOOPRATE 20
ros::Publisher traj_pub;
ros::Publisher change_follow_mode_pub;
ros::Subscriber change_attitude_mode;


//! global variable
std_msgs::Bool is_attitude_mode;
bool get_traj = false;
std_msgs::Bool change_follow_mode;
trajectory_msgs::JointTrajectory traj;
trajectory_msgs::JointTrajectoryPoint traj_point;
int traj_point_count = 0;


using namespace Eigen;
Vector3d current_position;
Vector3d current_velocity;



void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    /// ENU frame
    current_position << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    current_velocity << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
}

inline double max3_double(double a, double b, double c)
{
    double res = a > b ? a : b;
    res = res > c ? res : c;

    return res;
}

/* Function Declaration */
void motion_primitives(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0, 
                       Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Vector3d af, 
                       double T, double delt_t, int times, 
                       Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t, 
                       Eigen::Vector3d &max_v, Eigen::Vector3d &max_a);
void compute_tracking_traj(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0, 
                           Eigen::Vector3d goal_p0, Eigen::Vector3d goal_v0, Eigen::Vector3d goal_a0, 
                           double v_limit, double a_limit, double j_limit, double delt_T, double delt_t, 
                           Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t);
void pushback_traj_msg(const Eigen::MatrixXd &p, const Eigen::MatrixXd &v, const Eigen::MatrixXd &a, const Eigen::Vector3d &goal_p0);

void publish_traj_msg(int &i,double yaw = 0.0);
/* Function Declaration */



void mode_cb(const std_msgs::Bool::ConstPtr &msg)
{
    is_attitude_mode.data = msg->data;
}

void setPVA(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw=0.0)
{
    trajectory_msgs::JointTrajectoryPoint pva_setpoint;

    pva_setpoint.positions.push_back(p(0)); //x
    pva_setpoint.positions.push_back(p(1)); //y
    pva_setpoint.positions.push_back(p(2)); //z
    pva_setpoint.positions.push_back(yaw);

    pva_setpoint.velocities.push_back(v(0));
    pva_setpoint.velocities.push_back(v(1));
    pva_setpoint.velocities.push_back(v(2));

    pva_setpoint.accelerations.push_back(a(0));
    pva_setpoint.accelerations.push_back(a(1));
    pva_setpoint.accelerations.push_back(a(2));

    traj_pub.publish(pva_setpoint);
}

void calculate_and_pub_traj(Vector3d goal_p,Vector3d goal_v,Vector3d goal_a, double init_T = 5, ros::Rate lp = 20){
    Eigen::MatrixXd p, v, a;
    Eigen::VectorXd t;

    double v_limit = 2.0;
    double a_limit = 1.0;
    double j_limit = 1.0;

    double delt_T = init_T; //total time
    double delt_t = 1.0/LOOPRATE; //interval of pub time


    /* Planning Parameters */

    Eigen::Vector3d new_goal_p0;
    Eigen::Vector3d p0 = current_position;
    Eigen::Vector3d v0 = current_velocity;
    Eigen::Vector3d a0(0, 0, 0);

    Eigen::Vector3d goal_p0 = goal_p;
    Eigen::Vector3d goal_v0 = goal_v;
    Eigen::Vector3d goal_a0 = goal_a;

    //! calculate trajectory and pub
    get_traj = false;
    /* Planning Parameters */
    double planning_t = 0.0;
    while (!get_traj){
        new_goal_p0 = goal_p0 + goal_v0*planning_t;
        compute_tracking_traj(p0, v0, a0, new_goal_p0, goal_v0, goal_a0, v_limit, a_limit, j_limit,
                              delt_T, delt_t, p, v, a, t);
        planning_t += 1.0/LOOPRATE;
    }
    //! pub traj
    for (int i = 0; i < p.rows(); ++i) {
        setPVA(p.row(i), v.row(i), a.row(i));
        lp.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracking_test");
    ros::NodeHandle nh;

    traj_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);
    change_follow_mode_pub = nh.advertise<std_msgs::Bool>("change_to_follow_topic",1);
    change_attitude_mode = nh.subscribe<std_msgs::Bool>("change_to_attitude",1,mode_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, odomCallback);
    ros::Rate loop_rate(LOOPRATE);

    // // Test part
    // Eigen::Vector3d v(-1.0, -2.5, 3.0);
    // ROS_INFO("v is: %lf, %lf, %lf", v.cwiseAbs()(0), v.cwiseAbs()(1), v.cwiseAbs()(2));
    // ROS_INFO("v max elem is: %lf", v.cwiseAbs().maxCoeff());


    is_attitude_mode.data = false;
    // wait change mode in loop
    while (ros::ok() && !is_attitude_mode.data){
        std::cout << "position control , computing process: wait for attitude mode" << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }

    Vector3d g_p0(0,8,3);
    Vector3d g_v0(1,0,0);
    Vector3d g_a0(0,0,0);
    calculate_and_pub_traj(g_p0,g_v0,g_a0,5,loop_rate);


    Vector3d g_p1(current_position(0)+3,8,3);
    Vector3d g_v1(0,0,0);
    Vector3d g_a1(0,0,0);
    calculate_and_pub_traj(g_p1,g_v1,g_a1,2,loop_rate);

    //! hover calculate


//    for (int i = 0; i < p.rows(); ++i) {
//
//        auto start_t = std::chrono::steady_clock::now();
//
//        publish_traj_msg(i);
//
//        auto end_t = std::chrono::steady_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t);
//        std::cout << "for loop time: " << duration.count() << std::endl;
//
//        loop_rate.sleep();
//        ros::spinOnce();
//
//    }

//    change_follow_mode.data = false;
//    while (ros::ok() && !change_follow_mode.data) {
//        auto start_t = std::chrono::steady_clock::now();
//        publish_traj_msg();
//
//        ros::spinOnce();
//        loop_rate.sleep();
//        auto end_t = std::chrono::steady_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t);
//        std::cout << "publish_traj_msg time: " << duration.count() << std::endl;
//    }


//    change_follow_mode.data = true;
    while (ros::ok()) {
//        change_follow_mode_pub.publish(change_follow_mode);
        std::cout << "end of computing traj_point, enter idle mode ..." << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

//void pushback_traj_msg(const Eigen::MatrixXd &p, const Eigen::MatrixXd &v, const Eigen::MatrixXd &a, const Eigen::Vector3d &goal_p0)
//{
//    /* First push back the postion of car */
//    trajectory_msgs::JointTrajectoryPoint goal_pt;
//    goal_pt.positions.push_back(goal_p0(0));
//    goal_pt.positions.push_back(goal_p0(1));
//    goal_pt.positions.push_back(goal_p0(2));
//
//    traj.points.push_back(goal_pt);
//
//    /* Then trajectory points */
//    for (int i = 0; i < p.rows(); ++i) {
//        trajectory_msgs::JointTrajectoryPoint traj_pt;
//        traj_pt.positions.push_back(p(i, 0));
//        traj_pt.positions.push_back(p(i, 1));
//        traj_pt.positions.push_back(p(i, 2));
//        traj_pt.velocities.push_back(v(i, 0));
//        traj_pt.velocities.push_back(v(i, 1));
//        traj_pt.velocities.push_back(v(i, 2));
//        traj_pt.accelerations.push_back(a(i, 0));
//        traj_pt.accelerations.push_back(a(i, 1));
//        traj_pt.accelerations.push_back(a(i, 2));
//
//        traj.points.push_back(traj_pt);
//    }
////    traj_pub.publish(traj);
//}


//void publish_traj_msg(int &i, double yaw){
//    auto start_t = std::chrono::steady_clock::now();
//    trajectory_msgs::JointTrajectoryPoint traj_point_msg;
////    std::cout << "the current traj_point is: " << traj_point_count << std::endl;
//
////        std::cout << "p.rows(): " << p.rows() << std::endl;
//        traj_point_msg.positions.push_back(p(i, 0));
//        traj_point_msg.positions.push_back(p(i, 1));
//        traj_point_msg.positions.push_back(p(i, 2));
//        yaw = 0.0;
//        traj_point_msg.positions.push_back(yaw);
//
//        traj_point_msg.velocities.push_back(v(i, 0));
//        traj_point_msg.velocities.push_back(v(i, 1));
//        traj_point_msg.velocities.push_back(v(i, 2));
//        traj_point_msg.accelerations.push_back(a(i, 0));
//        traj_point_msg.accelerations.push_back(a(i, 1));
//        traj_point_msg.accelerations.push_back(a(i, 2));
////        std::cout << "p 0 1 2:" << std::endl
////                  << p(traj_point_count, 0) << " " << p(traj_point_count, 1) << " " << p(traj_point_count, 2) << std::endl
////                  << "v 0 1 2:" << std::endl
////                  << v(traj_point_count, 0) << " " << v(traj_point_count, 1) << " " << v(traj_point_count, 2) << std::endl
////                  << "a 0 1 2:" << std::endl
////                  << a(traj_point_count, 0) << " " << a(traj_point_count, 1) << " " << a(traj_point_count, 2)
////                  << std::endl;
//
//    change_follow_mode_pub.publish(change_follow_mode);
//    traj_pub.publish(traj_point_msg);
//    auto end_t = std::chrono::steady_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t);
//    std::cout << "publish_traj_msg time: " << duration.count() << std::endl;
//
//}

//void publish_traj_msg(){
////    auto start_t = std::chrono::steady_clock::now();
//    trajectory_msgs::JointTrajectoryPoint traj_point_msg;
////    std::cout << "the current traj_point is: " << traj_point_count << std::endl;
//
//    if(traj_point_count < p.rows()) {
////        std::cout << "p.rows(): " << p.rows() << std::endl;
//        traj_point_msg.positions.push_back(p(traj_point_count, 0));
//        traj_point_msg.positions.push_back(p(traj_point_count, 1));
//        traj_point_msg.positions.push_back(p(traj_point_count, 2));
//        traj_point_msg.velocities.push_back(v(traj_point_count, 0));
//        traj_point_msg.velocities.push_back(v(traj_point_count, 1));
//        traj_point_msg.velocities.push_back(v(traj_point_count, 2));
//        traj_point_msg.accelerations.push_back(a(traj_point_count, 0));
//        traj_point_msg.accelerations.push_back(a(traj_point_count, 1));
//        traj_point_msg.accelerations.push_back(a(traj_point_count, 2));
////        std::cout << "p 0 1 2:" << std::endl
////                  << p(traj_point_count, 0) << " " << p(traj_point_count, 1) << " " << p(traj_point_count, 2) << std::endl
////                  << "v 0 1 2:" << std::endl
////                  << v(traj_point_count, 0) << " " << v(traj_point_count, 1) << " " << v(traj_point_count, 2) << std::endl
////                  << "a 0 1 2:" << std::endl
////                  << a(traj_point_count, 0) << " " << a(traj_point_count, 1) << " " << a(traj_point_count, 2)
////                  << std::endl;
//        traj_point_count += 1;
//    }
//    else{
//        traj_point_msg.positions.push_back(p(traj_point_count-1, 0));
//        traj_point_msg.positions.push_back(p(traj_point_count-1, 1));
//        traj_point_msg.positions.push_back(p(traj_point_count-1, 2));
//        traj_point_msg.velocities.push_back(v(traj_point_count-1, 0));
//        traj_point_msg.velocities.push_back(v(traj_point_count-1, 1));
//        traj_point_msg.velocities.push_back(v(traj_point_count-1, 2));
//        traj_point_msg.accelerations.push_back(a(traj_point_count-1, 0));
//        traj_point_msg.accelerations.push_back(a(traj_point_count-1, 1));
//        traj_point_msg.accelerations.push_back(a(traj_point_count-1, 2));
////        change_follow_mode.data = true;
//    }
//
//
//    change_follow_mode_pub.publish(change_follow_mode);
//    traj_pub.publish(traj_point_msg);
////    auto end_t = std::chrono::steady_clock::now();
////    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t);
////    std::cout << "publish_traj_msg time: " << duration.count() << std::endl;
//}

void motion_primitives(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0, 
                       Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Vector3d af, 
                       double T, double delt_t, int times, 
                       Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t, 
                       Eigen::Vector3d &max_v, Eigen::Vector3d &max_a)
{
    // % calculate optimal jerk controls by Mark W. Miller
    for(int ii=0; ii<3; ii++)
    {
        double delt_a = af(ii) - a0(ii);
        double delt_v = vf(ii) - v0(ii) - a0(ii)*T;
        double delt_p = pf(ii) - p0(ii) - v0(ii)*T - 0.5*a0(ii)*T*T;

        //%  if vf is not free
        double alpha = delt_a*60/pow(T,3) - delt_v*360/pow(T,4) + delt_p*720/pow(T,5);
        double beta = -delt_a*24/pow(T,2) + delt_v*168/pow(T,3) - delt_p*360/pow(T,4);
        double gamma = delt_a*3/T - delt_v*24/pow(T,2) + delt_p*60/pow(T,3);

        for(int jj=0; jj<times; jj++)
        {
            double tt = (jj + 1)*delt_t;
            t(jj) = tt;
            p(jj,ii) = alpha/120*pow(tt,5) + beta/24*pow(tt,4) + gamma/6*pow(tt,3) + a0(ii)/2*pow(tt,2) + v0(ii)*tt + p0(ii);
            v(jj,ii) = alpha/24*pow(tt,4) + beta/6*pow(tt,3) + gamma/2*pow(tt,2) + a0(ii)*tt + v0(ii);
            a(jj,ii) = alpha/6*pow(tt,3) + beta/2*pow(tt,2) + gamma*tt + a0(ii);

            if (fabs(v(jj, ii)) > max_v(ii)) max_v(ii) = fabs(v(jj, ii));
            if (fabs(a(jj, ii)) > max_a(ii)) max_a(ii) = fabs(a(jj, ii));
        }
    }
}

void compute_tracking_traj(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0, 
                           Eigen::Vector3d goal_p0, Eigen::Vector3d goal_v0, Eigen::Vector3d goal_a0, 
                           double v_limit, double a_limit, double j_limit, double delt_T, double delt_t, 
                           Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t)
{
    Eigen::Vector3d delt_p0 = goal_p0 - p0;
    Eigen::Vector3d delt_v0 = goal_v0 - v0;
    Eigen::Vector3d delt_a0 = goal_a0 - a0;

    double T1 = delt_a0.cwiseAbs().maxCoeff()/j_limit;
    double T2 = delt_v0.cwiseAbs().maxCoeff()/a_limit;
    double T3 = delt_p0.cwiseAbs().maxCoeff()/v_limit;
    // TODO: Is a min limit for T required?
    double T = max3_double(T1, T2, T3);
    int times;
    Eigen::Vector3d goal_pf, goal_vf, goal_af, max_v, max_a;

    // Set max iteration time (millisecond)
    double max_ite_t = 100;
    auto ite_start_t = std::chrono::steady_clock::now();

    while (true) {
        // Compute final state of goal
        goal_af = goal_a0;
        goal_vf = goal_v0 + goal_a0*T;
        goal_pf = goal_p0 + goal_v0*T + 0.5*goal_a0*T*T;

        // Reset max_v and max_a;
        max_v = Eigen::Vector3d::Zero();
        max_a = Eigen::Vector3d::Zero();

        ROS_INFO("goal_pf: %lf, %lf", goal_pf(0), goal_pf(1));

        times = T/delt_t;
        ROS_INFO("times: %lf,%lf,%d", T,delt_t, times);
        p = Eigen::MatrixXd::Zero(times, 3);
        v = Eigen::MatrixXd::Zero(times, 3);
        a = Eigen::MatrixXd::Zero(times, 3);
        t = Eigen::VectorXd::Zero(times);

        ROS_INFO("here");

        motion_primitives(p0, v0, a0, goal_pf, goal_vf, goal_af, T, delt_t, times, p, v, a, t, max_v, max_a);

        ROS_INFO("max_v: %lf, %lf, %lf", max_v(0), max_v(1), max_v(2));
        ROS_INFO("max_a: %lf, %lf, %lf", max_a(0), max_a(1), max_a(2));

        /* If max iteration time reached, break */
        auto ite_current_t = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(ite_current_t - ite_start_t);
        double ite_t = duration.count()/(double)1000;

        if (ite_t > max_ite_t) {
            ROS_WARN("Max iteration time reached!");
            break;
        }

        /* If the trajectory is not dynamically feasible, continue */
        if (max_a.maxCoeff() > a_limit || max_v.maxCoeff() > v_limit) {
            T += delt_T;
            continue;
        } else {
            get_traj = true;
            // Trajectory is dynamically feasible
            break;
        }
    }
}
