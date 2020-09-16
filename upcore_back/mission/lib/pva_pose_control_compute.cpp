//
// Created by wzy on 2020/9/1.
//
#include <mission_core.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <chrono>


using namespace Eigen;


inline double mission_core::max3_double(double a, double b, double c)
{
    double res = a > b ? a : b;
    res = res > c ? res : c;

    return res;
}


void mission_core::setPVA(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw)
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

    pvaTrack(pva_setpoint);
}


void mission_core::pva_calculate_and_pub_traj(Vector3d goal_p,Vector3d goal_v,Vector3d goal_a, double init_T){
    Eigen::MatrixXd p, v, a;
    Eigen::VectorXd t;

    double v_limit = 5.0;
    double a_limit = 2.0;
    double j_limit = 1.0;

    double delt_T = init_T; //total time
    double delt_t = 1.0/LOOPRATE; //interval of pub time

    /* Planning Parameters */
    Eigen::Vector3d new_goal_p0;
    Eigen::Vector3d p0 = curr_p;
    Eigen::Vector3d v0 = curr_v;
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
        cout << "new_goal_p0: " << new_goal_p0 << endl;
        compute_tracking_traj(p0, v0, a0, new_goal_p0, goal_v0, goal_a0, v_limit, a_limit, j_limit,
                              delt_T, delt_t, p, v, a, t);
        planning_t += 1.0/LOOPRATE;
    }

    ROS_WARN("get traj !! ");

    //! pub traj
    if(get_traj) {
        for (int i = 0; i < p.rows(); ++i) {
            ros::spinOnce();
            setPVA(p.row(i), v.row(i), a.row(i),yaw_rotate);
            rate.sleep();
        }
        ROS_WARN("out of pva trajectory, into follow mode");
    }
}

void mission_core::motion_primitives(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0,
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

void mission_core::compute_tracking_traj(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0,
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
        cout << "goal_p0: " << goal_p0 << endl;
        cout << "goal_v0: " << goal_v0 << endl;
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

void mission_core::pva_tracker_control() {
    Vector3d goal_p(x_pos_road_origin - distance_car_drone*cos(yaw_rotate),y_pos_road_origin - distance_car_drone*sin(yaw_rotate),home_hover_height);
    Vector3d goal_v(car_esti_vel[0],car_esti_vel[1],0);
    Vector3d goal_a(0,0,0);
    double T = 1.0;
    pva_calculate_and_pub_traj(goal_p,goal_v,goal_a,T);
}