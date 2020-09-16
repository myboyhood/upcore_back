//
// Created by wzy on 2020/9/1.
//

#include "mission_core.h"

using namespace Eigen;

int main(int argc, char **argv) {

    ros::init(argc, argv, "mission_node");
    mission_core laser; //定义一个laser对象

    /**
     * param set
     */
    if(!laser.setParam())
        return 0;



    /**
     * auto_takeoff
     */
    laser.auto_takeoff();

    /**
     * yaw rotate
     */
    laser.yaw_rotate_process();

    /**
     * esti_car_velocity
     */
     laser.esti_car_velocity();

    /**
     * pva_tracker
     */
    ROS_INFO("!!!");
    laser.pva_tracker_control();

    /**
     * local follow mode,如果没有计算出合适轨迹，则执行px4_traj_track_control去追踪，
     * 否则，等pva_tracker也是继续用px4_traj_track_control追踪
     */
     laser.is_back_flag = true;
    if (!laser.get_traj) {
        ROS_WARN("can not get feasible trajectory... instead by px4_traj_track !! ");
        laser.px4_traj_track_control(laser.is_back_flag);
    } else {
        laser.px4_traj_track_control(laser.is_back_flag);
    }

    /**
     * pnp detect and follow
     */


    /**
     * homeward_voyage
     */
    laser.backhome();


    return 0;
}

