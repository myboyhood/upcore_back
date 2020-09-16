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
//     //不能完整去跑，所以自己估计车辆速度先写入文件。
   laser.length_of_velocity_esti = 10;
   laser.vel_time = 10;
   laser.car_esti_vel[0] = laser.length_of_velocity_esti*cos(laser.yaw_rotate)/laser.vel_time;
   cout << "vel_time: " << laser.vel_time << endl;
   cout << "car_esti_vel[0]: " << laser.car_esti_vel[0] << endl;
   laser.car_esti_vel[1] = laser.length_of_velocity_esti*sin(laser.yaw_rotate)/laser.vel_time;

   laser.start_trace_msg.data = true;
    for (int i = 0; i < 50; ++i) {
        ROS_WARN("pub start_trace topic !!");
        ros::spinOnce();
        laser.start_trace_pub.publish(laser.start_trace_msg);//小车开始向前跑
        laser.px4_setpoint_position_pub.publish(laser.px4_position_msg); //不需要更新，继续按旧的值去发布
        laser.rate.sleep();
    }
    // laser.esti_car_velocity();




    /**
     * pva_tracker
     */
     ROS_INFO("!!!");
     laser.pva_tracker_control();
//    laser.get_traj = false;
    /**
     * local follow mode,如果没有计算出合适轨迹，则执行px4_traj_track_control去追踪，
     * 否则，等pva_tracker也是继续用px4_traj_track_control追踪
     */
//    laser.car_pos_bias = Vector3d(laser.x_pos_home_origin+laser.x_pos_road_origin,laser.y_pos_home_origin+laser.y_pos_road_origin,0);//室外的飞机起飞点不是原点时，需要把x_pos_home_origin和y_pos_home_origin的偏置量加到car_pos,仅仅是单独飞轨迹时使用，当真正使用车辆GPS数据估计速度时，还是要把飞机从原点（0,0,0）起飞

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



