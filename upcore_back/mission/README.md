## Note
1. change `param file path` in `mission_node_gazebo.cpp` 或者 `mission_node_outdoor.cpp`

2. 车的偏航角应当事前测量好，写入程序的`mission_core.h`中的`yaw_rotate`以及`xbee_pose_recv_test_gazebo.py`中yaw的
偏角，起跑点`road_origin`也要在两个文件写好。
3. 在无人机起飞并调整好偏航角之后，
将车辆驶过起跑线之前的10m的阶段用作估计车辆速度的阶段，速度信息反映在`car_local_position`
的变化，用头和尾的时间相减算出车辆速度。
4. `mission_core`中，飞机起飞点坐标用`x_pos_home_origin`,`y_pos_home_origin`表示，不考虑起飞点的
轻微漂移。
车辆起跑点用`x_pos_road_origin`和`y_pos_road_origin`表示，其值是以飞机起飞点为坐标原点的ENU坐标系下的
坐标。
5. 在`mission_node_outdoor.cpp`中是车的虚拟点在跑，不需要小车。在`mission_node_outdoor_with_carGPS.cpp`
中需要现将小车的速度测定好，yaw角设定好。输入主程序，将小车放在pva_tacker的结束点位置，遥控使用。并设置
`maxtime`来调整GPS跟踪小车这一段的时间。
6. 在室外实验过程中发现起飞位置点可能不是（0,0,0），这样的话，小车的轨迹也不能是依照（0,0,0）固定好
路线，在pva_tracker过程结束时，应当把小车的home_position位置加到car_pos里。
7. 我在文件中忘记写固定红色usb_ttl端口和启动mavros的launch程序了，在到实际机载电脑里记得先开启
`gps_control`里面的`gps_mavros.launch`
8. 一定要事先确定好飞机悬停油门值，来改变pva_tracker里面的`thrust_factor`，使得
悬停油门值=`GRAVITATIONAL_ACC`*`thrust_factor`

9. camera param fx , fy * 2, and tf_camera_drone : -0.30 -0.025 0.11