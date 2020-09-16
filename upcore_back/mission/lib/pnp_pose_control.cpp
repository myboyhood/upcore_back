//
// Created by wzy on 2020/9/1.
//
#include <mission_core.h>

using namespace cv;
using namespace std;



bool mission_core::vision_pose_acqurie() {
    pnp_pose_is_good = false;
    threshold(img_ir,ir_binary,230,255,THRESH_BINARY);
    ROS_INFO("threshold complete");
    imshow("ir_binary",ir_binary);
    int structElementSize = 1;
    //!dilate
    Mat dilatestructElement = getStructuringElement(MORPH_RECT,
                                                   Size(3 * structElementSize + 1, 3 * structElementSize + 1));
    dilate(ir_binary,ir_dilate,dilatestructElement);
    ROS_INFO("dilate complete");
    findContours(ir_dilate,ir_contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);
    ROS_INFO("findcontours complete");
    cvtColor(ir_binary,ir_color,COLOR_GRAY2BGR);
    drawContours(ir_color,ir_contours,-1,(50, 100, 250), -1);
    imshow("contours",ir_color);
    waitKey(1);
    if(ir_contours.size() == 4){
        cout << "4 contours" << endl;
        //find each point and center
        marker_sum.x = 0;
        marker_sum.y = 0;
        marker_pixels.clear();
        for (int i = 0; i < ir_contours.size(); i++){
            Rect bbox;
            bbox = boundingRect(ir_contours[i]);
            marker_pixels.emplace_back(Point2d((bbox.tl() + bbox.br()).x/2.0,(bbox.tl() + bbox.br()).y/2.0));
            marker_sum += Point2d((bbox.tl() + bbox.br()).x/2.0,(bbox.tl() + bbox.br()).y/2.0);
        }
        marker_center = marker_sum/4.0;
        cout << "after rect" << endl;
        //依据每个点和中心点的上下左右关系确定对应的 left_up, left_down, right_up, right_down
        for (int j = 0; j < marker_pixels.size(); ++j) {
            if(marker_pixels[j].x < marker_center.x)
            {
                if(marker_pixels[j].y < marker_center.y)
                    left_up = marker_pixels[j];
                else
                    left_down = marker_pixels[j];
            }
            else
            {
                if(marker_pixels[j].y < marker_center.y)
                    right_up = marker_pixels[j];
                else
                    right_down = marker_pixels[j];
            }
        }
        cout << "get 4 points" << endl;
        //按序放入marker序列中
        marker_sequence_pixel.clear();
        marker_sequence_pixel.emplace_back(left_up);
        marker_sequence_pixel.emplace_back(left_down);
        marker_sequence_pixel.emplace_back(right_up);
        marker_sequence_pixel.emplace_back(right_down);
        cout << "before pnpsolve" << endl;
        //solvePnP
        solvePnP(marker_struct,marker_sequence_pixel,cameraMatrix,distCoeffs,outputRvecRaw,outputTvecRaw,false,SOLVEPNP_P3P);
        target_position_of_cam[0] = outputTvecRaw.val[0];
        target_position_of_cam[1] = outputTvecRaw.val[1];
        target_position_of_cam[2] = outputTvecRaw.val[2];
        target_position_of_cam[3] = 1.0;
        ROS_INFO("target_x: %f, target_y: %f, target_z: %f", target_position_of_cam[0],target_position_of_cam[1], target_position_of_cam[2]);
        target_position_of_drone = tf_camera_to_drone * (tf_image_to_enu * target_position_of_cam);
        ROS_INFO("target_x_of drone: %f, target_y_of drone: %f, target_z_of drone: %f", target_position_of_drone[0],target_position_of_drone[1], target_position_of_drone[2]);
        
        drone_euler = quaternion2euler(odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w);
        drone_euler.z  = drone_euler.z - drone_euler_init.z;
        ROS_INFO("drone_euler.z: %f, drone_euler_init.z: %f",drone_euler.z,drone_euler_init.z);
        drone_quaternion = euler2quaternion_eigen(drone_euler.x,drone_euler.y,drone_euler.z);
        ROS_INFO("drone_roll: %f, drone_pitch: %f ,drone_yaw: %f", drone_euler.x,drone_euler.y,drone_euler.z);
        drone_euler_msg.pose.position.x = drone_euler.x;
        drone_euler_msg.pose.position.y = drone_euler.y;
        drone_euler_msg.pose.position.z = drone_euler.z;
        drone_euler_pub.publish(drone_euler_msg);
        
        tf_drone_to_world = Eigen::Isometry3d::Identity();
        //! the inverse of rotationmatrix == the transpose of rotationmatrix
        tf_drone_to_world.prerotate(drone_quaternion.toRotationMatrix());
        tf_drone_to_world.pretranslate(Eigen::Vector3d(0,0,0));
        target_position_of_world = tf_drone_to_world * target_position_of_drone;
        drone_pos_vision.x() = - target_position_of_world.x();
        drone_pos_vision.y() = - target_position_of_world.y();
        drone_pos_vision.z() = - target_position_of_world.z();
        
        cout << "drone_pos_vision x y z: " << drone_pos_vision.x() << " " << drone_pos_vision.y() << " " << drone_pos_vision.z() << endl;
        //valid if the value is right
        if (fabs(drone_pos_vision.x()+3) < 2 && fabs(drone_pos_vision.y()) < 2)
        {
            cout << "the x y value is within 2m and 2m" << endl;
            pnp_pose_is_good = true;
            cs_road_to_enu(drone_pos_vision.x(), drone_pos_vision.y(), drone_pos_vision_in_enu.x(),drone_pos_vision_in_enu.y());
            drone_pos_vision_msg.pose.position.x = drone_pos_vision_in_enu.x();
            drone_pos_vision_msg.pose.position.y = drone_pos_vision_in_enu.y();
        }
        else{
            cout << "the x error is : " << drone_pos_vision.x() << "the y error is : " << drone_pos_vision.y() << endl;
        }
        drone_vision_pos_pub.publish(drone_pos_vision_msg);

    }
    else{
        cout << "marker number is wrong, the number is :" << ir_contours.size() << endl;


    }




}

void mission_core::camera_param_set() {
    //camera param
    double fx = 384.67279052734375;
    double fy = 384.67279052734375;
    double cx = 324.8748779296875;
    double cy = 237.512451171875;


    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    distCoeffs = std::vector<double>{0.0, -0.0, -0.0, 0.0, 0};
    cameraMatrix.at<double>(0, 0) = fx*2; // wzy test
    cameraMatrix.at<double>(0, 2) = cx; 
    cameraMatrix.at<double>(1, 1) = fy*2; // wzy test
    cameraMatrix.at<double>(1, 2) = cy;
    cameraMatrix.at<double>(2, 2) = 1;


    //target param

    marker_struct.clear();
    //the vector sequence is left_up, left_down, right_up, right_down
    //world points use m unit, image coordinate system, x is right, y is down, z is forward
    marker_struct.emplace_back(cv::Point3f(-0.345,-0.140,0.0));
    marker_struct.emplace_back(cv::Point3f(-0.370,0.050,0.0));
    marker_struct.emplace_back(cv::Point3f(0.340,-0.155,0.0));
    marker_struct.emplace_back(cv::Point3f(0.340,0.155,0.0));

}

void mission_core::tf_param_set() {
    //image coordinate to ENU coordinate

    tf_image_to_enu = Eigen::Isometry3d::Identity();
    tf_image_to_enu.matrix() << 0, 0, 1, 0,
                                -1, 0, 0, 0,
                                0, -1, 0, 0,
                                0, 0, 0, 1;


    //the camera position of drone, now only translate , not rotation yet
    // tf is based on ENU axis
    Eigen::Vector3d pose_camera_of_drone;
    pose_camera_of_drone.x() = tf_camera_drone[0];
    pose_camera_of_drone.y() = tf_camera_drone[1];
    pose_camera_of_drone.z() = tf_camera_drone[2];


    tf_camera_to_drone = Eigen::Isometry3d::Identity();
    tf_camera_to_drone.matrix() << 1, 0, 0, pose_camera_of_drone.x(),
                                    0, 1, 0, pose_camera_of_drone.y(),
                                    0, 0, 1, pose_camera_of_drone.z(),
                                    0 ,0, 0, 1;


}

void mission_core::get_init_yaw() {
    drone_euler_init = quaternion2euler(odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w);
    got_attitude_init = true;


}

bool mission_core::pnp_follow_control_prepare() {
    camera_param_set();
    tf_param_set();
    return got_attitude_init;


}