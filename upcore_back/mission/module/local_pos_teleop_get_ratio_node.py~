#!/usr/bin/env python

import rospy
import _thread
from mavros_msgs.msg import GlobalPositionTarget, State
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, CommandBoolRequest, CommandBoolResponse
from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

msg = """
Control Your laser plane!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease latitude weidu  north/south
a/d : increase/decrease longitude jingdu  east/west

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

"""
global variable
"""
gps_curr = NavSatFix()
gps_pub_msg = GlobalPositionTarget()
gps_pub_geo_msg = GeoPoseStamped()
local_curr_pose = PoseStamped()
uav_state = State()
ismove = Bool()
gps_list = list()
xy_list = list()
gps_xy_file = open("/home/wzy/catkin_ws/src/mission/scripts/gps_xy_file.txt", "a", buffering=1000)
ismove = False

def getKey():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def global_pos_cb(gps_data):
    global gps_curr
    gps_curr = gps_data
    # rospy.loginfo("lat:  %f ", gps_curr.latitude)
    # rospy.loginfo("lon:  %f ", gps_curr.longitude)
    # rospy.loginfo("alt:  %f ", gps_curr.altitude)


def state_cb(state_data):
    global uav_state
    uav_state = state_data


def move_flag_cb(move_data):
    global ismove
    ismove = move_data


def local_pos_cb(local_data):
    global local_curr_pose
    local_curr_pose = local_data

def write_to_file():
    global gps_list, xy_list
    gps_error_list = list()
    xy_error_list = list()
    g_e_sum_lon = 0
    g_e_sum_lat = 0
    x_e_sum = 0
    y_e_sum = 0


    print("write")
    gps_xy_file.write("\n\nnew data write into file\n")
    try:
        """GPS list"""
        gps_last = gps_list[0]
        print("-------- gps --------")
        for gps in gps_list:
            print(gps)
            gps_xy_file.write(str(gps[0]) + "," + str(gps[1]) + "\n")

            gps_error = gps - gps_last # gps[0] - gps_last[0], gps[1] - gps_last[1]
            gps_xy_file.write("lon_error: " + str(gps_error[0]) + "\n")
            gps_xy_file.write("lat_error: " + str(gps_error[1]) + "\n")
            gps_error_list.append(gps_error)
            gps_last = gps

        print(gps_error_list)
        for g_e in gps_error_list: # only len(gps_error_list)-1 is useful
            g_e_sum_lon += g_e[0] # error_sum in longitude
            g_e_sum_lat += g_e[1] # error_sum in longitude
            print(g_e_sum_lon)
            print(g_e_sum_lat)

        print("num: " + str(east_count - w_x_first))
        print("num: " + str(north_count - a_d_first))
        g_e_avg_lon = g_e_sum_lon / east_count - w_x_first
        g_e_avg_lat = g_e_sum_lat / north_count - a_d_first
        gps_xy_file.write("g_e_avg_lon : " + str(g_e_avg_lon)+"\n")
        gps_xy_file.write("g_e_avg_lat : " + str(g_e_avg_lat)+"\n")

        gps_xy_file.flush()

        """xy list"""
        xy_last = xy_list[0]
        print("-------- xy --------")
        for xy in xy_list:

            print(xy)
            gps_xy_file.write(str(xy[0]) + "," + str(xy[1]) + "\n")

            xy_error = xy - xy_last
            gps_xy_file.write("x_error: " + str(xy_error[0]) + "\n")
            gps_xy_file.write("y_error: " + str(xy_error[1]) + "\n")
            xy_error_list.append(xy_error)
            xy_last = xy
        print(xy_error_list)
        for xy_e in xy_error_list:
            x_e_sum += xy_e[0]
            y_e_sum += xy_e[1]

        print("num: "+ str(east_count - w_x_first))
        print("num: "+ str(north_count - a_d_first))
        x_e_avg = x_e_sum / east_count - w_x_first
        y_e_avg = y_e_sum / north_count - a_d_first
        gps_xy_file.write("x_e_avg : " + str(x_e_avg)+"\n")
        gps_xy_file.write("y_e_avg : " + str(y_e_avg)+"\n")

        """calculate ratio of lon / x , lat / y"""
        lon_x = g_e_avg_lon / x_e_avg
        lat_y = g_e_avg_lat / y_e_avg
        gps_xy_file.write("lon_x : " + str(lon_x)+"\n")
        gps_xy_file.write("lat_y : " + str(lat_y)+"\n")

        gps_xy_file.flush()
        gps_xy_file.close()

    except Exception:
        print("write to file error !!!")

if __name__=="__main__":
    # global gps_curr, gps_pub_msg

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('gps_pos_teleop')
    sub_gps_global = rospy.Subscriber('/mavros/global_position/global',NavSatFix, global_pos_cb)
    sub_local_pose = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, local_pos_cb)
    sub_state = rospy.Subscriber('/mavros/state',State, state_cb)
    sub_move_flag = rospy.Subscriber('move_flag', Bool, move_flag_cb)

    # pub_gps_global = rospy.Publisher('/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=1)
    # pub_gps_geo = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)

    pub_local_position = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    rate = rospy.Rate(20)

    # """ acquire GPS init """
    # count = 0
    # while not (rospy.is_shutdown() or count > 10):
    #     gps_lat_init = gps_curr.latitude
    #     gps_lon_init = gps_curr.longitude
    #     gps_alt_init = gps_curr.altitude
    #     print(gps_alt_init, " ", gps_lon_init, " ", gps_alt_init)
    #     rate.sleep()
    #     count += 1

    # """pub GPS global position  for 100 counts"""
    # count = 0
    # while not (rospy.is_shutdown() or count > 100):
    #     gps_pub_msg.latitude = gps_lat_init
    #     gps_pub_msg.longitude = gps_lon_init
    #     gps_pub_msg.altitude = gps_alt_init + 2
    #     pub_gps_global.publish(gps_pub_msg)
    #     print("pub gps global 100 ")
    #     rate.sleep()
    #     count += 1

    # """pub GPS geo position  for 100 counts"""
    # count = 0
    # while not (rospy.is_shutdown() or count > 100):
    #     gps_pub_geo_msg.pose.position.latitude = gps_lat_init
    #     gps_pub_geo_msg.pose.position.longitude = gps_lon_init
    #     gps_pub_geo_msg.pose.position.altitude = gps_alt_init + 2
    #     pub_gps_geo.publish(gps_pub_geo_msg)
    #     print("pub gps geo 100 ")
    #     rate.sleep()
    #     count += 1

    """pub local position  for 100 counts"""
    local_pub_pos = PoseStamped()
    local_pub_pos.pose.position.x = 0
    local_pub_pos.pose.position.y = 0
    local_pub_pos.pose.position.z = 2

    count = 0
    while not (rospy.is_shutdown() or count > 50):
        pub_local_position.publish(local_pub_pos)
        print("pub local position 100 ")
        rate.sleep()
        count += 1

    """change Mode"""
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    offb_set_mode = SetModeRequest()
    offb_set_mode.base_mode = 0
    offb_set_mode.custom_mode = "OFFBOARD"
    now = rospy.get_rostime()
    last_request = now.secs

    while not (rospy.is_shutdown()):
        # pub_gps_geo.publish(gps_pub_geo_msg)
        # pub_gps_global.publish(gps_pub_msg)
        pub_local_position.publish(local_pub_pos)

        if((uav_state.mode != "OFFBOARD") and (rospy.get_rostime().secs-last_request > 2)):
            rospy.loginfo(uav_state.mode)
            if((set_mode_client.call(offb_set_mode) == True) and (offb_set_mode.response.mode_sent == True)):
                rospy.loginfo(" offboard enabled")
            last_request = rospy.get_rostime().secs

        else:
            if((uav_state.armed == False) and (rospy.get_rostime().secs-last_request > 2)):
                rospy.loginfo(" arming...")
                if (arming_client.call(arm_cmd).success):
                    rospy.loginfo(" armed")
                    last_request = rospy.get_rostime().secs

        if abs(local_curr_pose.pose.position.z - 2) < 0.1:
            rospy.loginfo("into teleop mode")
            break

        rate.sleep()



    status = 0
    east_count = 0  # record how many times plane fly to east
    north_count = 0  # record how many times plane fly to north
    lat_sum = 0
    lon_sum = 0
    lat_delta = 0.0
    lon_delta = 0.0
    lat_base = 1 #  molecular of lat_delta
    lon_base = 1 #  molecular of lon_delta
    lat_denominator = 100000 # denominator of lat
    lon_denominator = 100000 # denominator of lon
    local_x_delta = 0 # [m]
    local_y_delta = 0 # [m]
    local_x_step = 1 # [m] step for east
    local_y_step = 1 # [m] step for north
    local_x_sum = 0 # [m]
    local_y_sum = 0 # [m]
    move_flag = False
    record_first_key = True
    w_x_first = 0
    a_d_first = 0

    try:

        print(msg)
        while not rospy.is_shutdown():
            key = getKey()

            if record_first_key and status == 1:
                record_first_key = False
                if key == 'w' or key == 'x':
                    w_x_first = 1
                    a_d_first = 0
                if key == 'a' or key == 'd':
                    w_x_first = 0
                    a_d_first = 1

            if key == 'w':
                local_x_delta = local_x_step
                local_x_sum = local_x_sum + local_x_delta
                status = status + 1
                east_count += 1
                print("w, to east : {delta}, sum: {sum}".format(delta=local_x_delta, sum=local_x_sum))

            elif key == 'x':
                local_x_delta = - local_x_step
                local_x_sum = local_x_sum + local_x_delta
                status = status + 1
                east_count -= 1
                print("x, to west : {delta}, sum: {sum}".format(delta=local_x_delta, sum=local_x_sum))

            elif key == 'a':
                local_y_delta = local_y_step
                local_y_sum = local_y_sum + local_y_delta
                status = status + 1
                north_count += 1
                print("a, to north : {delta}, sum: {sum}".format(delta=local_y_delta, sum=local_y_sum))

            elif key == 'd':
                local_y_delta = - local_y_step
                local_y_sum = local_y_sum + local_y_delta
                status = status + 1
                north_count -= 1
                print("d, to south : {delta}, sum: {sum}".format(delta=local_y_delta, sum=local_y_sum))

            elif key == ' ' or key == 's' : # if no key pressed or s, set delta = 0
                local_x_delta = 0
                local_y_delta = 0

                print("stop")

            else:
                if (key == '\x03'):
                    break



            """for the no press key time"""

            if status == 0:
                status += 1
                local_target_x = local_curr_pose.pose.position.x
                local_target_y = local_curr_pose.pose.position.y
                local_target_z = 2

            """ calculate local msg , firstly initialise with current pose"""
            local_pub_pos.pose.position.x = local_target_x
            local_pub_pos.pose.position.y = local_target_y
            local_pub_pos.pose.position.z = 2

            """ if new command or not """
            if abs(local_x_delta) > 0.5*local_x_step or abs(local_y_delta) > 0.5*local_y_step: # if true, move, otherwise, keep
                print("new command")
                print("status: ", status)
                local_target_x = local_pub_pos.pose.position.x + local_x_delta
                local_target_y = local_pub_pos.pose.position.y + local_y_delta
                move_flag = True

            print(abs(local_curr_pose.pose.position.x - local_target_x), abs(local_curr_pose.pose.position.y - local_target_y))
            """ if reach target position or not """
            if abs(local_curr_pose.pose.position.x - local_target_x) > 0.2 or abs(local_curr_pose.pose.position.y - local_target_y) > 0.2:
                print("reaching target ... ")
                local_pub_pos.pose.position.x = local_target_x
                local_pub_pos.pose.position.y = local_target_y
            else:
                rospy.loginfo(" reached destination ")
                gps_lat_now = gps_curr.latitude
                gps_lon_now = gps_curr.longitude
                if move_flag:
                    move_flag = False
                    print("append")
                    gps_list.append(np.array([gps_lon_now, gps_lat_now]))
                    xy_list.append(np.array([local_curr_pose.pose.position.x, local_curr_pose.pose.position.y]))

            """show list"""
            if key == 's':
                print(gps_list)
                print(xy_list)
                break

            """ clear command """
            local_x_delta = 0
            local_y_delta = 0

            pub_local_position.publish(local_pub_pos)
            rate.sleep()

        print("end of loop")
        # try:
        #     _thread.start_new_thread(write_to_file)
        #     while not rospy.is_shutdown():
        #         rate.sleep()
        # except:
        #     print("Thread can not start")
        write_to_file()

    except:
        print(e)

    finally:
        # gps_pub_msg.latitude = gps_curr.latitude
        # gps_pub_msg.longitude = gps_curr.longitude
        # gps_pub_msg.altitude = gps_curr.altitude # height is acquired by lidar lite v3
        # pub_gps_global.publish(gps_pub_msg)

        local_pub_pos.pose.position.x = local_curr_pose.pose.position.x
        local_pub_pos.pose.position.y = local_curr_pose.pose.position.y
        local_pub_pos.pose.position.z = 2 # height is acquired by lidar lite v3
        pub_local_position.publish(local_pub_pos)
