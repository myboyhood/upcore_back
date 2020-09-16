#!/usr/bin/env python

import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, CommandBoolRequest, CommandBoolResponse
from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
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
uav_state = State()
ismove = Bool()
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


if __name__=="__main__":
    # global gps_curr, gps_pub_msg

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('gps_pos_teleop')
    sub_gps_global = rospy.Subscriber('/mavros/global_position/global',NavSatFix, global_pos_cb)
    sub_state = rospy.Subscriber('/mavros/state',State, state_cb)
    sub_move_flag = rospy.Subscriber('move_flag', Bool, move_flag_cb)

    # pub_gps_global = rospy.Publisher('/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=1)
    pub_gps_geo = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)

    pub_local_position = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    rate = rospy.Rate(20)

    """ acquire GPS init """
    count = 0
    while not (rospy.is_shutdown() or count > 10):
        gps_lat_init = gps_curr.latitude
        gps_lon_init = gps_curr.longitude
        gps_alt_init = gps_curr.altitude
        print(gps_alt_init, " ", gps_lon_init, " ", gps_alt_init)
        rate.sleep()
        count += 1

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

    """pub GPS geo position  for 100 counts"""
    count = 0
    while not (rospy.is_shutdown() or count > 100):
        gps_pub_geo_msg.pose.position.latitude = gps_lat_init
        gps_pub_geo_msg.pose.position.longitude = gps_lon_init
        gps_pub_geo_msg.pose.position.altitude = gps_alt_init + 2
        pub_gps_geo.publish(gps_pub_geo_msg)
        print("pub gps geo 100 ")
        rate.sleep()
        count += 1

    # """pub local position  for 100 counts"""
    # local_pos = PoseStamped()
    # local_pos.pose.position.x = 0
    # local_pos.pose.position.y = 0
    # local_pos.pose.position.z = 2

    # count = 0
    # while not (rospy.is_shutdown() or count > 100):
    #     pub_gps_global.publish(gps_pub_msg)
    #     # pub_local_position.publish(local_pos)
    #     print("pub local position 100 ")
    #     rate.sleep()
    #     count += 1

    """change Mode"""
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    offb_set_mode = SetModeRequest()
    offb_set_mode.base_mode = 0
    offb_set_mode.custom_mode = "OFFBOARD"
    now = rospy.get_rostime()
    last_request = now.secs

    while not (rospy.is_shutdown() or ismove):
        pub_gps_geo.publish(gps_pub_geo_msg)
        # pub_gps_global.publish(gps_pub_msg)
        # pub_local_position.publish(local_pos)

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

        rate.sleep()



    status = 0
    lat_sum = 0
    lon_sum = 0
    lat_delta = 0.0
    lon_delta = 0.0
    lat_base = 1 #  molecular of lat_delta
    lon_base = 1 #  molecular of lon_delta
    lat_denominator = 100000 # denominator of lat
    lon_denominator = 100000 # denominator of lon
    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()

            if key == 'w' :
                lat_delta = lat_base/lat_denominator
                lat_sum = lat_sum + lat_delta
                status = status + 1
                print("w, to north : {delta}, sum: {sum}".format(delta=lat_delta, sum=lat_sum))

            elif key == 'x' :
                lat_delta = - lat_base/lat_denominator
                lat_sum = lat_sum + lat_delta
                status = status + 1
                print("x, to south : {delta}, sum: {sum}".format(delta=lat_delta, sum=lat_sum))

            elif key == 'a' :
                lon_delta = - lon_base/lon_denominator
                lon_sum = lon_sum + lon_delta
                status = status + 1
                print("a, to west : {delta}, sum: {sum}".format(delta=lon_delta, sum=lon_sum))

            elif key == 'd' :
                lon_delta = lon_base/lon_denominator
                lon_sum = lon_sum + lon_delta
                status = status + 1
                print("d, to east : {delta}, sum: {sum}".format(delta=lon_delta, sum=lon_sum))

            elif key == ' ' or key == 's' :
                lon_delta = 0
                lat_delta = 0

                print("stop")

            else:
                if (key == '\x03'):
                    break


            """for the no press key time"""
            if status == 0:
                gps_target_lat = gps_curr.latitude
                gps_target_lon = gps_curr.longitude

            """ calculate gps msg """
            gps_pub_msg.latitude = gps_curr.latitude
            gps_pub_msg.longitude = gps_curr.longitude
            gps_pub_msg.altitude = gps_curr.altitude # height is acquired by lidar lite v3

            """ if new command or not """
            if abs(lat_delta) > lat_base*0.5/lat_denominator or abs(lon_delta) > lon_base*0.5/lon_denominator: # if true, move, otherwise, keep
                print("new command")
                gps_target_lat = gps_pub_msg.latitude + lat_delta
                gps_target_lon = gps_pub_msg.longitude + lon_delta


            """ if reach target position or not """
            if abs(gps_target_lat - gps_curr.latitude) > lat_base*0.2/lat_denominator or abs(gps_target_lon - gps_curr.longitude) > lon_base*0.2/lon_denominator:
                print("reaching target ... ")
                gps_pub_msg.latitude = gps_target_lat
                gps_pub_msg.longitude = gps_target_lon

            """ clear command """
            lat_delta = 0
            lon_delta = 0

            print("gps_pub_msg")
            print(gps_pub_msg.latitude, gps_pub_msg.longitude, gps_pub_msg.altitude)
            # gps_pub_msg.IGNORE_LATITUDE = 0
            # gps_pub_msg.IGNORE_LONGITUDE = 0
            # gps_pub_msg.IGNORE_ALTITUDE = 0
            # pub_gps_global.publish(gps_pub_msg)
            pub_gps_geo.publish(gps_pub_geo_msg)
            rate.sleep()





    except:
        print(e)

    finally:
        # gps_pub_msg.latitude = gps_curr.latitude
        # gps_pub_msg.longitude = gps_curr.longitude
        # gps_pub_msg.altitude = gps_curr.altitude # height is acquired by lidar lite v3
        # pub_gps_global.publish(gps_pub_msg)

        gps_pub_geo_msg.pose.position.latitude = gps_curr.latitude
        gps_pub_geo_msg.pose.position.longitude = gps_curr.longitude
        gps_pub_geo_msg.pose.position.altitude = gps_curr.altitude # height is acquired by lidar lite v3
        pub_gps_geo.publish(gps_pub_geo_msg)