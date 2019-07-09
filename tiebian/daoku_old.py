# coding: utf-8
import numpy as np
import math
import serial
import ntrip
import thread
import utils
import globalvar as g
import time
from model import Point


def get_reverse_point(dist_exp, yaw_exp_deg, lat_stop_deg, lon_stop_deg):
    # dist_exp为倒库距离，单位米
    # yaw_exp_deg为船坞方向角，单位度
    # lat_stop_deg为最终停靠点纬度，单位度
    # lon_stop_deg为最终停靠点经度，单位度
    # 函数返回
    # lat_plan为倒库点纬度，单位度
    # lon_plan为倒库点经度，单位度
    re = 6367444.65712259
    sin_lat = math.sin(np.deg2rad(lat_stop_deg))
    sin_lon = math.sin(np.deg2rad(lon_stop_deg))
    cos_lon = math.cos(np.deg2rad(lon_stop_deg))
    cos_lat = math.cos(np.deg2rad(lat_stop_deg))
    y_east = dist_exp * math.sin(np.deg2rad(yaw_exp_deg))
    x_north = dist_exp * math.cos(np.deg2rad(yaw_exp_deg))
    trans_mat = np.array([[-sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon],
                          [-sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon],
                          [cos_lat, 0, -sin_lat]])
    xyz_ecef = np.dot(trans_mat, np.array([x_north, y_east, -re])[:, None])
    lon_plan = np.rad2deg(math.atan2(xyz_ecef[1], xyz_ecef[0]))
    lat_plan = np.rad2deg(math.asin(xyz_ecef[2] / re))
    return lat_plan, lon_plan


def get_distance_cos(lat1, lng1, lat2, lng2, angle_an):
    (dist, angle) = get_distance_angle(lat1, lng1, lat2, lng2)
    dist_cos = dist*math.cos(np.deg2rad(angle_an-angle))
    return dist_cos


def get_distance_angle(lat1, lng1, lat2, lng2):
    # 输入值单位均为度
    # 以1点为期望点
    re = 6367444.65712259
    sin_lat1 = math.sin(np.deg2rad(lat1))
    cos_lat1 = math.cos(np.deg2rad(lat1))
    sin_lon1 = math.sin(np.deg2rad(lng1))
    cos_lon1 = math.cos(np.deg2rad(lng1))
    cos_lon2 = math.cos(np.deg2rad(lng2))
    sin_lon2 = math.sin(np.deg2rad(lng2))
    cos_lat2 = math.cos(np.deg2rad(lat2))
    sin_lat2 = math.sin(np.deg2rad(lat2))

    x1 = cos_lat1 * cos_lon1
    y1 = cos_lat1 * sin_lon1
    z1 = sin_lat1

    x2 = cos_lat2 * cos_lon2
    y2 = cos_lat2 * sin_lon2
    z2 = sin_lat2

    # trans_mat = np.array([[-sin_lat2 * cos_lon2, -sin_lat2 *
    #                        sin_lon2, cos_lat2], [-sin_lon2, cos_lon2, 0]])
    trans_mat = np.array([[-sin_lat1 * cos_lon1, -sin_lat1 *
                           sin_lon1, cos_lat1], [-sin_lon1, cos_lon1, 0]])
    dist = re * math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
    # co_ned = np.dot(trans_mat, np.array([x1, y1, z1])[:, None])
    co_ned = np.dot(trans_mat, np.array([x2, y2, z2])[:, None])
    angle = np.rad2deg(math.atan2(co_ned[1], co_ned[0]))
    if angle < 0:
        angle += 360
    return dist, angle


def saturate(x_input, x_min, x_max):
    if x_input > x_max:
        return x_max
    elif x_input < x_min:
        return x_min
    else:
        return x_input

# 第一次原地转


def angle_control(angle_error, d_angle_error):
    p_ang = 0.5
    d_ang = 1
    # 修角
    if math.fabs(angle_error) > 90:
        # 全速修角
        direction = np.sign(angle_error) * 100
    else:
        # PD修角
        direction = p_ang * angle_error + d_ang * d_angle_error
        direction = saturate(direction, -100, 100)

    if direction < 0:
        direction = min(-15, direction)
    elif direction > 0:
        direction = max(15, direction)
    return direction


# 直行段
def angle_control_2(angle_error, d_angle_error):
    p_ang = 1.0
    d_ang = 0.3
    # 修角
    if math.fabs(angle_error) > 100:
        # 全速修角
        direction = np.sign(angle_error) * 100
    else:
        # PD修角
        direction = p_ang * angle_error + d_ang * d_angle_error
        direction = saturate(direction, -100, 100)
    if direction < 0:
        direction = min(-15, direction)
    elif direction > 0:
        direction = max(15, direction)
    return direction

# 接近段修正


def angle_control_3(angle_error, d_angle_error):
    p_ang = 2.0
    d_ang = 0.3
    # PD修角
    direction = p_ang * angle_error + d_ang * d_angle_error
    direction = saturate(direction, -100, 100)
    if direction < 0:
        direction = min(-20, direction)
    elif direction > 0:
        direction = max(20, direction)
    return direction

# 原地屁股朝后


def angle_control_4(angle_error, d_angle_error):
    p_ang = 0.5
    d_ang = 0.6
    # PD修角
    direction = p_ang * angle_error + d_ang * d_angle_error
    direction = saturate(direction, -100, 100)
    if direction < 0:
        direction = min(-20, direction)
    elif direction > 0:
        direction = max(20, direction)
    return direction


def angle_control_5(angle_error, d_angle_error):
    p_ang = 1
    d_ang = 0.6
    # PD修角
    direction = p_ang * angle_error + d_ang * d_angle_error
    direction = saturate(direction, -100, 100)
    if direction < 0:
        direction = min(-20, direction)
    elif direction > 0:
        direction = max(20, direction)
    return direction


dist_last = 1000


def control_reverse_point(lat_now, lng_now, yaw_now, ur, rz, lat_exp, lng_exp, flag_ang_control):
    global dist_last
    # lat_now当前位置_纬度(单位度)
    # lng_now当前位置_经度(单位度)
    # yaw_now当前方向角
    # ur 当前船速
    # rz 当前方向角速度(单位度)#注意：逆时针为正
    # 输出
    # mag 油门大小，幅值范围从-100到100
    # direction 方向，幅值范围从-100到100

    p_dis = 6  # 接近段启用
    d_dis = 2  # 接近段启用

    approach_distance = 8  # 接近段距离
    stop_circle_radius = 0.5  # 误差圆半径
    max_error_angle = 2  # 行进过程最大角度误差
    max_throttle = 0.4  # 接近段最大油门倍率

    (dist_now, angle_now) = get_distance_angle(
        lat_now, lng_now, lat_exp, lng_exp)
    dist_cos = get_distance_cos(
        lat_exp, lng_exp, lat_now, lng_now, 279-90)
    if dist_now > dist_last + 0.1 and dist_now < 2:
        return 0, 0, 0
    dist_last = dist_now

    angle_error = angle_now - yaw_now
    if angle_error > 180:
        angle_error -= 360
    elif angle_error <= -180:
        angle_error += 360
    d_angle_error = np.rad2deg(
        math.sin(np.deg2rad(angle_error)) * ur / dist_now) - rz
    d_dist = -ur * math.cos(np.deg2rad(angle_error))

    if math.fabs(angle_error) > max_error_angle and flag_ang_control == 0:
        mag = 0
        direction = angle_control(angle_error, d_angle_error)
        flag_ang_control = 0
    else:
        flag_ang_control = 1
        if dist_now > approach_distance:
            # 全速直线行驶
            mag = 100
            if math.fabs(angle_error) > max_error_angle:
                direction = angle_control_2(
                    angle_error, d_angle_error)
            else:
                direction = 0
        elif abs(dist_cos) < stop_circle_radius and dist_now < 2:
            # 到达
            mag = 0
            direction = 0
        else:
            mag = p_dis * dist_now + d_dis * d_dist
            mag = saturate(mag, -max_throttle * 100,
                           max_throttle * 100)  # 最大直线油门设置为0.5
            if math.fabs(mag) < 20:
                mag = np.sign(mag)*20
            direction = angle_control_3(angle_error, d_angle_error)
            if dist_now < 2:
                direction = saturate(direction, -20, 20)
    return mag, direction, flag_ang_control


# lat_rev = 34.248804333333
# lng_rev = 108.89448233333
lat_rev1, lng_rev1 = 34.24866133333333, 108.89447266666667


def serial_recv_thread():
    ser = serial.Serial("/dev/ttyAMA0", 115200)
    gz = 0
    flag_ang_control = 0
    flag_finish = False
    flag_back = False
    while True:
        if True or g.getValue('mode') != 1 and g.getValue('mode') != 0:
            try:
                msg = ser.readline().strip()
                if msg.startswith('$') and msg.endswith('#'):
                    datas = msg.replace('$', '').replace('#', '').split(';')
                    if len(datas) >= 10:
                        yaw = float(datas[3])
                        gps_speed = float(datas[7])

                        if flag_finish:
                            angle_error = 279 - yaw
                            if angle_error > 180:
                                angle_error -= 360
                            elif angle_error <= -180:
                                angle_error += 360

                            if flag_back:
                                dire = angle_control_5(angle_error, -gz)
                                dire = 0
                                ser.write(utils.parityGenerate(
                                    "$E;-25,%d#" % dire))
                                dist = utils.getDistance(Point(lat_rev1, lng_rev1), Point(
                                    g.getValue('lat'), g.getValue('lng')))
                                print(dist, angle_error)
                                if dist < 2:
                                    time.sleep(1)
                                    ser.write(utils.parityGenerate("$E;0,0#"))
                                    break
                                continue

                            if abs(angle_error) < 10:
                                ser.write(utils.parityGenerate("$E;-25,0#"))
                                flag_back = True
                            dire = angle_control_4(angle_error, -gz)
                            print("角度差、方向油门：%d\t%d" % (angle_error, dire))
                            ser.write(utils.parityGenerate(
                                "$E;%d,%d#" % (0, dire)))
                            continue

                        (thrust, dire, flag_ang_control) = control_reverse_point(
                            g.getValue('lat'), g.getValue('lng'), yaw, gps_speed, -gz, lat_rev, lng_rev, flag_ang_control)
                        # print(g.getValue('lat'), g.getValue(
                        #     'lng'), g.getValue('mode'), yaw, gps_speed, thrust, dire)
                        ser.write(utils.parityGenerate(
                            "$E;%d,%d#" % (thrust, dire)))
                        if thrust == 0 and dire == 0:
                            flag_finish = True
                elif msg.startswith('&') and msg.endswith('#'):
                    datas = msg.replace('&', '').replace('#', '').split(';')
                    gz = float(datas[0])
            except KeyboardInterrupt:
                break
            except Exception:
                continue
    ser.close()
    print('Finish')


# example:
if __name__ == '__main__':
    g.init()
    (lat_rev, lng_rev) = get_reverse_point(5, 279, lat_rev1, lng_rev1)
    thread.start_new_thread(serial_recv_thread, ())
    ntrip.fun()
    # lat_stop_example = 40.009435000000003  # 终点
    # lng_stop_example = 1.163236750000000e+02  # 终点
    # yaw_exp_deg2 = 240  # 船坞方向角
    # dist_exp2 = 10  # 倒库距离

    # lat_now_example = 40.009426001786771  # 当前纬度
    # lng_now_example = 1.163236085034641e+02  # 当前经度
    # yaw_now = 200  # 当前方向角
    # ur = 1.4  # 当前船速
    # rz = 2  # 当前方向角速度

    # (lat_rev, lng_rev) = get_reverse_point(
    #     dist_exp2, yaw_exp_deg2, lat_stop_example, lng_stop_example)
    # print("倒库点纬度为", lat_rev, "度,", "经度为", lng_rev, "度")

    # (dist2, angle2) = get_distance_angle(
    #     lat_now_example, lng_now_example, lat_rev, lng_rev)
    # print("当前距离倒库点", dist2, "米,", "角度差为", angle2, "度")

    # (flag, mag) = control_reverse_point(lat_now_example,
    #                                     lng_now_example, yaw_now, ur, rz, lat_rev, lng_rev)
    # print(flag, mag)
