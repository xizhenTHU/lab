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


def get_half_point(half_length,yaw, lat, lng):
    # yaw为当前船方向角，单位度
    # lat为当前纬度，单位度
    # lng为当前经度，单位度
    # 函数返回
    # lat_half为船中点纬度，单位度
    # lng_half为船中点经度，单位度

    #half_length = 0.7  # GPS天线到船中点位置，单位米，注意GPS在船尾时此值为正，反之为负
    re = 6367444.65712259
    sin_lat = math.sin(np.deg2rad(lat))
    sin_lon = math.sin(np.deg2rad(lng))
    cos_lon = math.cos(np.deg2rad(lng))
    cos_lat = math.cos(np.deg2rad(lat))
    y_east = half_length * math.sin(np.deg2rad(yaw))
    x_north = half_length * math.cos(np.deg2rad(yaw))
    trans_mat = np.array([[-sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon],
                          [-sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon],
                          [cos_lat, 0, -sin_lat]])
    xyz_ecef = np.dot(trans_mat, np.array([x_north, y_east, -re])[:, None])
    lng_half = np.rad2deg(math.atan2(xyz_ecef[1], xyz_ecef[0]))
    lat_half = np.rad2deg(math.asin(xyz_ecef[2] / re))
    return lat_half, lng_half


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
    trans_mat = np.array([[-sin_lat1 * cos_lon1, -sin_lat1 *
                           sin_lon1, cos_lat1], [-sin_lon1, cos_lon1, 0]])
    dist = re * math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
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


'''
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
'''
gz_last = 0  # global参量


def PID_Turn(error, yawShip, gz):  # 原地转弯PID
    global gz_last  # 上一时刻角速度
    PID_P_gz = 5
    # PID_D_gz 在下面if结构中定义
    # yawAim = utils.getAimYaw(pShip, pAim)  # 期望方向角
    # error = utils.angleDiff(yawAim, yawShip)  # 角度差
    gzAim = 10*utils.symbol(error)  # 期望角速度

    if abs(error) > 100:
        dire = 100*utils.symbol(error)  # 角度差大于100度时全速转弯
    elif abs(error) < 20:
        gzAim = gzAim*abs(error)/20*0.5
        error_gz = gzAim-gz  # 角速度误差
        dire = PID_P_gz * error_gz
        if dire < 0:
            dire = min(dire, -10)
        else:
            dire = max(dire, 10)
    else:
        error_gz = gzAim-gz  # 角速度误差
        d_gz = gz-gz_last  # 角速度增量
        gz_last = gz
        if error_gz*d_gz > 0:
            PID_D_gz = -1
        else:
            PID_D_gz = 1
        dire = PID_P_gz * error_gz+10*utils.symbol(error)
    gz_last = gz
    dire = min(dire, 60)
    dire = max(dire, -60)
    return dire


gps_speed = 0  # global参量
thrust_last = 0  # global参量


def PID_Go(error, dist, yawShip, gz):  # 直行段PID
    global thrust_last  # 上一时刻油门值
    PID_P = 0.5
    # PID_D = 0 在下面if结构中定义
    # yawAim = utils.getAimYaw(pShip, pAim)  # 期望方向角
    # error = utils.angleDiff(yawAim, yawShip)  # 方向角误差
    # dist = utils.getDistance(pShip, pAim)  # 距离误差
    if error*gz > 0:
        PID_D = -0.8
    else:
        PID_D = 0.8
    dire = PID_P * error + gz * PID_D
    dire = max(dire, -100)
    dire = min(dire, 100)

    thrust = 100-abs(error)*3
    if dist < 10:
        tar_speed = dist/8-abs(error)/20  # 目标速度
        tar_speed = max(tar_speed, 0.2)  # 目标速度限幅在0.2到1之间
        tar_speed = min(tar_speed, 1)
        err_speed = tar_speed-gps_speed
        thrust = thrust_last+err_speed*50
    thrust = abs(thrust)
    thrust = max(thrust, 10)  # 下限是10
    thrust = min(thrust, 100)  # 上限100
    thrust_last = thrust
    return int(dire), thrust


dist_last = 1000


def control_reverse_point(lat_now, lng_now, yaw_now, ur, gz, lat_exp, lng_exp, flag_ang_control):
    global dist_last
    # lat_now当前位置_纬度(单位度)
    # lng_now当前位置_经度(单位度)
    # yaw_now当前方向角
    # ur 当前船速
    # gz 当前方向角速度(单位度)#注意：逆时针为正
    # 输出
    # mag 油门大小，幅值范围从-100到100
    # direction 方向，幅值范围从-100到100

    stop_circle_radius = 0.5  # 误差圆半径
    max_error_angle = 2  # 行进过程最大角度误差

    (dist_now, angle_now) = get_distance_angle(
        lat_now, lng_now, lat_exp, lng_exp)  # 计算距离误差，期望方向角
    dist_cos = get_distance_cos(
        lat_exp, lng_exp, lat_now, lng_now, 279-90)  # 距离误差在岸向投影

    if dist_now > dist_last + 0.1 and dist_now < 2:
        return 0, 0, 0
    dist_last = dist_now

    angle_error = angle_now - yaw_now
    if angle_error > 180:
        angle_error -= 360
    elif angle_error <= -180:
        angle_error += 360

    if math.fabs(angle_error) > max_error_angle and flag_ang_control == 0:
        # Turn
        thrust = 0
        dire = PID_Turn(angle_error, yaw_now, gz)
        flag_ang_control = 0
    else:
        flag_ang_control = 1
        if dist_now > stop_circle_radius:
            # Go
            dire, thrust = PID_Go(angle_error, dist_now, yaw_now, gz)
        else:
            # 到达
            thrust = 0
            dire = 0
    return thrust, dire, flag_ang_control


# lat_rev = 34.248804333333
# lng_rev = 108.89448233333
lat_rev1, lng_rev1 = 34.24866133333333, 108.89447266666667


def serial_recv_thread():
    global gps_speed
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
                        if flag_finish:  # 到达倒库点
                            angle_error = 279 - yaw
                            if angle_error > 180:
                                angle_error -= 360
                            elif angle_error <= -180:
                                angle_error += 360

                            if flag_back:  # 在倒库点修完角，开始后退
                                # 边后退边修角，修角PID采用PID_Go中的
                                PID_P = 0.5
                                if angle_error*(-gz) > 0:
                                    PID_D = -0.8
                                else:
                                    PID_D = 0.8
                                dire = PID_P * angle_error + (-gz) * PID_D
                                dire = saturate(dire, -100, 100)
                                #考虑是否使用PID_Go然后thrust加负号

                                ser.write(utils.parityGenerate(
                                    "$E;-15,%d#" % dire))
                                dist = utils.getDistance(Point(lat_rev1, lng_rev1), Point(
                                    g.getValue('lat'), g.getValue('lng')))  # 离最终点距离
                                print(dist, angle_error)
                                if dist < 2:  # 离最终点距离小于2m，停止
                                    time.sleep(1)
                                    ser.write(utils.parityGenerate("$E;0,0#"))
                                    break
                                continue

                            if abs(angle_error) < 10:
                                ser.write(utils.parityGenerate("$E;-15,0#"))
                                flag_back = True
                            dire = PID_Turn(
                                angle_error, yaw_now, -gz)  # 在倒库点修角
                            # dire = angle_control_4(angle_error, -gz)
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
