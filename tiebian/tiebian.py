# coding: utf-8
import utils
from model import Point
import math
import numpy as np
import csv
from rcRecord import simplified
from enum import IntEnum
import globalvar as g
import ntrip
import logging
import serial
import thread
import time


class State(IntEnum):
    TURN = 1
    GO = 2


sidePoints = list()
yaw = 0
speed = 0
gz = 0
gz_last = 0  # global参量
thrust_last = 0  # global参量
ser = None


def lla2ned_2D(latlng,lat0,lng0):
    sin_lat = math.sin(np.deg2rad(lat0))
    sin_lng = math.sin(np.deg2rad(lng0))
    cos_lng = math.cos(np.deg2rad(lng0))
    cos_lat = math.cos(np.deg2rad(lat0))
    trans_mat = np.array([[-sin_lat * cos_lng, -sin_lng, -cos_lat * cos_lng],
                          [-sin_lat * sin_lng, cos_lng, -cos_lat * sin_lng],
                          [cos_lat, 0, -sin_lat]])
    re = 6367444.65712259
    NE_vec=[]
    for  ii in range(len(latlng)):
        lat, lng=latlng[ii,0],latlng[ii,1]
        sin_lat = math.sin(np.deg2rad(lat))
        sin_lng = math.sin(np.deg2rad(lng))
        cos_lng = math.cos(np.deg2rad(lng))
        cos_lat = math.cos(np.deg2rad(lat))
        ecef=re*np.array([cos_lat*cos_lng,cos_lat*sin_lng,sin_lat])
        NED=np.dot(trans_mat.T, ecef)
        NE_vec.append([NED[0],NED[1]])
    return  np.array(NE_vec)#x_north,y_east

def ExponentialDecay(decay_factor, list_length):
    # 指数衰减加权
    list_factor=decay_factor**np.arange(1,1+list_length)
    list_factor/=sum(list_factor)
    return list_factor

def ExponentialDecay2(decay_factor, list_length):
    #指数衰减加权,与前一个函数计算有差异，这个衰减的更快，基本上只有前两个点的权重
    list_factor=np.zeros(list_length)
    list_factor[0]=decay_factor
    for ii in range(list_length - 2):
        list_factor[ii+1] = decay_factor * (1 - np.sum(list_factor[0:ii + 1]))# 左开右闭
    list_factor[-1]=1-np.sum(list_factor[0:list_length-1])
    return list_factor

def CalYawAim(weight, angle_list):
    # 输入weight表示权重，angle_list表示切线方向角序列
    # 输出加权后计算出的当地期望方向角
    factor_len=len(weight)
    angle_len=len(angle_list)
    yawAim=np.zeros(angle_len+1)# 补齐位数
    temp_angle=np.zeros(angle_len+factor_len)# 补齐位数
    temp_angle[0:angle_len]=angle_list
    temp_angle[angle_len:] = angle_list[-1]
    for ii in range(angle_len+1):
        temp=temp_angle[ii:ii+factor_len].copy()# 面向对象
        if np.ptp(temp)>300.0:# 角度差过大，需要转换
            temp[temp>180]-=360
        yawAim[ii]=np.dot(temp, weight)
    yawAim[yawAim<0]+=360
    return yawAim

def simplified2(latlng,dist_similar=0.2):
    # 输入latlng是np.array数组
    # dist_similar控制点与点最小间距，小于dist_similar时删除点
    # 输出latlng也是np.array数组

    # 去除重复数据
    _, ia= np.unique(latlng, return_index=True, axis=0)
    latlng=latlng[np.sort(ia),:]
    # 去除相近数据
    # 使用西安附近经纬度拟合
    dist_lla = np.linalg.norm(np.diff(latlng, axis=0), ord=2, axis=1) * 1.015271759128623e+05 + 0.006120193348871
    while sum(dist_lla<dist_similar):
        index=np.insert((dist_lla>=0.2), 0, [True], 0)
        latlng=latlng[index]
        dist_lla = np.linalg.norm(np.diff(latlng, axis=0), ord=2, axis=1) * 1.015271759128623e+05 + 0.006120193348871
    return latlng

def getYawAim(latlng):
    # 计算每点处期望方向角
    # 输入latlng是np.array数组
    # 输出yawAim也是np.array数组
    NorthEast=lla2ned_2D(latlng, latlng[0, 0], latlng[0, 1])
    # 计算切线方向角
    tangent_angle=np.rad2deg(np.arctan2(np.diff(NorthEast[:, 1]), np.diff(NorthEast[:, 0])))
    # 角度限幅至0~360
    tangent_angle[tangent_angle < 0]+=360
    # 计算加权系数
    WeightFactor=ExponentialDecay(0.7,list_length=5)#指数衰减加权
    # WeightFactor=np.ones(5)*1/5.0 #平均加权
    # 计算每点处期望方向角
    yawAim=CalYawAim(WeightFactor,tangent_angle)
    return yawAim

def getPoints(filename):
    global sidePoints
    with open(filename) as f:
        reader = csv.reader(f)
        for row in reader:
            try:
                sidePoints.append(Point(float(row[0]), float(row[1])))
            except ValueError:
                continue
    # **********************修改1******************
    # latlng和sidePoints转换
    # sidePoints = simplified(sidePoints)
    sidePoints = simplified2(sidePoints)

def index_cal(pos,index,SearchLength=10):
    # 计算当前点之后的SearchLength个点中距离当前最近的点的序号
    # 输入参数
    # pos为当前位置
    # index为当前点序号
    # SearchLength为搜索序列长度
    # 输出：距离最近的点序号
    global sidePoints
    if index+SearchLength>len(sidePoints):
        SearchLength=len(sidePoints)-index
    dist_list=list()
    for ii in range(SearchLength):
        dist_list.append(utils.getDistance(pos, sidePoints[ii+index]))
    index_max=dist_list.index(max(dist_list))
    return index_max+index

def PID_Turn(pShip, pAim, yawShip, gz):  # 原地转弯PID
    global gz_last  # 上一时刻角速度
    PID_P_gz = 5
    # PID_D_gz 在下面if结构中定义
    yawAim = utils.getAimYaw(pShip, pAim)  # 期望方向角
    error = utils.angleDiff(yawAim, yawShip)  # 角度差
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


def PID_Go(pShip, pAim, yawShip, gz):  # 直行段PID
    global thrust_last  # 上一时刻油门值
    PID_P = 0.5
    # PID_D = 0 在下面if结构中定义
    yawAim = utils.getAimYaw(pShip, pAim)  # 期望方向角
    error = utils.angleDiff(yawAim, yawShip)  # 方向角误差
    dist = utils.getDistance(pShip, pAim)  # 距离误差
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
        err_speed = tar_speed-speed
        thrust = thrust_last+err_speed*50
    thrust = abs(thrust)
    thrust = max(thrust, 10)  # 下限是10
    thrust = min(thrust, 100)  # 上限100
    thrust_last = thrust
    return int(dire), thrust


def serial_recv_thread():
    global yaw, speed, gz, ser
    ser = serial.Serial("/dev/ttyAMA0", 115200)
    gz = 0
    flag_ang_control = 0
    flag_finish = False
    while True:
        try:
            msg = ser.readline().strip()
            if msg.startswith('$') and msg.endswith('#'):
                datas = msg.replace('$', '').replace('#', '').split(';')
                if len(datas) >= 10:
                    yaw = float(datas[3])
                    speed = float(datas[7])
            elif msg.startswith('&') and msg.endswith('#'):
                datas = msg.replace('&', '').replace('#', '').split(';')
                gz = float(datas[0])
        except KeyboardInterrupt:
            break
        except Exception:
            continue
    ser.close()


if __name__ == '__main__':
    g.init()
    # 开启串口接收线程
    thread.start_new_thread(serial_recv_thread, ())
    # 开启RTK线程
    thread.start_new_thread(ntrip.fun, ())
    # 等待定位成功
    while g.getValue('lat') == None or g.getValue('lat') == 0:
        time.sleep(1)
        pass
    logging.info('已定位')
    # 获取导航点
    getPoints('gps.csv')
    # **********************修改1******************
    # latlng和sidePoints需要转换
    yawAim=getYawAim(latlng)
    if len(sidePoints) > 1:
        logging.info('已加载导航点，共有%d个点' % len(sidePoints))
        index = 0
        pos = Point(g.getValue('lat'), g.getValue('lng'))
        # 判断刚开始时，是否需要原地转
        state = State.TURN if utils.getDistance(
            pos, sidePoints[0]) > 3 else State.GO
        while index < len(sidePoints):
            pos = Point(g.getValue('lat'), g.getValue('lng'))
            if state == State.TURN:
                thrust = 0
                # dire = PID_Turn(pos, sidePoints[index], yaw, -gz)
                # **********************修改2******************
                # PID_ Turn增加参数
                dire = PID_Turn(pos, sidePoints[index],yawAim[index], yaw, -gz)
                if dire == 0:
                    # 转弯完毕，进入直行阶段
                    state = State.GO
            elif state == State.GO:
                dire, thrust = PID_Go(pos, sidePoints[index], yaw, -gz)

            # 已到达
            dist = utils.getDistance(pos, sidePoints[index])
            # if dist < 1:
            #     if index == 0:
            #         state = State.TURN
            #     index += 1
            #     if index == len(sidePoints):
            #         # 跑完
            #         thrust = 0
            #         dire = 0
            # 将最近点序号作为当前序号
            index = index_cal(pos,index,SearchLength=10)#函数中使用了global量sidePoints
            if index == 0:
                state = State.TURN
            if index == len(sidePoints):
                # 跑完
                thrust = 0
                dire = 0
            ser.write(utils.parityGenerate("$E;%d,%d#" % (thrust, dire)))
            logging.info("%s\t%d\t%.1f\t%.1f\t%d\t%.9f\t%.9f" %
                         (state, index, thrust, dire, dist, pos.lat, pos.lng))
            time.sleep(1)
        logging.info('导航结束')
        while True:
            pass
