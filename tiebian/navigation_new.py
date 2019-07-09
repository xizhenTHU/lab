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
speed = 0# global参量
gz = 0
gz_last = 0  # global参量
thrust_last = 0  # global参量
ser = None


def getPoints(filename):
    global sidePoints
    with open(filename) as f:
        reader = csv.reader(f)
        for row in reader:
            try:
                sidePoints.append(Point(float(row[0]), float(row[1])))
            except ValueError:
                continue
    sidePoints = simplified(sidePoints)


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
    if dist < 10:  # 接近段
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
                dire = PID_Turn(pos, sidePoints[index], yaw, -gz)
                if dire == 0:
                    # 转弯完毕，进入直行阶段
                    state = State.GO
            elif state == State.GO:
                dire, thrust = PID_Go(pos, sidePoints[index], yaw, -gz)

            # 已到达
            dist = utils.getDistance(pos, sidePoints[index])
            if dist < 1:
                if index == 0:
                    state = State.TURN
                index += 1
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
