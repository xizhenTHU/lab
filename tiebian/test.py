
import numpy as np
from enum import IntEnum
import csv
from math import asin, acos, pow, sqrt, sin, cos, pi
import matplotlib.pyplot as plt

class State(IntEnum):
    TURN = 1
    GO = 2

class Point:
    def __init__(self, lat, lng):
        self.lat = lat
        self.lng = lng
    
    def __str__(self):
        # return "Lat:%.6f\tLng:%.6f" % (self.lat, self.lng)
        return "%.6f,%.6f" % (self.lat, self.lng)   

def simplified2(latlng,dist_similar=0.2):
    # 输入latlng是np.array数组
    # dist_similar控制点与点间距，小于dist_similar时删除点
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

def radian(value):
    return value * pi / 180

def getDistance(p1, p2):
    radlat1 = radian(p1.lat)
    radlat2 = radian(p2.lat)
    a = radlat1 - radlat2
    b = radian(p1.lng) - radian(p2.lng)

    dst = 2 * asin((sqrt(pow(sin(a / 2), 2) + cos(radlat1)
                         * cos(radlat2) * pow(sin(b / 2), 2))))
    dst = dst * 6378.137 * 1000

    return dst

sidePoints=list()
def getPoints(filename):
    global sidePoints
    with open(filename) as f:
        reader = csv.reader(f)
        for row in reader:
            try:
                sidePoints.append(Point(float(row[0]), float(row[1])))
            except ValueError:
                continue

def index_cal(pos,index,SearchLength=10):
    # 计算当前点之后的SearchLength个点中距离当前最近的点的序号
    # 输入参数
    # pos为当前位置
    # index为当前点序号
    # SearchLength为搜索序列长度
    # 输出：距离最近点的序号
    global sidePoints
    if index+SearchLength>len(sidePoints):
        SearchLength=len(sidePoints)-index
    dist_list=list()
    for ii in range(SearchLength):
        dist_list.append(getDistance(pos, sidePoints[ii+index]))
    index_max=dist_list.index(max(dist_list))
    return index_max+index





# data=np.loadtxt('gps.txt')
# # data=np.loadtxt('gps.csv',delimiter=',')
# data=simplified2(data,dist_similar=0.2)
getPoints('gps2.csv')
pos = Point(34.24849467, 108.8945288)
index=0
index=index_cal(pos,index,SearchLength=5)
if index == len(sidePoints):
    # 跑完
    thrust = 0
    dire = 0

aa = [1,2,3,4,5]
aa.index(max(aa))
print(aa.index(max(aa)))
# sidePoints=[1,2,3,4,5]


# 已到达
# dist = utils.getDistance(pos, sidePoints[index])
# if dist < 1:
#     if index == 0:
#         state = State.TURN
#     index += 1
#     if index == len(sidePoints):
#         # 跑完
#         thrust = 0
#         dire = 0