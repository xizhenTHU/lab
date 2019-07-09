import numpy as np
import math
import matplotlib.pyplot as plt

def RemoveSimilarData(latlng, dist_similar=0.2):
    # 使用西安附近经纬度拟合
    dist_lla = np.linalg.norm(np.diff(latlng, axis=0), ord=2, axis=1) * 1.015271759128623e+05 + 0.006120193348871
    while sum(dist_lla<dist_similar):
        index=np.insert((dist_lla>=0.2), 0, [True], 0)
        latlng=latlng[index]
        dist_lla = np.linalg.norm(np.diff(latlng, axis=0), ord=2, axis=1) * 1.015271759128623e+05 + 0.006120193348871
    return latlng

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

def getYawAim(weight, angle_list):
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

latlng=np.loadtxt('gps.txt')
# latlng=np.loadtxt('gps.csv', delimiter=',')
# 去除重复数据
_, ia= np.unique(latlng, return_index=True, axis=0)
latlng= latlng[np.sort(ia), :]
# 去除相近数据
latlng=RemoveSimilarData(latlng, dist_similar=0.2)
# 坐标系转换
NorthEast=lla2ned_2D(latlng, latlng[0, 0], latlng[0, 1])
# 计算切线方向角
tangent_angle=np.rad2deg(np.arctan2(np.diff(NorthEast[:, 1]), np.diff(NorthEast[:, 0])))
# 角度限幅至0~360
tangent_angle[tangent_angle < 0]+=360
# 计算加权系数
WeightFactor=ExponentialDecay(0.7,list_length=5)#指数衰减加权
# WeightFactor=np.ones(5)*1/5.0 #平均加权
# 计算每点处期望方向角
yawAim=getYawAim(WeightFactor,tangent_angle)

plt.figure(1)
plt.plot(NorthEast[:,1],NorthEast[:,0], 'k-.')
# 切线方向
plt.quiver(NorthEast[:,1],NorthEast[:,0], np.sin(np.deg2rad(tangent_angle)), np.cos(np.deg2rad(tangent_angle)), color='r')
# 加权后方向
plt.quiver(NorthEast[:,1],NorthEast[:,0], np.sin(np.deg2rad(yawAim)), np.cos(np.deg2rad(yawAim)), color='b')
plt.axis("equal")
plt.xlabel('East')
plt.ylabel('North')
plt.show()

# plt.figure(2)
# kk=yawAim[0:-1]-tangent_angle
# kk[kk>180]-=360
# kk[kk<-180]+=360
# plt.plot(abs(kk), 'k-.')
# plt.xlabel('num')
# plt.ylabel('dif')
# plt.show()
#
# plt.figure(1)
# plt.plot(NorthEast[192-2:192+6,1],NorthEast[192-2:192+6,0], 'k-.')
# plt.quiver(NorthEast[192-2:192+6,1],NorthEast[192-2:192+6,0], np.sin(np.deg2rad(tangent_angle[192-2:192+6])), np.cos(np.deg2rad(tangent_angle[192-2:192+6])), color='r')
# plt.quiver(NorthEast[192-2:192+6,1],NorthEast[192-2:192+6,0], np.sin(np.deg2rad(yawAim[192-2:192+6])), np.cos(np.deg2rad(yawAim[192-2:192+6])), color='b')
# plt.axis("equal")
# plt.xlabel('East')
# plt.ylabel('North')
# plt.show()