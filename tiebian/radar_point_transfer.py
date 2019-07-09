import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt



def get_radar_point_latlng(x_radar, y_radar, yaw, lat, lng):
    # 处理一个数据
    # x_radar为雷达点在雷达系下x坐标(船头前向)，单位米
    # y_radar为雷达点在雷达系下y坐标(船右手侧方向)，单位米
    # yaw为当前船方向角，单位度
    # lat为当前纬度，单位度
    # lng为当前经度，单位度
    # 函数返回
    # lat_out为雷达点纬度，单位度
    # lng_out为雷达点经度，单位度

    re = 6367444.65712259
    sin_lat = math.sin(np.deg2rad(lat))
    sin_lon = math.sin(np.deg2rad(lng))
    cos_lon = math.cos(np.deg2rad(lng))
    cos_lat = math.cos(np.deg2rad(lat))
    cos_yaw = math.cos(np.deg2rad(yaw))
    sin_yaw = math.sin(np.deg2rad(yaw))
    y_east = cos_yaw*x_radar+sin_yaw*y_radar
    x_north = -sin_yaw*x_radar+cos_yaw*y_radar
    trans_mat = np.array([[-sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon],
                          [-sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon],
                          [cos_lat, 0, -sin_lat]])
    xyz_ecef = np.dot(trans_mat, np.array([x_north, y_east, -re])[:, None])
    lng_out = np.rad2deg(math.atan2(xyz_ecef[1], xyz_ecef[0]))
    lat_out = np.rad2deg(math.asin(xyz_ecef[2] / re))
    return lat_out, lng_out


def get_radar_latlng(x_radar, y_radar, yaw, lat, lng):
    # 处理一包数据
    # x_radar为n行1列numpy 数组
    # y_radar为n行1列numpy 数组
    # yaw为当前船方向角，单位度
    # lat为当前纬度，单位度
    # lng为当前经度，单位度
    # 函数返回
    # lat_out为n行1列numpy 数组
    # lng_out为n行1列numpy 数组
    num = np.shape(x_radar)[0]
    lat_out = np.zeros(num)
    lng_out = np.zeros(num)
    for ii in range(num):
        lat_point, lng_point = get_radar_point_latlng(
            x_radar[ii], y_radar[ii], yaw, lat, lng)
        lat_out[ii] = lat_point
        lng_out[ii] = lng_point
    return lat_out.reshape(-1,1), lng_out.reshape(-1,1)


if __name__ == "__main__":
    data=pd.read_csv('data.csv')
    data=data.loc[abs(data.loc[:,'x'])<5,:] 
    data=data.loc[abs(data.loc[:,'y'])<10,:] 
    data=data.loc[abs(data.loc[:,'y'])>1.5,:] 
    data=data.loc[data.loc[:,'v']<100,:] 
    data2=np.loadtxt('time_2019_05_31_16_21_47.txt')
    plt.rcParams['figure.figsize'] = (50.0, 50.0)
    for ii in range(data2.shape[0]):
        if ii==0:
            continue
        index_bao=int(data2[ii,0]-1)
        lat =data2[ii,3]
        lng=data2[ii,2]
        yaw=data2[ii,4]
        data_cat=data[data['index']==index_bao]
        if data_cat.empty:
            continue
        x_input=np.array(data_cat['x']).reshape(-1,1)
        y_input=np.array(data_cat['y']).reshape(-1,1)

        lat_out, lng_out = get_radar_latlng(x_input, y_input, yaw, lat, lng)
        plt.scatter(lat_out,lng_out, s=5)
    plt.show()

    # x_input = np.array([[1.], [2.], [3.]])
    # y_input = np.array([[4.], [5.], [6.]])
    # yaw = 340.2
    # lat = 34.24701743999026
    # lng = 108.89901353771107
    # lat_out, lng_out = get_radar_latlng(x_input, y_input, yaw, lat, lng)
    # print(lat_out)
    # print(lng_out)
