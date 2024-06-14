import time
import math
import sys
import VisionCaptureApi
import PX4MavCtrlV4 as PX4MavCtrl
import cv2
import matplotlib.pyplot as plt
import UE4CtrlAPI
ue = UE4CtrlAPI.UE4CtrlAPI()
import numpy as np
import socket
import struct

fig = plt.figure()


def viz_matplot(points, name="lidar"):
    plt.clf()
    plt.title(name)
    ax = fig.add_subplot(111, projection='3d')
    x = points[:, 0]  # x position of point
    y = -points[:, 1]  # y position of point
    z = points[:, 2]  # z position of point
    
    diff = np.abs(z - vis.ImgData[i, 3])
    indices = np.where(diff < 0.1)
    x_values = x[indices]
    y_values = y[indices]
    
    ax.scatter(y,   # y
               x,   # x
               z,   # z
               c=z,  # height data for color
               cmap='rainbow',
               marker=".")
    ax.axis()
    plt.pause(0.0001)
    plt.ioff()


def viz_matplot_2d(points, height, name="lidar"):
    plt.clf()
    plt.title(name)
    x = points[:, 0]  # x position of point
    y = -points[:, 1]  # y position of point
    z = points[:, 2]  # z position of point
    
    diff = np.abs(z - height)
    indices = np.where(diff < 0.1)
    x_values = x[indices]
    y_values = y[indices]
    
    plt.scatter(x_values, y_values, marker=".", c='black')
    plt.axis()
    plt.pause(0.0001)
    plt.ioff()


def send_cmd(x_d, y_d, z_d):
    # 创建一个 INET, Datagram (UDP) 套接字
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    doubles = [x_d, y_d, z_d]

    # 将double数据转换为字节
    data = struct.pack('d'*len(doubles), *doubles)

    # 定义Simulink服务器的主机名和端口号
    host = "127.0.0.1"
    port = 1230

    # 发送数据
    s.sendto(data, (host, port))

    # 关闭套接字
    s.close()


# Create a new MAVLink communication instance, UDP sending port (CopterSim’s receving port) is 20100
mav = PX4MavCtrl.PX4MavCtrler(1)

# The IP should be specified by the other computer
vis = VisionCaptureApi.VisionCaptureApi()

# Send command to UE4 Window 1 to change resolution
# 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
ue.sendUE4Cmd('r.setres 1280x720w', 0)
ue.sendUE4Cmd('t.MaxFPS 90', 0)  # 设置UE4最大刷新频率，同时也是取图频率
time.sleep(2)

# VisionCaptureApi 中的配置函数
vis.jsonLoad()  # 加载Config.json中的传感器配置文件

# vis.RemotSendIP = '192.168.3.80'
# 注意，手动修改RemotSendIP的值，可以将图片发送到本地址
# 如果不修改这个值，那么发送的IP地址为json文件中SendProtocol[1:4]定义的IP
# 图片的发送端口，为json中SendProtocol[5]定义好的。

isSuss = vis.sendReqToUE4()  # 向RflySim3D发送取图请求，并验证
if not isSuss:  # 如果请求取图失败，则退出
    sys.exit(0)
vis.startImgCap()  # 开启取图，并启用共享内存图像转发，转发到填写的目录
isShowCloud = True
# 下面的程序按30Hz读取点云数据
# 注意：如果要去看点云的实际接收频率，请搜索img_udp_thrdNew函数
lastTime = time.time()
num = 0
lastClock = time.time()
x=[0,150,150,300]
y=[250,250,25,25]

for i in range(4):   
    send_cmd(x[i],y[i], 0)
    # print('num:',num,x.astar(num),y.astar(num))
    # num=num+1
    lastTime = lastTime + 2.5
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime)
    else:
        lastTime = time.time()

    for i in range(len(vis.hasData)):
        if vis.hasData[i]:
            # vis.Img[i].tolist()  #在这里获取并处理你的点云
            # if(isShowCloud):
            #     isShowCloud = False
            height = vis.ImgData[i][2] - 1.47
            print('height: ', height)
            viz_matplot_2d(vis.Img[i], height)
            print('PosAngNum: ', vis.ImgData[i])  # 雷达在地面坐标系下的位置、姿态和点云总数
            print('TimeStmp: ', vis.timeStmp[i])  # 当前的数据时间戳
            vis.hasData[i] = False
