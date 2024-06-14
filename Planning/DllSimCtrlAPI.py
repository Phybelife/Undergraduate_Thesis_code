import socket
import threading
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import struct
import math
import sys
import copy
import os
import cv2
import numpy as np
import ctypes

class DllSimCtrlAPI:

    # constructor function
    def __init__(self,CopterID = 1, ip='127.0.0.1'):
        self.ip = ip
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
        self.CopterID = CopterID

    def fillList(self,data,inLen):
        if isinstance(data, np.ndarray):
            data = data.tolist()
            
        if isinstance(data, list) and len(data)==inLen:
            return data
        else:
            if isinstance(data, list):
                datLen = len(data)
                if datLen<inLen:
                    data = data + [0]* (inLen-datLen)
                    
                if datLen>inLen:
                    data = data[0:inLen]
            else:
                data = [data] + [0]* (inLen-1)
        return data
        
    # 发送一个16维double型数据到指定端口，本接口可用于与Simulink通信
    # struct CustomData{
    #     int checksum;
    #     int CopterID;
    #     double data[16];
    # } ii16d 136包长
    # data 支持1-16维的numpy或list向量
    def sendCustomData(self,CopterID=0,data=[0]*16,checksum=123456,port=50000,IP='127.0.0.1'):
        """ Send 16d message to desired IP and port
        """
        data=self.fillList(data,16)
        buf = struct.pack("ii16d",checksum,CopterID,*data)
        self.udp_socket.sendto(buf, (IP, port))   
   
    # 发送到DLL模型的inSILInts and inSILFLoats接口
    #    struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    #    }
    # inSILInts and inSILFLoats will send to DLL model's inputs
    # CopterID is the vehicle you want to send, if copterID=-1 then it will send to yourself.
    # inSILInts 支持1-8维的numpy或list向量
    # inSILFLoats 支持1-20维的numpy或list向量
    def sendSILIntFloat(self,inSILInts=[0]*8,inSILFLoats=[0]*20,copterID=-1):
        checkSum=1234567897
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        inSILInts=self.fillList(inSILInts,8)
        inSILFLoats=self.fillList(inSILFLoats,20)
        #inSILInts=int(inSILInts)
        buf = struct.pack("10i20f",checkSum,ID,*inSILInts,*inSILFLoats)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    # 发送到DLL模型的inDoubCtrls接口
    # struct DllInDoubCtrls{
    #     int checksum;//校验码1234567897
    #     int CopterID; // 飞机的ID
    #     double inDoubCtrls[28];//28维的double型输入
    # };
    # inSILInts and inSILDoubs will send to DLL model's inputs
    # CopterID is the vehicle you want to send, if copterID=-1 then it will send to yourself.
    # inSILInts 支持1-8维的numpy或list向量
    # inSILDoubs 支持1-20维的numpy或list向量
    def sendSILIntDouble(self,inSILInts=[0]*8,inSILDoubs=[0]*20,copterID=-1):
        checkSum=1234567897
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        inSILInts=self.fillList(inSILInts,8)
        inSILDoubs=self.fillList(inSILDoubs,20)
        buf = struct.pack("2i28d",checkSum,ID,*inSILInts,*inSILDoubs)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    # 数据结构体和sendSILIntDouble()一致，保留28位double数据精度，支持用户自定义输入至模型。
    # inDoubsCtrls 支持1-28维的numpy或list向量
    def sendInDoubCtrls(self,inDoubsCtrls=[0]*28,copterID=-1):
        checkSum=1234567897
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        inDoubsCtrls=self.fillList(inDoubsCtrls,28)
        buf = struct.pack("2i28d",checkSum,ID,*inDoubsCtrls)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    # 发送到DLL模型的InCtrlExt系列接口
    # CopterID is the vehicle you want to send, if copterID=-1 then it will send to yourself.
    # inSILInts 支持1-8维的numpy或list向量
    # inSILFLoats 支持1-20维的numpy或list向量
    # iDxNum=1-5，对应InCtrlExt1-InCtrlExt5端口
    def sendInCtrlExt(self,inSILInts=[0]*8,inSILFLoats=[0]*20,iDxNum=1,copterID=-1):
        checkSum=1234567800+iDxNum
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        # pack for SOut2SimulatorSimpleTime
        
        inSILInts=self.fillList(inSILInts,8)
        inSILFLoats=self.fillList(inSILFLoats,20)
        #inSILInts=int(inSILInts)
        buf = struct.pack("10i20f",checkSum,ID,*inSILInts,*inSILFLoats)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    # 数据结构体和sendSILIntDouble()一致，保留28位double数据精度，支持用户自定义输入至模型。
    # inDoubsCtrls 支持1-28维的numpy或list向量
    # iDxNum=1-5，对应InCtrlExt1-InCtrlExt5端口
    # 本接口对应double型方式发送，发送给指定InCtrlExt系列端口
    # update 表示是否立刻更新
    def sendInCtrlExtDoub(self,inDoubsCtrls=[0]*28,iDxNum=1,copterID=-1,update=True):
        #iDxNum=int(iDxNum)
        if iDxNum>=1 and iDxNum<=5: # 对应inCtrlExt1~5的发送端口
            checkSum=1234567800+iDxNum
        
        if not update:
            checkSum=checkSum + 100 # 不立刻更新
            
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        inDoubsCtrls=self.fillList(inDoubsCtrls,28)
        buf = struct.pack("2i28d",checkSum,ID,*inDoubsCtrls)
        self.udp_socket.sendto(buf, (self.ip, PortNum))
        
    # inDoubsCtrls 支持1-140维的numpy或list向量
    # 本接口会一次性发给InCtrlExt1-InCtrlExt5端口
    def sendInCtrlExtAll(self,inDoubsCtrls=[0]*140,copterID=-1):
        lenCtrl=math.ceil(len(inDoubsCtrls)/28.0)
        if lenCtrl>5:
            lenCtrl=5
        inDoubsCtrls=self.fillList(inDoubsCtrls,140)
        for i in range(lenCtrl): 
            if i==lenCtrl-1:
                self.sendInCtrlExtDoub(inDoubsCtrls[28*i:28*(i+1)],i+1,copterID,True) # 更新
            else:
                self.sendInCtrlExtDoub(inDoubsCtrls[28*i:28*(i+1)],i+1,copterID,False) # 等数全部发过去再更新
        
    # //输出到CopterSim DLL模型的FaultParamAPI.FaultInParams[32]参数
    # struct PX4ModelInParams{
    #     int checksum;//1234567891
    #     uint32 Bitmask;
    #     double InParams[32];
    # };
    #struct.pack iI32d
    def sendModelInParams(self,Bitmask,InParams,copterID=-1):
        checkSum=1234567891
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        InParams=self.fillList(InParams,32)
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("iI32d",checkSum,Bitmask,*InParams)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    # //输出到CopterSim DLL模型的FaultParamAPI.InitInParams参数
    # struct PX4ModelInParams{
    #     int checksum;//1234567892 for InitInParams //注意，这里主要是checksum的区别
    #     uint32 Bitmask;
    #     double InParams[32];
    # };
    #struct.pack iI32d
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    # bitmask has 32bits, each bit == 1 means modify the params in ModelInParams
    def sendInitInParams(self,Bitmask=0,InParams=[0]*32,copterID=-1):
        checkSum=1234567892 # 这里主要是checksum的区别
        #Bitmask=ctypes.c_uint32(Bitmask).value
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        InParams=self.fillList(InParams,32)
        buf = struct.pack("iI32d",checkSum,Bitmask,*InParams)
        self.udp_socket.sendto(buf, (self.ip, PortNum))


    # //输出到CopterSim DLL模型的FaultParamAPI.DynModiParams参数
    # struct PX4DynModiParams参数{
    #     int checksum;//1234567893 这里主要是checksum的区别
    #     int CopterID;//飞机ID号，这里是为了填充数位
    #     uint64 Bitmask;
    #     double InParams[64];
    # };
    #struct.pack iiQ64d
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    # bitmask has 32bits, each bit == 1 means modify the params in ModelInParams
    def sendDynModiParams(self,Bitmask=0,InParams=[0]*64,copterID=-1):
        checkSum=1234567893 # 这里主要是checksum的区别
        #Bitmask=ctypes.c_uint32(Bitmask).value
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        InParams=self.fillList(InParams,64)
        buf = struct.pack("iiQ64d",checkSum,copterID,Bitmask,*InParams)
        self.udp_socket.sendto(buf, (self.ip, PortNum))


    # //输出到CopterSim DLL模型的InFromUE接口
    # struct UEToCopterDataD{
    #     int checksum; //1234567899为校验ID
    #     int CopterID; //发出本消息的飞机ID
    #     double inFromUE[32]; //通过蓝图发出的数据
    # }
    #struct.pack ii32d
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    def sendUE2Coptersim(self,inFromUE=[0]*32,copterID=-1):
        checkSum=1234567899 # 这里主要是checksum的区别
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        inFromUE=self.fillList(inFromUE,32)
        buf = struct.pack("ii32d",checkSum,copterID,*inFromUE)
        self.udp_socket.sendto(buf, (self.ip, PortNum))


    # //输出到CopterSim DLL模型的inFloatsCollision接口
    # struct Ue4RayTraceDrone {
    #     int checksum;//校验码1234567890
    #     int CopterID;
    #     float size;
    #     float velE[3];
    #     float ray[6];//前后左右上下
    #     float posE[3];
    # }
    #struct.pack ii13f
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    def sendFloatsColl(self,size=0,velE=[0,0,0],ray=[0,0,0,0,0,0],posE=[0,0,0],copterID=-1):
        checkSum=1234567890
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("ii13f",checkSum,copterID,size,*velE,*ray,*posE)
        self.udp_socket.sendto(buf, (self.ip, PortNum))


    # //输出到CopterSim DLL模型的inCollision20d接口
    # struct Ue4Coll20d {
    #     int checksum; //校验码1234567880
    #     int CopterID;
    #     float inCollision20d[20];
    # }
    #struct.pack ii20f
    # CopterID is the vehicle you want to send, if copterID then it will send to yourself.
    # Coll20d是1到20维的numpy或list向量
    # 最终会发给DLL的inCollision20d输入口
    def sendColl20d(self,Coll20d=[0]*20,copterID=-1):
        checkSum=1234567880 # 这里主要是checksum的区别
        Coll20d=self.fillList(Coll20d,20)
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("ii20f",checkSum,copterID,*Coll20d)
        self.udp_socket.sendto(buf, (self.ip, PortNum))

    # TerrIn15d是1到15维的numpy或list向量
    # 最终会发给DLL的TerrainIn15d输入口
    def sendTerrIn15d(self,TerrIn15d=[0]*15,copterID=-1):
        checkSum=1234567881 # 这里主要是checksum的区别，相对sendColl20d
        TerrIn15d=self.fillList(TerrIn15d,20)
        ID=copterID
        if copterID<=0:
            ID=self.CopterID
        PortNum = 30100+(ID-1)*2 
        buf = struct.pack("ii20f",checkSum,copterID,*TerrIn15d) # 和sendColl20d同样通道
        self.udp_socket.sendto(buf, (self.ip, PortNum))

class RflySimCP:
    """
    RflySim 综合模型控制协议
    """
    # inSILInts数据对应的下
    ILen = 8
    ICmd = 0
    IOffboard = 1
    # 整数型纬度
    ILat = 6
    # 整数型高度
    ILon = 7

    # inSILInts[0]指令标志
    CmdEn = 1
    CmdSIL = 1 << 1
    CmdArmed = 1 << 2
    CmdTakeoff = 1 << 8
    CmdPosition = 1 << 9
    CmdLand = 1 << 10
    CmdReturn = 1 << 11
    CmdOffboard_Pos = 1 << 16
    CmdOffboard_Att = 1 << 17
    CmdBase = 3

    # inSILInts[1] Offboard标志
    HasPos = 1
    HasVel = 1 << 1
    HasAcc = 1 << 2
    HasYaw = 1 << 3
    HasYawRate = 1 << 4
    HasAtt = 1 << 8
    HasRollRate = 1 << 9
    HasPitchRate = 1 << 10
    HasThrust = 1 << 11
    HasFull = 1 << 15
    NED = 1 << 16
    Global = 1 << 17

    # inSILFloats位置、速度、加速度、姿态、角速率、油门数据对应的起始下标
    FLen = 20
    FPos = 0
    FVel = 3
    FAcc = 6
    FAtt = 9
    FAttRate = 12
    FThrust = 15

    @staticmethod
    def IsBitSet(num, bit):
        mask = 1 << bit
        return (num & mask) != 0

    @staticmethod
    def IsPosSet(num):
        return RflySimCP.IsBitSet(num, 0)
