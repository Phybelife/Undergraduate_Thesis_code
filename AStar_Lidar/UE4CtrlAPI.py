import socket
import threading
import time
import struct
import sys
import copy
import os
import cv2
import numpy as np


class PX4SILIntFloat:
    # //输出到CopterSim DLL模型的SILints和SILFloats数据
    # struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    # };
    # struct.pack 10i20f
    def __init__(self, iv=None):
        if iv != None:
            self.checksum = iv[0]
            self.CopterID = iv[1]
            self.inSILInts = iv[2:10]
            self.inSILFLoats = iv[10:30]
            return
        self.checksum = 0
        self.CopterID = 0
        self.inSILInts = [0, 0, 0, 0, 0, 0, 0, 0]
        self.inSILFLoats = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    # def __init__(self, iv):
    #     self.checksum = iv[0]
    #     self.CopterID = iv[1]
    #     self.inSILInts = iv[2:10]
    #     self.inSILFLoats = iv[10:30]


# struct reqVeCrashData {
# 	int checksum; //数据包校验码1234567897
# 	int copterID; //当前飞机的ID号
# 	int vehicleType; //当前飞机的样式
# 	int CrashType;//碰撞物体类型，-2表示地面，-1表示场景静态物体，0表示无碰撞，1以上表示被碰飞机的ID号
# 	double runnedTime; //当前飞机的时间戳
# 	float VelE[3]; // 当前飞机的速度
# 	float PosE[3]; //当前飞机的位置
# 	float CrashPos[3];//碰撞点的坐标
# 	float targetPos[3];//被碰物体的中心坐标
# 	float AngEuler[3]; //当前飞机的欧拉角
# 	float MotorRPMS[8]; //当前飞机的电机转速
#   float ray[6]; //飞机的前后左右上下扫描线
# 	char CrashedName[16];//被碰物体的名字
#  } 4i1d29f20s


class reqVeCrashData:
    def __init__(self, iv=None):
        if iv != None:
            self.CopyData(iv)
            return
        self.checksum = 1234567897
        self.copterID = 0
        self.vehicleType = 0
        self.CrashType = 0
        self.runnedTime = 0
        self.VelE = [0, 0, 0]
        self.PosE = [0, 0, 0]
        self.CrashPos = [0, 0, 0]
        self.targetPos = [0, 0, 0]
        self.AngEuler = [0, 0, 0]
        self.MotorRPMS = [0, 0, 0, 0, 0, 0, 0, 0]
        self.ray = [0, 0, 0, 0, 0, 0]
        self.CrashedName = ""
        self.hasUpdate = True

    # def __init__(self, iv):
    #     self.CopyData(iv)

    def CopyData(self, iv):
        self.checksum = iv[0]
        self.copterID = iv[1]
        self.vehicleType = iv[2]
        self.CrashType = iv[3]
        self.runnedTime = iv[4]
        self.VelE = iv[5:8]
        self.PosE = iv[8:11]
        self.CrashPos = iv[11:14]
        self.targetPos = iv[14:17]
        self.AngEuler = iv[17:20]
        self.MotorRPMS = iv[20:28]
        self.ray = iv[28:34]
        self.CrashedName = iv[34].decode("UTF-8")
        self.CrashedName = self.CrashedName.strip(b"\x00".decode())
        self.hasUpdate = True


# struct CoptReqData { //80
# 	int checksum = 0; //1234567891作为校验
# 	int CopterID;//飞机ID
# 	float PosUE[3]; //物体中心位置（人为三维建模时指定，姿态坐标轴，不一定在几何中心）
# 	float angEuler[3];//物体欧拉角
# 	float angQuat[4];//物体四元数
# 	float boxOrigin[3];//物体几何中心坐标
# 	float BoxExtent[3];//物体外框长宽高的一半
# 	double timestmp;//时间戳
# };


class CoptReqData:  # 长度80, 2i16f1d
    def __init__(self, iv=None):
        if iv != None:
            self.CopyData(iv)
            return
        self.checksum = 0  # 1234567891作为校验
        self.CopterID = 0  # 飞机ID
        self.PosUE = [0, 0, 0]
        self.angEuler = [0, 0, 0]
        self.angQuat = [0, 0, 0, 0]
        self.boxOrigin = [0, 0, 0]
        self.BoxExtent = [0, 0, 0]
        self.timestmp = 0
        self.hasUpdate = True

    # def __init__(self, iv):
    #     self.CopyData(iv)

    def CopyData(self, iv):
        self.checksum = iv[0]
        self.CopterID = iv[1]
        self.PosUE = iv[2:5]
        self.angEuler = iv[5:8]
        self.angQuat = iv[8:12]
        self.boxOrigin = iv[12:15]
        self.BoxExtent = iv[15:18]
        self.timestmp = iv[18]
        self.hasUpdate = True

    def CopyDataOld(self, iv):
        self.checksum = iv[0]
        self.CopterID = iv[1]
        self.PosUE = iv[2:5]
        self.angEuler = iv[5:8]
        self.boxOrigin = iv[8:11]
        self.BoxExtent = iv[11:14]
        self.timestmp = iv[14]
        self.hasUpdate = True


# struct ObjReqData { //112
# 	int checksum = 0; //1234567891作为校验
# 	int seqID = 0;
# 	float PosUE[3]; //物体中心位置（人为三维建模时指定，姿态坐标轴，不一定在几何中心）
# 	float angEuler[3];//物体欧拉角
# 	float angQuat[4];//物体四元数
# 	float boxOrigin[3];//物体几何中心坐标
# 	float BoxExtent[3];//物体外框长宽高的一半
# 	double timestmp;//时间戳
# 	char ObjName[32] = { 0 };//碰物体的名字
# };


class ObjReqData:  # 长度112, 2i16f1d32s
    def __init__(self, iv=None):
        if iv != None:
            self.CopyData(iv)
            return
        self.checksum = 0  # 1234567891作为校验
        self.seqID = 0
        self.PosUE = [0, 0, 0]
        self.angEuler = [0, 0, 0]
        self.angQuat = [0, 0, 0, 0]
        self.boxOrigin = [0, 0, 0]
        self.BoxExtent = [0, 0, 0]
        self.timestmp = 0
        self.ObjName = ""
        self.hasUpdate = True

    # def __init__(self, iv):
    #     self.CopyData(iv)

    def CopyData(self, iv):
        self.checksum = iv[0]
        self.seqID = iv[1]
        self.PosUE = iv[2:5]
        self.angEuler = iv[5:8]
        self.angQuat = iv[8:12]
        self.boxOrigin = iv[12:15]
        self.BoxExtent = iv[15:18]
        self.timestmp = iv[18]
        self.ObjName = iv[19].decode("UTF-8")
        self.ObjName = self.ObjName.strip(b"\x00".decode())
        self.hasUpdate = True

    def CopyDataOld(self, iv):
        self.checksum = iv[0]
        self.seqID = iv[1]
        self.PosUE = iv[2:5]
        self.angEuler = iv[5:8]
        self.boxOrigin = iv[8:11]
        self.BoxExtent = iv[11:14]
        self.timestmp = iv[14]
        self.ObjName = iv[15].decode("UTF-8")
        self.ObjName = self.ObjName.strip(b"\x00".decode())
        self.hasUpdate = True


# struct CameraData { //72
# 	int checksum = 0;//1234567891
# 	int SeqID; //相机序号
# 	int TypeID;//相机类型
# 	int DataHeight;//像素高
# 	int DataWidth;//像素宽
# 	float CameraFOV;//相机视场角
# 	float PosUE[3]; //相机中心位置
# 	float angEuler[3];//相机欧拉角
# 	float angQuat[4];//相机四元数
# 	double timestmp;//时间戳
# };
class CameraData:  # 长度72, 5i11f1d
    def __init__(self, iv=None):
        if iv != None:
            self.CopyData(iv)
            return
        self.checksum = 0  # 1234567891作为校验
        self.SeqID = 0
        self.TypeID = 0
        self.DataHeight = 0
        self.DataWidth = 0
        self.CameraFOV = 0
        self.PosUE = [0, 0, 0]
        self.angEuler = [0, 0, 0]
        self.angQuat = [0, 0, 0, 0]
        self.timestmp = 0
        self.hasUpdate = True

    # def __init__(self, iv):
    #     self.CopyData(iv)

    def CopyData(self, iv):
        self.checksum = iv[0]
        self.SeqID = iv[1]
        self.TypeID = iv[2]
        self.DataHeight = iv[3]
        self.DataWidth = iv[4]
        self.CameraFOV = iv[5]
        self.PosUE = iv[6:9]
        self.angEuler = iv[9:12]
        self.angQuat = iv[12:16]
        self.timestmp = iv[16]
        self.hasUpdate = True

    def CopyDataOld(self, iv):
        self.checksum = iv[0]
        self.SeqID = iv[1]
        self.TypeID = iv[2]
        self.DataHeight = iv[3]
        self.DataWidth = iv[4]
        self.CameraFOV = iv[5]
        self.PosUE = iv[6:9]
        self.angEuler = iv[9:12]
        self.timestmp = iv[12]
        self.hasUpdate = True


# PX4 ueLink listen and control API and RflySim3D control API
class UE4CtrlAPI:
    """ """

    # constructor function
    def __init__(self, ip="127.0.0.1"):
        self.ip = ip
        self.startTime = time.time()  # 添加这行以初始化 startTime 属性

        self.udp_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )  # Create socket
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_socketUE4 = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )  # Create socket
        self.udp_socketUE4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.inSilVect = []
        self.inReqVect = []

        self.stopFlagUE4 = True
        self.CoptDataVect = []
        self.ObjDataVect = []
        self.CamDataVect = []

        self.hasMsgEvent = threading.Event()
        self.trueMsgEvent = threading.Event()

    def sendUE4Cmd(self, cmd, windowID=-1):
        """send command to control the display style of RflySim3D
        The available command are list as follows, the command string is as b'RflyShowTextTime txt time'
        RflyShowTextTime(String txt, float time)\\ let UE4 show txt with time second
        RflyShowText(String txt)\\  let UE4 show txt 5 second
        RflyChangeMapbyID(int id)\\ Change the map to ID (int number)
        RflyChangeMapbyName(String txt)\\ Change to map with name txt
        RflyChangeViewKeyCmd(String key, int num) \\ the same as press key + num on UE4
        RflyCameraPosAngAdd(float x, float y, float z,float roll,float pitch,float yaw) \\ move the camera with x-y-z(m) and roll-pitch-yaw(degree) related to current pos
        RflyCameraPosAng(float x, float y, float z, float roll, float pitch, float yaw) \\ set the camera with x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
        RflyCameraFovDegrees(float degrees) \\ change the cameras fov (degree)
        RflyChange3DModel(int CopterID, int veTypes=0) \\ change the vehicle 3D model to ID
        RflyChangeVehicleSize(int CopterID, float size=0) \\change vhielce's size
        RflyMoveVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ move the vehicle's  x-y-z(m) and roll-pitch-yaw(degree) related to current pos
        RflySetVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ set the vehilce's x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
        RflyScanTerrainH(float xLeftBottom(m), float yLeftBottom(m), float xRightTop(m), float yRightTop(m), float scanHeight(m), float scanInterval(m)) \\ send command to let UE4 scan the map to generate png and txt files
        RflyCesiumOriPos(double lat, double lon, double Alt) \\ change the lat, lon, Alt (degrees) of the Cesium map origin
        RflyClearCapture \\ clear the image capture unit
        struct Ue4CMD0{
            int checksum;
            char data[52];
        } i52s
        struct Ue4CMD{
            int checksum;
            char data[252];
        } i252s

        """

        # 如果是str类型，则转换为bytes类型
        if isinstance(cmd, str):
            cmd = cmd.encode()

        # print(type(cmd))
        if len(cmd) <= 51:
            buf = struct.pack("i52s", 1234567890, cmd)
        elif len(cmd) <= 249:
            buf = struct.pack("i252s", 1234567890, cmd)
        else:
            print("Error: Cmd is too long")
            return
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send

        # 如果是改变场景的命令，需要等待场景切换完成
        if "RflyChangeMap" in cmd.decode():
            time.sleep(0.5)

        if "RflyChangeMap" in cmd.decode():
            time.sleep(0.5)

    def sendUE4CmdNet(self, cmd):
        """send command to control all RflySim3D in LAN
        struct Ue4CMDNet { //总长度为96
            int checksum; //校验位，这里应该是1234567897
            char data[92];
        } i92s

        """

        # 如果是str类型，则转换为bytes类型
        if isinstance(cmd, str):
            cmd = cmd.encode()

        # print(type(cmd))
        if len(cmd) <= 91:
            buf = struct.pack("i92s", 1234567897, cmd)
        else:
            print("Error: Cmd is too long")
            return
        self.udp_socket.sendto(
            buf, ("224.0.0.10", 20009)
        )  # multicast address, send to all RflySim3Ds on all PC in LAN

        # 如果是改变场景的命令，需要等待场景切换完成
        if "RflyChangeMap" in cmd.decode():
            time.sleep(0.5)

        # 如果是改变场景的命令，需要等待场景切换完成
        if "RflyChangeMap" in cmd.decode():
            time.sleep(0.5)

    def sendUE4LabelID(
        self, CopterID=0, Txt="", fontSize=30, RGB=[255, 0, 0], windowID=-1
    ):
        dispTime = 1
        dispFlag = 0
        # dispFlag =0 且 dispTime>=0 表示更新ID行（第0行数据）
        self.sendUE4LabelMsg(CopterID, Txt, fontSize, RGB, dispTime, dispFlag, windowID)

    def sendUE4LabelMsg(
        self,
        CopterID=0,
        Txt="",
        fontSize=30,
        RGB=[255, 0, 0],
        dispTime=0,
        dispFlag=-1,
        windowID=-1,
    ):
        # struct CopterMsg {
        # int checksum; //校验位，这里必须设定为1234567899
        # int CopterID; //飞机的ID号，具体显示哪一个飞机。注意，如果CopterID<=0，则所有飞机都显示消息；如果CopterID大于0，则对应飞机显示消息
        # int dispFlag;//显示需要，如果flag<0，则消息会逐层累加，最多显示5条消息；如果flag=0，会清理所有消息；如果flag>0，则会更新对应的消息；如果flag大于当前消息总数，则消息顺延在末尾。
        # int RGB[3];//RGB的颜色，0~255，分别表示红、绿、蓝
        # float dispTime;//消失时间（单位秒）。如果<0，则立刻消失；如果=0，则永远显示；如果>0,则设定秒数后消失
        # float fontSize;//字体大小；默认是20；
        # char data[120];//显示的文字。
        # };6i2f120s
        # dispFlag <0 且 dispTime>=0 表示依次累加的方式添加消息
        # dispFlag <0 且 dispTime<0 表示清空所有消息
        # dispFlag =0 且 dispTime>=0 表示更新ID行（第0行数据）
        # dispFlag >=1 and <5 表示更新1到5行的消息，注意此时dispTime<0会删除本消息，若>=0会更新消息
        # dispFlag>当前消息数，则切换到累加模式
        data = Txt.encode("UTF-8")
        buf = struct.pack(
            "6i2f120s", 1234567899, CopterID, dispFlag, *RGB, dispTime, fontSize, data
        )
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send

    def sendUE4Attatch(self, CopterIDs, AttatchIDs, AttatchTypes, windowID=-1):
        """Send msg to UE4 to attach a vehicle to another (25 vehicles);
        CopterIDs,AttatchIDs,AttatchTypes can be a list with max len 25
        """
        # change the 1D variable to 1D list
        if isinstance(CopterIDs, int):
            CopterIDs = [CopterIDs]

        if isinstance(AttatchIDs, int):
            AttatchIDs = [AttatchIDs]

        if isinstance(AttatchTypes, int):
            AttatchTypes = [AttatchTypes]

        if (
            not isinstance(CopterIDs, list)
            or not isinstance(AttatchIDs, list)
            or not isinstance(AttatchTypes, list)
        ):
            print("Error: Wrong sendUE4Attatch input Type")
            return

        if (
            len(CopterIDs) != len(AttatchIDs)
            or len(CopterIDs) != len(AttatchTypes)
            or len(CopterIDs) > 25
        ):
            print("Error: Wrong sendUE4Attatch input dimension")
            return

        vLen = len(CopterIDs)
        if vLen < 25:  # Extend the IDs to 25D
            CopterIDs = CopterIDs + [0] * (25 - vLen)
            AttatchIDs = AttatchIDs + [0] * (25 - vLen)
            AttatchTypes = AttatchTypes + [0] * (25 - vLen)

        if vLen > 25:
            CopterIDs = CopterIDs[0:25]
            AttatchIDs = AttatchIDs[0:25]
            AttatchTypes = AttatchTypes[0:25]

        # struct VehicleAttatch25 {
        # 	int checksum;//1234567892
        # 	int CopterIDs[25];
        # 	int AttatchIDs[25];
        # 	int AttatchTypes[25];//0：正常模式，1：相对位置不相对姿态，2：相对位置+偏航（不相对俯仰和滚转），3：相对位置+全姿态（俯仰滚转偏航）
        # }i25i25i25i

        buf = struct.pack(
            "i25i25i25i", 1234567892, *CopterIDs, *AttatchIDs, *AttatchTypes
        )
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send

    def sendUE4Pos(
        self,
        copterID=1,
        vehicleType=3,
        MotorRPMSMean=0,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        windowID=-1,
    ):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states"""
        VelE = [0, 0, 0]
        self.sendUE4PosNew(
            copterID,
            vehicleType,
            PosE,
            AngEuler,
            VelE,
            [MotorRPMSMean] * 8,
            -1,
            windowID,
        )

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4Pos2Ground(
        self,
        copterID=1,
        vehicleType=3,
        MotorRPMSMean=0,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        windowID=-1,
    ):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states on the ground
        checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
        struct SOut2SimulatorSimple {
            int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
            int copterID;  //Vehicle ID
            int vehicleType;  //Vehicle type
            float MotorRPMSMean; // mean motor speed
            float PosE[3];   //NED vehicle position in earth frame (m)
            float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
        }  3i7f
        """
        buf = struct.pack(
            "3i7f",
            1234567891,
            copterID,
            vehicleType,
            MotorRPMSMean,
            PosE[0],
            PosE[1],
            PosE[2],
            AngEuler[0],
            AngEuler[1],
            AngEuler[2],
        )
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send

    def sendUE4PosScale(
        self,
        copterID=1,
        vehicleType=3,
        MotorRPMSMean=0,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        Scale=[1, 1, 1],
        windowID=-1,
    ):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
        struct SOut2SimulatorSimple1 {
            int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
            int copterID;  //Vehicle ID
            int vehicleType;  //Vehicle type
            float MotorRPMSMean; // mean motor speed
            float PosE[3];   //NED vehicle position in earth frame (m)
            float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            float Scale[3];
        }  3i10f
        """
        buf = struct.pack(
            "3i10f",
            1234567890,
            copterID,
            vehicleType,
            MotorRPMSMean,
            PosE[0],
            PosE[1],
            PosE[2],
            AngEuler[0],
            AngEuler[1],
            AngEuler[2],
            Scale[0],
            Scale[1],
            Scale[2],
        )
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send

    def sendUE4PosScale2Ground(
        self,
        copterID=1,
        vehicleType=3,
        MotorRPMSMean=0,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        Scale=[1, 1, 1],
        windowID=-1,
    ):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
        checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
        struct SOut2SimulatorSimple1 {
            int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
            int copterID;  //Vehicle ID
            int vehicleType;  //Vehicle type
            float MotorRPMSMean; // mean motor speed
            float PosE[3];   //NED vehicle position in earth frame (m)
            float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            float Scale[3];
        }  3i10f
        """
        buf = struct.pack(
            "3i10f",
            1234567891,
            copterID,
            vehicleType,
            MotorRPMSMean,
            PosE[0],
            PosE[1],
            PosE[2],
            AngEuler[0],
            AngEuler[1],
            AngEuler[2],
            Scale[0],
            Scale[1],
            Scale[2],
        )
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4PosFull(
        self, copterID, vehicleType, MotorRPMS, VelE, PosE, RateB, AngEuler, windowID=-1
    ):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        # struct SOut2Simulator {
        #     int copterID;  //Vehicle ID
        #     int vehicleType;  //Vehicle type
        #     double runnedTime; //Current Time stamp (s)
        #     float VelE[3];   //NED vehicle velocity in earth frame (m/s)
        #     float PosE[3];   //NED vehicle position in earth frame (m)
        #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
        #     float AngQuatern[4]; //Vehicle attitude in Quaternion
        #     float MotorRPMS[8];  //Motor rotation speed (RPM)
        #     float AccB[3];       //Vehicle acceleration in body frame x y z (m/s/s)
        #     float RateB[3];      //Vehicle angular speed in body frame x y z (rad/s)
        #     double PosGPS[3];    //vehicle longitude, latitude and altitude (degree,degree,m)

        # }
        # typedef struct _netDataShort {
        #     int tg;
        #     int        len;
        #     char       payload[192];
        # }netDataShort;

        """
        runnedTime = -1
        # VelE=[0,0,0]
        AngQuatern = [0, 0, 0, 0]
        AccB = [0, 0, 0]
        # RateB=[0,0,0]
        PosGPS = PosE
        # buf for SOut2Simulator, len=152
        buf0 = struct.pack(
            "2i1d27f3d",
            copterID,
            vehicleType,
            runnedTime,
            *VelE,
            *PosE,
            *AngEuler,
            *AngQuatern,
            *MotorRPMS,
            *AccB,
            *RateB,
            *PosGPS,
        )
        # buf for remaining 192-152=40bytes of payload[192] of netDataShort
        buf1 = bytes([0] * (192 - len(buf0)))
        # buf for tg and len in netDataShort
        buf2 = struct.pack("2i", 2, len(buf0))
        # buf for netDataShort
        buf = buf2 + buf0 + buf1
        # print(len(buf))
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send
        # print('Message Send')

    def sendUE4ExtAct(
        self,
        copterID=1,
        ActExt=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        windowID=-1,
    ):
        # struct Ue4ExtMsg {
        #     int checksum;//1234567894
        #     int CopterID;
        #     double runnedTime; //Current  stamp (s)
        #     double ExtToUE4[16];
        # }
        # struct.pack 2i1d16f
        runnedTime = time.time() - self.startTime
        checkSum = 1234567894
        buf = struct.pack("2i1d16d", checkSum, copterID, runnedTime, *ActExt)
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send
        # print('Message Send')

    def sendUE4PosSimple(
        self,
        copterID,
        vehicleType,
        PWMs,
        VelE,
        PosE,
        AngEuler,
        runnedTime=-1,
        windowID=-1,
    ):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        # //输出到模拟器的数据
        # struct SOut2SimulatorSimpleTime {
        #     int checkSum;
        #     int copterID;  //Vehicle ID
        #     int vehicleType;  //Vehicle type
        #     float PWMs[8];
        #     float PosE[3];   //NED vehicle position in earth frame (m)
        #     float VelE[3];
        #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
        #     double runnedTime; //Current Time stamp (s)
        # };
        #struct.pack 3i17f1d
        """
        # if runnedTime<0:
        #     runnedTime = time.time()-self.startTime
        checkSum = 1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack(
            "3i17f1d",
            checkSum,
            copterID,
            vehicleType,
            *PWMs,
            *PosE,
            *VelE,
            *AngEuler,
            runnedTime,
        )
        # print(len(buf))
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send
        # print('Message Send')

    def sendUE4PosNew(
        self,
        copterID=1,
        vehicleType=3,
        PosE=[0, 0, 0],
        AngEuler=[0, 0, 0],
        VelE=[0, 0, 0],
        PWMs=[0] * 8,
        runnedTime=-1,
        windowID=-1,
    ):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        # //输出到模拟器的数据
        # struct SOut2SimulatorSimpleTime {
        #     int checkSum; //1234567890
        #     int copterID;  //Vehicle ID
        #     int vehicleType;  //Vehicle type
        #     float PWMs[8];
        #     float VelE[3];
        #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
        #     double PosE[3];   //NED vehicle position in earth frame (m)
        #     double runnedTime; //Current Time stamp (s)
        # };
        #struct.pack 3i14f4d
        """
        # if runnedTime<0:
        #     runnedTime = time.time()-self.startTime
        checkSum = 1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack(
            "3i14f4d",
            checkSum,
            copterID,
            vehicleType,
            *PWMs,
            *VelE,
            *AngEuler,
            *PosE,
            runnedTime,
        )
        # print(len(buf))
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send
        # print('Message Send')

    # //输出到CopterSim DLL模型的SILints和SILFloats数据
    # struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    # };
    # struct.pack 10i20f

    def sendUE4PosScale100(
        self,
        copterID,
        vehicleType,
        PosE,
        AngEuler,
        MotorRPMSMean,
        Scale,
        isFitGround=False,
        windowID=-1,
    ):
        """send the position & angle information to RflySim3D to create 100 vehicles once
        #struct Multi3DData100New {
        #    int checksum;
        #    uint16 copterID[100];
        #    uint16 vehicleType[100];
        #    float PosE[300];
        #    float AngEuler[300];
        #    uint16 Scale[300];
        #    float MotorRPMSMean[100];
        #}
        #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
        #copterID是100维整型数组，飞机ID，这个从1给到100即可,如果ID给0则不显示次飞机
        #vehicleType是100维整型数组，飞机类型，四旋翼给3即可
        #MotorRPMSMean是100维数组，飞机螺旋桨转速，单位RPM，四旋翼默认给1000即可
        #PosE是300维数组，飞机位置，可以随机生成，单位是m
        #AngEuler是300维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
        #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
        """

        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack(
            "i200H600f300H100f",
            checksum,
            *copterID,
            *vehicleType,
            *PosE,
            *AngEuler,
            *Scale,
            *MotorRPMSMean,
        )
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send

    def sendUE4PosScalePwm20(
        self,
        copterID,
        vehicleType,
        PosE,
        AngEuler,
        Scale,
        PWMs,
        isFitGround=False,
        windowID=-1,
    ):
        """send the position & angle information to RflySim3D to create 20 vehicles once
        # struct Multi3DData2New {
        #   int checksum;
        # 	uint16 copterID[20];
        # 	uint16 vehicleType[20];
        # 	float PosE[60];
        # 	float AngEuler[60];
        # 	uint16 Scale[60];
        # 	float PWMs[160];
        # }
        #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
        #copterID是20维整型数组，飞机ID，这个从1给到20即可,如果ID给0则不显示次飞机
        #vehicleType是20维整型数组，飞机类型，四旋翼给3即可
        #PosE是60维数组，飞机位置，可以随机生成，单位是m
        #AngEuler是60维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
        #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
        #PWMs是160维数组，对应20个飞机各8个旋翼转速，单位RPM，四旋翼默认给1000即可
        """
        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack(
            "i40H120f60H160f",
            checksum,
            *copterID,
            *vehicleType,
            *PosE,
            *AngEuler,
            *Scale,
            *PWMs,
        )
        if windowID < 0:
            for i in range(3):  # 假设最多开了三个窗口
                self.udp_socket.sendto(buf, (self.ip, 20010 + i))

            # if self.ip == "127.0.0.1":
            #     for i in range(6):
            #         self.udp_socket.sendto(buf, (self.ip, 20010 + i))
            # else:
            #     self.udp_socket.sendto(
            #         buf, ("224.0.0.10", 20009)
            #     )  # multicast address, send to all RflySim3Ds on all PC in LAN
        else:
            # if self.ip != "127.0.0.1" and self.ip != "255.255.255.255":
            #     self.udp_socket.sendto(
            #         buf, ("127.0.0.1", 20010 + windowID)
            #     )  # ensure this PC can reciver message under specify IP mode
            self.udp_socket.sendto(
                buf, (self.ip, 20010 + windowID)
            )  # specify PC's IP to send

    def getUE4Pos(self, CopterID=1):
        if self.stopFlagUE4:  # 如果没有启用监听程序
            self.initUE4MsgRec()
            time.sleep(1)

        for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
            if self.inReqVect[i].copterID == CopterID:
                posE = self.inReqVect[i].PosE
                return [posE[0], posE[1], posE[2], 1]
        return [0, 0, 0, 0]

    def getUE4Data(self, CopterID=1):
        if self.stopFlagUE4:  # 如果没有启用监听程序
            self.initUE4MsgRec()
            time.sleep(1)

        for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
            if self.inReqVect[i].copterID == CopterID:
                return self.inReqVect[i]
        return 0

    def initUE4MsgRec(self):
        """Initialize the UDP data linsening from UE4,
        currently, the crash data is listened
        """
        self.stopFlagUE4 = False
        # print("InitUE4MsgRec", self.stopFlagUE4)
        self.inSilVect = []
        self.inReqVect = []
        self.inReqUpdateVect = []
        MYPORT = 20006
        MYGROUP = "224.0.0.10"
        ANY = "0.0.0.0"
        # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.udp_socketUE4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socketUE4.bind((ANY, MYPORT))
        try:
            status = self.udp_socketUE4.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton(MYGROUP) + socket.inet_aton(ANY),
            )
        except:
            print('Failed to Init multicast!')
        self.t4 = threading.Thread(target=self.UE4MsgRecLoop, args=())
        self.t4.start()

    def endUE4MsgRec(self):
        """End UE4 message listening"""
        self.stopFlagUE4 = True
        time.sleep(0.5)
        self.t4.join()
        self.udp_socketUE4.close()

    def UE4MsgRecLoop(self):
        """UE4 message listening dead loop"""
        # lastTime = time.time()
        # while True:
        #     if self.stopFlagUE4:
        #         break
        #     lastTime = lastTime + 0.01
        #     sleepTime = lastTime - time.time()
        #     if sleepTime > 0:
        #         time.sleep(sleepTime)
        #     else:
        #         lastTime = time.time()
        #     # print(time.time())

        # struct CopterSimCrash {
        # 	int checksum;
        # 	int CopterID;
        # 	int TargetID;
        # }
        while True:
            if self.stopFlagUE4:
                break

            try:
                buf, addr = self.udp_socketUE4.recvfrom(65500)
                # print('Data Received!')
                if len(buf) == 12:
                    checksum, CopterID, targetID = struct.unpack("iii", buf[0:12])
                    if checksum == 1234567890:
                        if targetID > -0.5 and CopterID == self.CopterID:
                            self.isVehicleCrash = True
                            self.isVehicleCrashID = targetID
                        print(
                            "Vehicle #", CopterID, " Crashed with vehicle #", targetID
                        )

                if len(buf) == 120:
                    iValue = struct.unpack("10i20f", buf[0:120])
                    if iValue[0] == 1234567897:
                        isCopterExist = False
                        for i in range(len(self.inSilVect)):  # 遍历数据列表，飞机ID有没有出现过
                            if self.inSilVect[i].CopterID == iValue[1]:  # 如果出现过，就直接更新数据
                                isCopterExist = True
                                self.inSilVect[i].checksum = iValue[0]
                                self.inSilVect[i].inSILInts = iValue[2:10]
                                self.inSilVect[i].inSILFLoats = iValue[10:30]
                                # break
                        if not isCopterExist:  # 如果没有出现过，就创建一个结构体
                            vsr = PX4SILIntFloat(iValue)
                            self.inSilVect = self.inSilVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                if len(buf) == 160:
                    isCopterExist = False
                    iValue = struct.unpack("4i1d29f20s", buf[0:160])
                    if iValue[0] == 1234567897:
                        # print(vsr.copterID,vsr.vehicleType)
                        for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
                            if self.inReqVect[i].copterID == iValue[1]:  # 如果出现过，就直接更新数据
                                isCopterExist = True
                                self.inReqVect[i].CopyData(
                                    iValue
                                )  # =copy.deepcopy(vsr)
                                self.inReqUpdateVect[i] = True
                                break
                                # break
                        if not isCopterExist:  # 如果没有出现过，就创建一个结构体
                            vsr = reqVeCrashData(iValue)
                            self.inReqVect = self.inReqVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素
                            self.inReqUpdateVect = self.inReqUpdateVect + [True]

                # 旧版协议，无四元数输出
                if len(buf) == 56:  # CameraData: #长度56, 5i7f1d
                    # print('hello')
                    iValue = struct.unpack("5i7f1d", buf[0:56])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按SeqID来构建相机列表
                        for i in range(len(self.CamDataVect)):  # 遍历数据列表，相机ID有没有出现过
                            if self.CamDataVect[i].SeqID == iValue[1]:  # 如果出现过，就直接更新数据
                                self.CamDataVect[i].CopyDataOld(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = CameraData()
                            vsr.CopyDataOld(iValue)
                            self.CamDataVect = self.CamDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                if len(buf) == 72:  # CameraData: #长度72, 5i11f1d
                    # print('hello')
                    iValue = struct.unpack("5i11f1d", buf[0:72])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按SeqID来构建相机列表
                        for i in range(len(self.CamDataVect)):  # 遍历数据列表，相机ID有没有出现过
                            if self.CamDataVect[i].SeqID == iValue[1]:  # 如果出现过，就直接更新数据
                                self.CamDataVect[i].CopyData(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = CameraData(iValue)
                            self.CamDataVect = self.CamDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                # 旧版协议，无四元数输出
                if len(buf) == 64:  # CoptReqData: #长度64, 2i12f1d
                    iValue = struct.unpack("2i12f1d", buf[0:64])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按CopterID来构建物体飞机
                        for i in range(len(self.CoptDataVect)):  # 遍历数据列表，飞机ID有没有出现过
                            if (
                                self.CoptDataVect[i].CopterID == iValue[1]
                            ):  # 如果出现过，就直接更新数据
                                self.CoptDataVect[i].CopyDataOld(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = CoptReqData()
                            vsr.CopyDataOld(iValue)
                            self.CoptDataVect = self.CoptDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                if len(buf) == 80:  # CoptReqData: #长度80, 2i16f1d
                    iValue = struct.unpack("2i16f1d", buf[0:80])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按CopterID来构建物体飞机
                        for i in range(len(self.CoptDataVect)):  # 遍历数据列表，飞机ID有没有出现过
                            if (
                                self.CoptDataVect[i].CopterID == iValue[1]
                            ):  # 如果出现过，就直接更新数据
                                self.CoptDataVect[i].CopyData(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = CoptReqData(iValue)
                            self.CoptDataVect = self.CoptDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                # 旧版协议，无四元数输出
                if len(buf) == 96:  # ObjReqData: #长度96, 2i12f1d32s
                    iValue = struct.unpack("2i12f1d32s", buf[0:96])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按reqID来构建物体列表
                        for i in range(len(self.ObjDataVect)):  # 遍历数据列表，物体ID有没有出现过
                            if self.ObjDataVect[i].seqID == iValue[1]:  # 如果出现过，就直接更新数据
                                self.ObjDataVect[i].CopyDataOld(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = ObjReqData()
                            vsr.CopyDataOld(iValue)
                            self.ObjDataVect = self.ObjDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                if len(buf) == 112:  # ObjReqData: #长度112, 2i16f1d32s
                    iValue = struct.unpack("2i16f1d32s", buf[0:112])
                    if iValue[0] == 1234567891:
                        isDataExist = False
                        # 按reqID来构建物体列表
                        for i in range(len(self.ObjDataVect)):  # 遍历数据列表，物体ID有没有出现过
                            if self.ObjDataVect[i].seqID == iValue[1]:  # 如果出现过，就直接更新数据
                                self.ObjDataVect[i].CopyData(iValue)
                                isDataExist = True
                                break
                        if not isDataExist:  # 如果没有出现过，就创建一个结构体
                            vsr = ObjReqData(iValue)
                            self.ObjDataVect = self.ObjDataVect + [
                                copy.deepcopy(vsr)
                            ]  # 扩充列表，增加一个元素

                self.trueMsgEvent.set()

            except:
                self.stopFlagUE4 = True
                print("Error Data")
                break

    def getCamCoptObj(self, type_id=1, objName=1):
        # type=0表示相机，1表示手动创建带copter_id的目标，2表示场景中自带的物体
        # 相机时，objName对应相机seqID；指定copterid目标时，objName对应CopterID，请求场景中自带的物体时，objName对应物体名字(字符串)

        if (
            type_id == 1
            and type(objName) == str
            or type_id == 2
            and type(objName) == int
        ):
            print("get type_id not macthing objName")
            return 0

        if type_id == 0:
            for i in range(len(self.CamDataVect)):  # 遍历数据列表，相机ID有没有出现过
                if self.CamDataVect[i].SeqID == objName:
                    return self.CamDataVect[i]

        if type_id == 1:
            for i in range(len(self.CoptDataVect)):  # 遍历数据列表，飞机ID有没有出现过
                if self.CoptDataVect[i].CopterID == objName:
                    return self.CoptDataVect[i]

        if type_id == 2:
            for i in range(len(self.ObjDataVect)):  # 遍历数据列表，物体ID有没有出现过
                if self.ObjDataVect[i].ObjName == objName:
                    return self.ObjDataVect[i]
        return 0

    def reqCamCoptObj(self, type_id=1, objName=1, windowID=0):
        # type=0表示相机，1表示手动创建带copter_id的目标，2表示场景中自带的物体
        # 相机时，objName对应相机seqID；指定copterid目标时，objName对应CopterID，请求场景中自带的物体时，objName对应物体名字(字符串)
        # windowID 表示想往哪个RflySim3D发送消息，默认是0号窗口

        # RflyReqObjData(int opFlag, FString objName, FString colorflag)函数命令
        # opFlag 0创建相机，1创建copter_id对应的目标，2创建物体

        # 为防止误操作，这里加上一个保护,这里在UE端判断最好，UE端
        if type_id == 0:  # 如果请求相机需要确保相机存在,在UE端处理
            pass

        if isinstance(objName, list):
            for i in range(len(objName)):
                val = objName[i]
                if (
                    type_id == 2
                    and type(val) != str
                    or type_id == 1
                    and type(val) != int
                ):
                    print(
                        "current request type_id({0}) not matching objname type({1})".format(
                            type_id, val
                        )
                    )
                    continue
                cmd = "RflyReqObjData %d %s %d" % (type_id, str(val), 0)
                self.sendUE4Cmd(cmd.encode(), windowID)
                time.sleep(0.1)
        else:
            if (
                type_id == 2
                and type(objName) != str
                or type_id == 1
                and type(objName) != int
            ):
                print(
                    "current request type_id({0}) not matching objname type({1})".format(
                        type_id, val
                    )
                )
                return
            cmd = "RflyReqObjData %d %s %d" % (type_id, str(objName), 0)
            print(cmd)
            self.sendUE4Cmd(cmd.encode(), windowID)

    class Radiation:
        def __init__(self, ue: any):
            # self.tempEmissivity = np.array([['Floor',298,0.96,True],
            #                             ['zebra',307,0.98,True],
            #                             ['rhinoceros',299,0.96,True],
            #                             ['hippopotamus',298,0.96,True],
            #                             ['crocodile',303,0.96,True],
            #                             ['human',301,0.985,True],
            #                             ['tree',293,0.952,True],
            #                             ['grass',293,0.958,True],
            #                             ['soil',288,0.914,True],
            #                             ['shrub',293,0.986,True],
            #                             ['truck',293,0.8,True],
            #                             ['water',293,0.96,True]])
            # 这里需要预先给定场景中1：所有物体的名字（或名字中的某一段）、2：温度(开尔文)、
            # 3：发射率（与理想黑体）、4：是否是Actor的名字（如果是Actor的名字，那么它所有的Mesh的辐射值都会被设置，否则会设置一个Mesh）
            # 每个地图都会不一样
            # K = C + 273.15
            self.ue = ue
            self.tempEmissivity = np.array(
                [
                    ["Floor", 276, 0.96, True],
                    ["SK_Man_Arms", 309, 0.98, False],
                    ["SK_Man_Head", 309, 0.98, False],
                    ["SK_Man_Bag", 278, 0.98, False],
                    ["SK_Man_Cloth_Face", 300, 0.98, False],
                    ["SK_Man_Jacket", 283, 0.98, False],
                    ["SK_Man_Pants", 285, 0.98, False],
                    ["SK_Man_Shoes", 285, 0.98, False],
                    ["Copter", 310, 0.98, True],
                    ["Cube", 273, 0.98, True],
                    ["tree", 293, 0.952, True],
                    ["House", 295, 0.85, True],
                    ["BodyMesh_1", 297, 0.98, False],
                    ["Rotor", 285, 0.95, False],
                    ["Road", 293, 0.90, True],
                    ["Birch", 293, 0.952, True],
                    ["Door", 276, 0.96, False],
                    ["Chimney", 310, 0.98, True],
                    ["Wall", 293, 0.952, True],
                    ["Hedge", 293, 0.952, True],
                    ["Fence", 295, 0.85, True],
                    ["Fir", 293, 0.952, True],
                ]
            )

        def sendUE4SetStencilValueByActorName(
            self, Actorname, StencilValue, is_name_regex=False, windowID=-1
        ):
            cmd = f"RflySetStencilValueByActorName {Actorname} {StencilValue} {is_name_regex}"
            self.ue.sendUE4Cmd(cmd.encode(), windowID)
            time.sleep(0.1)

        def sendUE4SetStencilValueByMeshComponentName(
            self, Meshname, StencilValue, is_name_regex=False, windowID=-1
        ):
            cmd = f"RflySetStencilValueByMeshComponentName {Meshname} {StencilValue} {is_name_regex}"
            self.ue.sendUE4Cmd(cmd.encode(), windowID)
            time.sleep(0.1)

        def sendUE4SetStencilValueByCopterID(self, CopterID, StencilValue, windowID=-1):
            cmd = f"RflySetStencilValueByCopterID {CopterID} {StencilValue}"
            self.ue.sendUE4Cmd(cmd.encode(), windowID)
            time.sleep(0.1)

        # 根据温度与辐射率，计算普朗克公式的积分
        def radiance(self, absoluteTemperature, emissivity, dx=0.01, response=None):
            wavelength = np.arange(8, 14, dx)
            c1 = 1.19104e8  # (2 * 6.62607*10^-34 [Js] *
            # (2.99792458 * 10^14 [micron/s])^2 * 10^12 to convert
            # denominator from microns^3 to microns * m^2)       乘pi结果也不会改变，因为最后还是要归一化，乘了等于没乘
            c2 = 1.43879e4  # (hc/k) [micron * K]
            if response is not None:
                radiance = (
                    response
                    * emissivity
                    * (
                        c1
                        / (
                            (wavelength**5)
                            * (np.exp(c2 / (wavelength * absoluteTemperature)) - 1)
                        )
                    )
                )
            else:
                radiance = emissivity * (
                    c1
                    / (
                        (wavelength**5)
                        * (np.exp(c2 / (wavelength * absoluteTemperature)) - 1)
                    )
                )
            if absoluteTemperature.ndim > 1:
                return radiance, np.trapz(radiance, dx=dx, axis=1)
            else:
                return radiance, np.trapz(radiance, dx=dx)

        # 调用radiance函数，获得
        def get_new_temp_emiss_from_radiance(self, tempEmissivity, response):
            numObjects = tempEmissivity.shape[0]

            L = self.radiance(
                tempEmissivity[:, 1].reshape((-1, 1)).astype(np.float64),
                tempEmissivity[:, 2].reshape((-1, 1)).astype(np.float64),
                response=response,
            )[1].flatten()
            L = ((L / L.max()) * 255).astype(np.uint8)

            tempEmissivityNew = np.hstack(
                (
                    tempEmissivity[:, 0].reshape((numObjects, 1)),
                    L.reshape((numObjects, 1)),
                )
            )
            return tempEmissivityNew

    # 利用Radiation类，计算得到目标的辐射值，然后设置给UE4（只要名字中某段能匹配就会设置）
    def SetUE4RadianceValue(self, UEObjectName, windows=-1):
        response = None
        self.radiation = self.Radiation(self)
        tempEmissivityNew = radiation.get_new_temp_emiss_from_radiance(
            radiation.tempEmissivity, response
        )
        index = np.where(radiation.tempEmissivity[:, 0] == UEObjectName)
        key = tempEmissivityNew[index][0]
        if tempEmissivityNew[index][0][3] == "True":
            radiation.sendUE4SetStencilValueByActorName(
                "[\\w]*" + key + "[\\w]*", tempEmissivityNew[index][1], True, windows
            )
        else:
            radiation.sendUE4SetStencilValueByMeshComponentName(
                "[\\w]*" + key + "[\\w]*", tempEmissivityNew[index][1], True, windows
            )

    # 与上面的函数一样，但它设置 Radiation.tempEmissivity 中所有物体的辐射值
    def SetUE4RadianceValue__All(self, windows=-1):
        response = None
        radiation = self.Radiation(self)
        tempEmissivityNew = radiation.get_new_temp_emiss_from_radiance(
            radiation.tempEmissivity, response
        )
        print(tempEmissivityNew)
        # index = np.where(radiation.tempEmissivity[:, 0] == UEObjectName)
        for index, element in enumerate(tempEmissivityNew):
            key = tempEmissivityNew[index][0]
            print(f"key:{key},{radiation.tempEmissivity[index][3]}")
            if radiation.tempEmissivity[index][3] == "True":
                radiation.sendUE4SetStencilValueByActorName(
                    "[\\w]*" + key + "[\\w]*",
                    tempEmissivityNew[index][1],
                    True,
                    windows,
                )
                print("Actor!!")
            else:
                radiation.sendUE4SetStencilValueByMeshComponentName(
                    "[\\w]*" + key + "[\\w]*",
                    tempEmissivityNew[index][1],
                    True,
                    windows,
                )
                print("Mesh!!!")
    
    def sendUE4SetStencilValueByActorName(
        self, Actorname, StencilValue, is_name_regex=False, windowID=-1
    ):
        cmd = f"RflySetStencilValueByActorName {Actorname} {StencilValue} {is_name_regex}"
        self.sendUE4Cmd(cmd.encode(), windowID)
        time.sleep(0.1)

    def sendUE4SetStencilValueByMeshComponentName(
        self, Meshname, StencilValue, is_name_regex=False, windowID=-1
    ):
        cmd = f"RflySetStencilValueByMeshComponentName {Meshname} {StencilValue} {is_name_regex}"
        self.sendUE4Cmd(cmd.encode(), windowID)
        time.sleep(0.1)
        
    def sendUE4SetStencilValueByCopterID(self, CopterID, StencilValue, windowID=-1):
        cmd = f"RflySetStencilValueByCopterID {CopterID} {StencilValue}"
        self.sendUE4Cmd(cmd.encode(), windowID)
        time.sleep(0.1)