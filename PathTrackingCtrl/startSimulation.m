%%%%%%%%%%%%%%%%%%%%%%%%
%清空工作区
clear;
close all;
clc;

%% 
simDT = 0.01 ;        % 扫频
Ts = simDT;    
Tt = 100;              
time = (0:Ts:Tt)';    
sampleTime = 0.01;
ModelParam.timeSamp =0.01;
%% 饱和
saturation_x=2;
saturation_y=2;
saturation_z=2;
saturation_psi=2;

%% 控制器参数
k_p_x = 1.2;
k_d_x = 1.5;
k_i_x = 0.01;%真实速度PID

k_p_y = 1.5;
k_d_y = 1.5;
k_i_y = 0.01;%真实速度PID

%% P273中的控制器参数.
k_p_x_change =3;
k_d_x_change = 0.3;
k_p_y_change =3;
k_d_y_change = 0.5;

%% 滤波器参数
yita_x=0.05;
yita_y=0.15;

%% 高度通道参数
k_p_z =2;
k_d_z =0.5; 
k_i_z=0.01;%真实速度PID
%% 偏航通道参数
k_p_yaw =2;
k_d_yaw=1;
k_i_yaw=0.01;%真实速度PID


limit_cmd_xy = inf;
g = 9.8;
%{
DO NOT CHANGE FROM NOW ON
%}

%% 非线性模型参数
k_p_x_model = 1.5;
k_d_x_model = 3.0;% 1.5
k_p_y_model = 1.5;
k_d_y_model = 3.0;% 1.5

%% 
k_p_z_model = 1;
k_d_z_model = 0.1; 
%% 
k_p_yaw_model = 1;
%% 
tdelay_x=0.1308;    
tdelay_y=0.1303;
tdelay_h=0.1184;
tdelay_yaw=0.2555;
tdelay_test=0;


%% 初始化参数
% =================================================================
%  F450-FlightGear
% =================================================================

%飞机初始状态
ModelInit.PosE=[0,0,0];  %初始位置（单位：m）
ModelInit.VelB=[0,0,0];  %初始速度（单位：m/s）
ModelInit.AngEuler=[0,0,0];%初始角度（单位：rad）
ModelInit.RateB=[0,0,0];%初始角速度（单位：rad/s）
ModelInit.RPM=[0 0 0 0];%电机初始转速（单位：rad/s）
% ModelInit.RPM=[548.3,548.3,548.3,548.3];%电机初始转速（单位：rad/s）
%电机参数
ModelParam.motorMinThr=0;  %电机最小油门死区
ModelParam.motorCr=716.91;    %电机转速-油门曲线斜率（单位：rad/s）
ModelParam.motorWb=158.17;    %电机转速-油门曲线常数项(单位：rad/s）
ModelParam.motorT= 0.0261;    %电机惯性时间常数（单位：s）
ModelParam.motorJm =0.0001287; %电机螺旋桨转动惯量（单位：kg.m^2）

%螺旋桨参数
%螺旋桨反扭矩系数Cm（单位：N.m/(rad/s)^2）
%定义：力矩M（单位N.m），螺旋桨转速w（单位：rad/s），M=Cm*w^2
ModelParam.rotorCm=2.25e-07;
%螺旋桨拉力系数Ct（单位：N/(rad/s)^2）
%定义：拉力T（单位：N），T=Ct**w^2
ModelParam.rotorCt=1.276e-05;

% 多旋翼参数
ModelParam.uavType = int8(0); %整型 0表示叉字型，1表示+字型
ModelParam.uavMotNumbs = int8(4);%电机数量
ModelParam.ControlMode = int8(1); %整型 1表示Auto模式，0表示Manual模式
%阻力系数Cd
%定义：阻力D（N),前飞速度V（m/s），D=Cd*V^2
ModelParam.uavCd =0.1365 ;%阻力系数（单位： N/(m/s)^2）0.1365
ModelParam.uavMass=1.5; %多旋翼质量（单位：kg）
ModelParam.uavR=0.225; %多旋翼机架半径（单位：m）
ModelParam.uavJxx =1.491E-2;%x轴转动惯量（单位： kg.m^2）
ModelParam.uavJyy =1.491E-2;%y轴转动惯量（单位： kg.m^2）
ModelParam.uavJzz =2.712E-2;%z轴转动惯量（单位： kg.m^2）

%环境参数
ModelParam.envGravityAcc = 9.81;     %重力加速度
ModelParam.envAirDensity = 1.225;    %空气密度
ModelParam.envLongitude = 8.545592;  %初始点经纬度
ModelParam.envLatitude = 47.397742;  %初始点维度
ModelParam.envAltitude = -50;        %参考高度，负值为正
ModelParam.envDiffPressure = 0;      %标准气压

ModelParam.GPSEphTimeConstant=0.2;
ModelParam.GPSEpvTimeConstant=0.2;
ModelParam.GPSEphInitial=20;
ModelParam.GPSEpvInitial=20;
ModelParam.GPSEphFinal=0.3;
ModelParam.GPSEpvFinal=0.4;
ModelParam.GPSEphMin3d=3;
ModelParam.GPSEphMin2d=4;
ModelParam.GPSFix3DFix=3;
ModelParam.GPSFix2DFix=2;
ModelParam.GPSFixNoFix=0;
ModelParam.GPSSatsVisible=10;

%IMU噪声，这里是白噪声，这里是经过归一化
ModelParam.noisePowerIMU=0;
%GPS定位误差噪声，均匀噪声，这里填x,y,z的波动上限，单位是m
ModelParam.noiseUpperGPS=0;  
ModelParam.noiseGPSSampTime=0.5;%默认0.05

ModelParam.noiseUpperBaro=0; %气压计噪声，均匀噪声，这里填高度的波动上限，单位是m
ModelParam.noiseBaroSampTime=0.5;%气压计噪声更新频率，，默认0.05
ModelParam.noiseBaroCoupleWithSpeed=0;%气压计测量高度与动压关系，也就是风速与气压计掉高模型的系数，当前参数0.008飞机10m/s掉高1m

ModelParam.noiseUpperWindBodyRatio=0;%风波动系数，风速*(1+该系数)
ModelParam.noiseWindSampTime=0.01;


ModelParam.envGravityAcc = 9.81;
ModelParam.envAirDensity = 1.225;    
ModelParam.envLongitude = 8.545592;
ModelParam.envLatitude = 47.397742;
ModelParam.envAltitude = -50;     %参考高度，负值
ModelParam.envDiffPressure = 0; 
ModelParam.noiseTs = 0.001;


% Kp_RP_ANGLE =6.5;
% Ki_RP_ANGLE =0.0;
% Kp_RP_AgngleRate = 1.30;
% Ki_RP_AgngleRate = 0.01;
% Kd_RP_AgngleRate = 0.03;
% Kff_RP_AngleRate = 0.0;

Kp_RP_ANGLE =6.5;
Kp_RP_AgngleRate = 0.55;
Ki_RP_AgngleRate = 0.01;
Kd_RP_AgngleRate = 0.005;

Kp_YAW_AngleRate = 3.2;
Ki_YAW_AngleRate = 0.8;
Kd_YAW_AngleRate = 0.05;
%最大控制角度，单位度
MAX_CONTROL_ANGLE_RP = 45;
MAX_CONTROL_ANGLE_Y = 180;
%最大控制角速度，单位度
MAX_CONTROL_ANGLE_RATE_RP = 180;
MAX_CONTROL_ANGLE_RATE_Y = 90;

RC_THR=1600; %1544
RC_PITCH=1600; %1500
RC_ROLL=1500; %1500
RC_YAW=1500; %1500
LED=1000;
X_DESIRED=00;
Y_DESIRED=00;
Z_DESIRED=100;
TF_TrajectoireFollowing_Segment