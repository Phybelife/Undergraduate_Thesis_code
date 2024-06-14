%%%%%%%%%%%%%%%%%%%%%%%%
%��չ�����
clear;
close all;
clc;

%% 
simDT = 0.01 ;        % ɨƵ
Ts = simDT;    
Tt = 100;              
time = (0:Ts:Tt)';    
sampleTime = 0.01;
ModelParam.timeSamp =0.01;
%% ����
saturation_x=2;
saturation_y=2;
saturation_z=2;
saturation_psi=2;

%% ����������
k_p_x = 1.2;
k_d_x = 1.5;
k_i_x = 0.01;%��ʵ�ٶ�PID

k_p_y = 1.5;
k_d_y = 1.5;
k_i_y = 0.01;%��ʵ�ٶ�PID

%% P273�еĿ���������.
k_p_x_change =3;
k_d_x_change = 0.3;
k_p_y_change =3;
k_d_y_change = 0.5;

%% �˲�������
yita_x=0.05;
yita_y=0.15;

%% �߶�ͨ������
k_p_z =2;
k_d_z =0.5; 
k_i_z=0.01;%��ʵ�ٶ�PID
%% ƫ��ͨ������
k_p_yaw =2;
k_d_yaw=1;
k_i_yaw=0.01;%��ʵ�ٶ�PID


limit_cmd_xy = inf;
g = 9.8;
%{
DO NOT CHANGE FROM NOW ON
%}

%% ������ģ�Ͳ���
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


%% ��ʼ������
% =================================================================
%  F450-FlightGear
% =================================================================

%�ɻ���ʼ״̬
ModelInit.PosE=[0,0,0];  %��ʼλ�ã���λ��m��
ModelInit.VelB=[0,0,0];  %��ʼ�ٶȣ���λ��m/s��
ModelInit.AngEuler=[0,0,0];%��ʼ�Ƕȣ���λ��rad��
ModelInit.RateB=[0,0,0];%��ʼ���ٶȣ���λ��rad/s��
ModelInit.RPM=[0 0 0 0];%�����ʼת�٣���λ��rad/s��
% ModelInit.RPM=[548.3,548.3,548.3,548.3];%�����ʼת�٣���λ��rad/s��
%�������
ModelParam.motorMinThr=0;  %�����С��������
ModelParam.motorCr=716.91;    %���ת��-��������б�ʣ���λ��rad/s��
ModelParam.motorWb=158.17;    %���ת��-�������߳�����(��λ��rad/s��
ModelParam.motorT= 0.0261;    %�������ʱ�䳣������λ��s��
ModelParam.motorJm =0.0001287; %���������ת����������λ��kg.m^2��

%����������
%��������Ť��ϵ��Cm����λ��N.m/(rad/s)^2��
%���壺����M����λN.m����������ת��w����λ��rad/s����M=Cm*w^2
ModelParam.rotorCm=2.25e-07;
%����������ϵ��Ct����λ��N/(rad/s)^2��
%���壺����T����λ��N����T=Ct**w^2
ModelParam.rotorCt=1.276e-05;

% ���������
ModelParam.uavType = int8(0); %���� 0��ʾ�����ͣ�1��ʾ+����
ModelParam.uavMotNumbs = int8(4);%�������
ModelParam.ControlMode = int8(1); %���� 1��ʾAutoģʽ��0��ʾManualģʽ
%����ϵ��Cd
%���壺����D��N),ǰ���ٶ�V��m/s����D=Cd*V^2
ModelParam.uavCd =0.1365 ;%����ϵ������λ�� N/(m/s)^2��0.1365
ModelParam.uavMass=1.5; %��������������λ��kg��
ModelParam.uavR=0.225; %��������ܰ뾶����λ��m��
ModelParam.uavJxx =1.491E-2;%x��ת����������λ�� kg.m^2��
ModelParam.uavJyy =1.491E-2;%y��ת����������λ�� kg.m^2��
ModelParam.uavJzz =2.712E-2;%z��ת����������λ�� kg.m^2��

%��������
ModelParam.envGravityAcc = 9.81;     %�������ٶ�
ModelParam.envAirDensity = 1.225;    %�����ܶ�
ModelParam.envLongitude = 8.545592;  %��ʼ�㾭γ��
ModelParam.envLatitude = 47.397742;  %��ʼ��ά��
ModelParam.envAltitude = -50;        %�ο��߶ȣ���ֵΪ��
ModelParam.envDiffPressure = 0;      %��׼��ѹ

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

%IMU�����������ǰ������������Ǿ�����һ��
ModelParam.noisePowerIMU=0;
%GPS��λ�������������������������x,y,z�Ĳ������ޣ���λ��m
ModelParam.noiseUpperGPS=0;  
ModelParam.noiseGPSSampTime=0.5;%Ĭ��0.05

ModelParam.noiseUpperBaro=0; %��ѹ������������������������߶ȵĲ������ޣ���λ��m
ModelParam.noiseBaroSampTime=0.5;%��ѹ����������Ƶ�ʣ���Ĭ��0.05
ModelParam.noiseBaroCoupleWithSpeed=0;%��ѹ�Ʋ����߶��붯ѹ��ϵ��Ҳ���Ƿ�������ѹ�Ƶ���ģ�͵�ϵ������ǰ����0.008�ɻ�10m/s����1m

ModelParam.noiseUpperWindBodyRatio=0;%�粨��ϵ��������*(1+��ϵ��)
ModelParam.noiseWindSampTime=0.01;


ModelParam.envGravityAcc = 9.81;
ModelParam.envAirDensity = 1.225;    
ModelParam.envLongitude = 8.545592;
ModelParam.envLatitude = 47.397742;
ModelParam.envAltitude = -50;     %�ο��߶ȣ���ֵ
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
%�����ƽǶȣ���λ��
MAX_CONTROL_ANGLE_RP = 45;
MAX_CONTROL_ANGLE_Y = 180;
%�����ƽ��ٶȣ���λ��
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