function [encoders, imu, hokuyo ]= readfiles(n)
addpath('C:\Users\sabhajit singh\Desktop\learning in robotics\project 4\Project4\data')
encoders = ['Encoders_test' num2str(n) '.mat'];
imu = ['imuRaw_test' num2str(n) '.mat'];
%kinect = ['kinect' num2str(n) '.mat'];
hokuyo = ['Hokuyo_test' num2str(n) '.mat'];

encoders= load(encoders);
imu = load(imu);
%files.kinect = load(kinect);
hokuyo = load(hokuyo);

%[~, ts_E_IMU]= timestamps_sync(E_ts,IMU_ts);
