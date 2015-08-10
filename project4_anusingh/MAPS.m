
function [xRobot, yRobot, theta, stdTheta ]  =MAPS(Encoders, imu)
E_ts = Encoders.ts - Encoders.ts(1);
IMU_ts = imu.ts;
IMU_ts = IMU_ts - IMU_ts(1);

%%%%Converting the raw IMU gyro readings to yaw angles
%using the code from the UKF project

Euler_W = geteuler(imu.vals);
Yaw_gyro = Euler_W(:,1);
%Yaw_gyro = Euler_W;

%synchronize timestamps of encoder and IMU
[~, ts_E_IMU]= timestamps_sync(E_ts,IMU_ts);

%using first two encoder counts calculate the centre of the front axel
%%encoder
res = 180; %counts per revolution
R = 82.55; %mm
%R = 254/2;

leftWheelFront  = Encoders.counts(2,:);
rightWheelFront = Encoders.counts(1,:);
leftWheelRear  = Encoders.counts(4,:);
rightWheelRear = Encoders.counts(3,:);

xF = zeros(size(E_ts));
yF = zeros(size(E_ts));
xR = zeros(size(E_ts));
yR = zeros(size(E_ts));
 delta_theta = zeros(size(E_ts));
 theta = Yaw_gyro(ts_E_IMU);
 
for i = 1:numel(E_ts)-1
    deltaL = leftWheelFront(i);
    deltaR = rightWheelFront(i);
    %forward motion(displacement)
     Fm = (deltaR + deltaL)/2;
    Fm = Fm*2*pi*R/res;
    
    delta_theta(i) =  theta(i+1) - theta(i);
    
    RdeltaL = leftWheelRear(i);
    RdeltaR = rightWheelRear(i);
  
    %forward motion(displacement)
    RFm = (RdeltaR + RdeltaL)/2;
   RFm = RFm*2*pi*R/res;

    xR(i+1) = xR(i) + RFm*cos(theta(i));
    yR(i+1) = yR(i) + RFm*sin(theta(i));
   
    xF(i+1) = xF(i) + Fm*cos(theta(i));
    yF(i+1) = yF(i) + Fm*sin(theta(i));
    
end

stdTheta = std(delta_theta);



xRobot =  500 + ((xF + xR) / 2)*10^-2;
yRobot = 500 +((yF + yR) / 2)*10^-2;
figure, plot(xRobot, yRobot,'mx')






