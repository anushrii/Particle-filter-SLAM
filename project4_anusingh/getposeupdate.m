
function [xR, xF, yF, yR] =  getposeupdate( Encoders, theta)

%%encoder
res = 180; %counts per revolution
R = 82.55; %mm
%R = 254/2;

leftWheelFront  = Encoders(2);
rightWheelFront = Encoders(1);
leftWheelRear  = Encoders(4);
rightWheelRear = Encoders(3);

% xF = zeros(size(E_ts));
% yF = zeros(size(E_ts));
% xR = zeros(size(E_ts));
% yR = zeros(size(E_ts));
 %delta_theta = zeros(size(E_ts));
 %theta = Yaw_gyro(ts_E_IMU);
 

    deltaL = leftWheelFront;
    deltaR = rightWheelFront;
    %forward motion(displacement)
     Fm = (deltaR + deltaL)/2;
    Fm = Fm*2*pi*R/res*10^-2;
    
   % delta_theta(i) =  theta(i+1) - theta(i);
    
    RdeltaL = leftWheelRear;
    RdeltaR = rightWheelRear;
  
    %forward motion(displacement)
    RFm = (RdeltaR + RdeltaL)/2;
   RFm = RFm*2*pi*R/res*10^-2;

%     xR = x + RFm*cos(theta);
%     yR = y + RFm*sin(theta);
%    
%     xF = x + Fm*cos(theta);
%     yF = y + Fm*sin(theta);
    
    xR = RFm*cos(theta);
    yR = RFm*sin(theta);
   
    xF = Fm*cos(theta);
    yF = Fm*sin(theta);


%stdTheta = std(delta_theta);



% xRobot =  ((xF + xR) / 2);
% yRobot = ((yF + yR) / 2);