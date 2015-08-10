function [map,w] = pf()
n=24;
[encoders, imu, hokuyo ]  = readfiles(n);

E_ts = encoders.Encoders.ts - encoders.Encoders.ts(1);
IMU_ts = imu.ts;
IMU_ts = IMU_ts - IMU_ts(1);

%%%%Converting the raw IMU gyro readings to yaw angles
%using the code from the UKF project

Euler_W = geteuler(imu.vals);
Yaw_gyro = Euler_W(:,1);
%Yaw_gyro = Euler_W;

%synchronize timestamps of encoder and IMU
[~, ts_E_IMU]= timestamps_sync(E_ts,IMU_ts);
theta_gyro = Yaw_gyro(ts_E_IMU);

[xRobot, yRobot , theta_robot, stdTheta]  = MAPS(encoders.Encoders, imu);


range = hokuyo.Hokuyo0.ranges;
angles = hokuyo.Hokuyo0.angles;
H_ts = hokuyo.Hokuyo0.ts;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
map = ones(1000,1000,'int8');
% minX = (min(xRobot)-80);
% maxX = (max(xRobot)+80);
% minY = (min(yRobot)-80);
% maxY = (max(yRobot)+80);

y_im = (1:size(map,2));
x_im = (1:size(map,2));

%for first pose
indices =  find(range(:,1)>0.05 & range(:,1)<5);
    thetaE = theta_robot(1) + angles(indices);
    xgrid = round((xRobot(1)) + range(indices,1).*cos(thetaE)*10);
    ygrid = round((yRobot(1)) + range(indices,1).*sin(thetaE)*10);
    
    index = sub2ind(size(map), ygrid, xgrid);
    map(index) = map(index) + 1;
%noise model


% encoderNoise = (stdTheta*theta + 0.005)*randn(1,1);
% 
% gyroNoise = (stdTheta*theta + pi/150)*randn(1,1);
%new_pose = cell(1,3);
newX = xRobot(1);
newY = yRobot(1);
%new_theta = theta_robot(1);
 %X = [];
 deltax = 0;
 deltay = 0;
 deltaTheta = 0;
 
  
for i= 2:numel(encoders.Encoders.ts)-50
    i
     if mod(i,30) == 0
          imagesc(map); colormap hot; colorbar;
          drawprt(map, newX, newY );
          drawnow;
     end
%    x_range = -10:0.5:10;
%    y_range = -10:0.5:10;
%    
%   x = zeros(1,50);
%   y = zeros(1,50);
%   theta = zeros(1,50);
  
     
     if i==500
         i
     end
       [x , y, theta] = samplingPF(newX, newY, theta_gyro(i),deltax, deltay, deltaTheta, stdTheta);
   
    w = zeros(1,length(x));
    for j = 1:length(x)
        
    indices =  find(range(:,i)>0.05 & range(:,i)<5);
    thetaES = theta(j) + angles(indices);
    xgridS = round(x(j) + range(indices,i).*cos(thetaES)*10);
    ygridS = round(y(j) + range(indices,i).*sin(thetaES)*10);
    %Y =  [xgridS';ygridS';zeros(size(xgridS))'];
    %c = map_correlation(int8(map),x_im,y_im,Y(1:3,:),x_range,y_range);
    %w(j) = max(max(c(:)));
    
    
    indGood = (ygridS > 1) & (xgridS > 1) & (xgridS < 1000) & (ygridS < 1000);
    index = sub2ind(size(map), ygridS(indGood), xgridS(indGood));
    w(j) = sum(map(index));
    end
 
      ind = find(max(w));
      %chopse the jth particle as the best fit and update map
      
      
      new_x = x(ind) ; 
      new_y = y(ind) ; 
      new_theta = theta(ind) ; 
      
    indices =  find(range(:,i)>0.05 & range(:,i)<5);
    thetaE = new_theta + angles(indices);
    xgrid = round(new_x + range(indices,i).*cos(thetaE)*10);
    ygrid = round(new_y + range(indices,i).*sin(thetaE)*10);
    
    indGood = (ygrid > 1) & (xgrid > 1) & (xgrid < 1000) & (ygridS < 1000);
    index = sub2ind(size(map), ygrid(indGood), xgrid(indGood));
    map(index) = map(index) + 1;

    
    if length(x)>10  
   IND = resample (w,10);
   x = x(IND);
   y = y(IND);
   theta = theta(IND); 
    end
%    newX = zeros(10,1);
%    newY = zeros(10,1);
   
    %for j=1:10
    [xR, xF, yF, yR] =  getposeupdate(encoders.Encoders.counts(:,i), theta_gyro(i));
    xR = xR + x;
    yR = yR + y;
    yF = yF + y;
    xF = xF + x;
    
    newX = (xF + xR) /2;
    newY = (yR + yF) / 2;
         
    %end
    
    deltax = abs(newX - x);
    deltay = abs(newY - y);
    deltaTheta = abs(theta_gyro(i) - theta);   
      
end

    
    