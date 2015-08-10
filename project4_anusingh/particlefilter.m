
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% MAIN FILE.
% n = test file number, (1,2,3)

n=2;
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

% to get the robot pose and heading angle at evry instant of time using raw
% data
[xRobot, yRobot , theta_robot, stdTheta]  = MAPS(encoders.Encoders, imu);



range = hokuyo.Hokuyo0.ranges;
angles = hokuyo.Hokuyo0.angles;
H_ts = hokuyo.Hokuyo0.ts;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OGM
map = ones(1000,1000,'int8');
minX = (min(xRobot)-80);
maxX = (max(xRobot)+80);
minY = (min(yRobot)-80);
maxY = (max(yRobot)+80);


%for first pose
indices =  find(range(:,1)>0.05 & range(:,1)<5);
    thetaE = theta_robot(1) + angles(indices);
    xgrid = round((xRobot(1)) + range(indices,1).*cos(thetaE)*10);
    ygrid = round((yRobot(1)) + range(indices,1).*sin(thetaE)*10);
    
    index = sub2ind(size(map), ygrid, xgrid);
    map(index) = map(index) + 1;

newX = xRobot(1);
newY = yRobot(1);
new_theta = theta_robot(1);
 X = [];
 deltax = 0;
 deltay = 0;
 deltaTheta = 0;
 newX = repmat(newX,200,1);
 newY = repmat(newY,200,1);
 new_theta = repmat(new_theta,200,1);
  
for i= 2:numel(encoders.Encoders.ts)-50
   
    if mod(i,60) == 0
          imagesc(map); colormap hot; colorbar;
          drawprt(map, newX, newY );
          drawnow;
    end
  w = zeros(1,200); 
     
    %MOTION MODEL
    [xR, xF, yF, yR] =  getposeupdate (encoders.Encoders.counts(:,i), theta_gyro(i));
    xR = xR + newX;
    yR = yR + newY;
    yF = yF + newY;
    xF = xF + newX;
    
    new_X = (xF + xR) /2;
    new_Y = (yR + yF) / 2;
         
    
    % calculating deltas 
    deltax = abs(newX - new_X);
    deltay = abs(newY - new_Y);
    deltaTheta = abs( new_theta - theta_gyro(i) ); 
    
    %sampling
    [x , y, theta] = sampling(new_X, new_Y, theta_gyro(i),deltax, deltay, deltaTheta,stdTheta);
   for j=1:200
    
    indices =  find(range(:,i)>0.05 & range(:,i)<5);
    thetaES = theta(j) + angles(indices);
    xgridS = round(x(j) + range(indices,i).*cos(thetaES)*10);
    ygridS = round(y(j) + range(indices,i).*sin(thetaES)*10);
  
   indGood = (ygridS > 1) & (xgridS > 1) & (xgridS < 1000) & (ygridS < 1000);
   index = sub2ind(size(map), ygridS(indGood), xgridS(indGood));
   w(j) = sum(map(index));

   end
   %UPDATE
      ind = find(max(w));
    %chopse the jth particle as the best fit and update map
     
      new_x = x(ind) ; 
      new_y = y(ind) ; 
      new_theta = theta(ind) ; 
      
    indices =  find(range(:,i)>0.05 & range(:,i)<5);
    thetaE = new_theta + angles(indices);
    xgrid = round(new_x + range(indices,i).*cos(thetaE)*10);
    ygrid = round(new_y + range(indices,i).*sin(thetaE)*10);
    
    % MAP UPDATE
    indGood = (ygrid > 1) & (xgrid > 1) & (xgrid < 1000) & (ygridS < 1000);
    index = sub2ind(size(map), ygrid(indGood), xgrid(indGood));
    map(index) = map(index) + 1;
 
 
   %resampling
   IND = resample (w,200);
   newX = x(IND);
   newY = y(IND);
   new_theta = theta(IND); 
   
end

figure,
imshow(map(minY:maxY,minX:maxX));  
    