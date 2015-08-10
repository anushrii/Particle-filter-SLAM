%close all
%clear all

%%%%making grid map
n=1;
[encoders, imu, hokuyo ]  = readfiles(n);
map = 0.5*ones(1000,1000,'int8');
[xRobot, yRobot , theta_robot]  = MAPS(encoders.Encoders, imu);

%for LIDAR
range = hokuyo.Hokuyo0.ranges;
angles = hokuyo.Hokuyo0.angles;
H_ts = hokuyo.Hokuyo0.ts;
figure,
minX = (min(xRobot)-80);
maxX = (max(xRobot)+80);
minY = (min(yRobot)-80);
maxY = (max(yRobot)+80);

for i= 1:numel(encoders.Encoders.ts)-50
    
    indices =  find(range(:,i)>0.05 & range(:,i)<5);
    thetaE = theta_robot(i) + angles(indices);
    xgrid = round((xRobot(i)) + range(indices,i).*cos(thetaE)*10);
    ygrid = round((yRobot(i)) + range(indices,i).*sin(thetaE)*10);
    
  
    
  index = sub2ind(size(map), ygrid, xgrid);
     map(index) = map(index) + 1;
    
    Y =  [xgrid;ygrid;zeros(size(xgrid))];
   
end

figure,
imshow(map(minY:maxY,minX:maxX));








