function [xnew , ynew, theta_new] = samplingPF(x, y, theta, deltax , deltay, deltaTheta)
% sampling for the particle filter named PF.



theta = repmat(theta,length(x),1);

for i=1:length(x)
xnew(:,i) = x(i) + deltax(i)*unifrnd(-3,3,[10,1]);
ynew(:,i) = y(i) + deltay(i)*unifrnd(-3,3,[10,1]);
theta_new(:,i) = theta(i) + deltaTheta(i)*unifrnd(-3,3,[10,1]);

end

xnew = reshape(xnew,[],1);
ynew = reshape(ynew,[],1);
theta_new = reshape(theta_new,[],1);

%pose_sample = [x , y, theta];