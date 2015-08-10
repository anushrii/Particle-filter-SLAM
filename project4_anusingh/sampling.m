function [xnew , ynew, theta_new] = sampling(x, y, theta, deltax , deltay, deltaTheta,stdTheta)

%function for creating the random noise in gyro and encoder readings

encoderNoiseX = deltax.*unifrnd(-2,2,[200,1]);
encoderNoiseY = deltay.*unifrnd(-2,2,[200,1]);
gyroNoise = deltaTheta.*unifrnd(-3.5,3.5,[200,1]);

%gyroNoise = (stdTheta*theta + pi/150)*randn(100,1);
xnew = x + encoderNoiseX;
ynew = y + encoderNoiseY;
theta_new  = theta + gyroNoise;

