
function Euler_W = geteuler(vals)
% for gyro, finding angualar velocity
Wx = vals(4,:);
Wy = vals(5,:);
Wz = vals(6,:);
W = [ Wx; Wy; Wz];

% subtracting bias
bias_Wx = mean(Wx(1:400)); 
bias_Wy = mean(Wy(1:400));
bias_Wz = mean(Wz(1:400));
bias_W = [ bias_Wx; bias_Wy; bias_Wz];

Wr = bsxfun(@minus,W,bias_W);

% scaling it to physical values
sw = 3.3;
Vref = 3300;
sfw = (Vref/1023)/sw;
Wr = Wr*sfw*pi/180;
%%in the g frame first component is sign inverted
W=[Wr(2:3,:); -Wr(1,:)];

delta_t = 1/100;
norm_W  = sqrt(sum(Wr.*Wr));
alpha_delta = norm_W*delta_t;%angle
e_delta = bsxfun(@rdivide,Wr,norm_W);

%corresponding quaternion_delta

quat_delta = [cos(alpha_delta/2) ;bsxfun(@times,e_delta,sin(alpha_delta/2))];
quat_delta = quat_delta';
quat_i = [1 0 0 0 ];
% new state q_k+1 = q_k*delta_q,
% used a loop to calculate the states.
% starting with initial state of quaternion  [1 0 0 0]


 q_out = zeros(size(W,2),4);
 for i=1:size(W,2)
     
     q_out(i,:) = quatmultiply(quat_i,quat_delta(i,:));
     q_out(i,:) = quatnormalize(q_out(i,:));
     quat_i = q_out(i,:);
 end
 Rw =  quat2dcm(q_out);
 
 Euler_W = quat2euler(q_out);