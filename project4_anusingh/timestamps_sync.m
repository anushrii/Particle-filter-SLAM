% [ts_i_c, ts_c_i]= timestamps_sync(ts_c,ts_i)
function [ts_IMU_E, ts_E_IMU]= timestamps_sync(E_ts,IMU_ts)
%Encoder time and imu time
for  i= 1:size(E_ts,2)
    [ ~ ,idx] = min(abs(E_ts(i) - IMU_ts));
    ts_IMU_E(idx) = i;
    ts_E_IMU(i) = idx;
end