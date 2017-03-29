function trigger_mocap = func3(theta,trigger_shimmer)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  transfer motion kinematics from shimmer to mocap  %%%
%%%               transformation matrix                %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% quat                input         rotate angle£¬from shimmer to mocap, XOY plane, 3x1
% trigger_shimmer     input         trigger motion kinematics(accel, gyro, velocity or position) in shimmer system, nx3
% trigger_mocap       output        trigger motion kinematics(accel, gyro, velocity or position) in mocap system, nx3
%% declare some values
dataLength = size(trigger_shimmer,1);
trigger_mocap = zeros(dataLength,2);
%% transfer
for i=1:dataLength
    trigger_mocap(i,1) = cos(theta)*trigger_shimmer(i,1) - sin(theta)*trigger_shimmer(i,2);
    trigger_mocap(i,2) = sin(theta)*trigger_shimmer(i,1) + cos(theta)*trigger_shimmer(i,2);
end