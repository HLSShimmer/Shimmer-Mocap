function trigger_mocap = func2(quat,trigger_shimmer)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  transfer motion kinematics from shimmer to mocap  %%%
%%%                 quaternion method                  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% quat                input         quatenion,from shimmer global system to mocap global system,4x1
% trigger_shimmer     input         trigger motion kinematics(accel, gyro, velocity or position) in shimmer system, nx3
% trigger_mocap       output        trigger motion kinematics(accel, gyro, velocity or position) in mocap system, nx3
%% declare some values
dataLength = size(trigger_shimmer,1);
trigger_mocap = zeros(dataLength,4);
%% transfer
for i=1:dataLength
    trigger_mocap(i,1:3) = CoordinateTransfer(trigger_shimmer(i,:).',quat,'b2r').';
end
trigger_mocap(:,4) = norm(quat);            %%make sure the norm of quat is 1