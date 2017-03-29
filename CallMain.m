clear all;clc;
count = 1;
% fmin_veloccity = 0.04:0.02:0.3;
fmin_position = 0.04:0.02:0.3;
% velocityResult = cell(length(fmin_veloccity),1);
positionResult = cell(length(fmin_position),1);
save CallMainTempParas count fmin_position positionResult
while count<=length(fmin_position)
    tic
    load CallMainTempParas count fmin_position positionResult
    main
    positionResultStruct.motionPositionSeries_mocap = motionPositionSeries_mocap;
    positionResult{count,1} = positionResultStruct;
    count = count + 1;
    save CallMainTempParas count fmin_position positionResult ankle_position_mocap
    toc
end

%% 
clear all;
load CallMainTempParas_position2 fmin_position positionResult ankle_position_mocap
m = [];
for i=1:2:length(fmin_position)
    m = [m,positionResult{i}.motionPositionSeries_mocap(:,3)];
end
figure(1)
plot(m(100:end,:))
hold on
plot(ankle_position_mocap(1030:end,3),'-.')

%% 
clear all;
load CallMainTempParas_velocity2 fmin_velocity velocityResult ankle_velocity_mocap_filtered
m = [];
for i=1:2:length(fmin_velocity)
    m = [m,velocityResult{i}.motionVelocitySeries_mocap(:,3)];
end
figure(1)
subplot(121)
plot(m(100:end,1:5))
hold on
plot(ankle_velocity_mocap_filtered(1030:end,3),'-.')
subplot(122)
plot(m(100:end,1:5))
hold on
plot(ankle_velocity_mocap_filtered(1030:end,3),'-.')