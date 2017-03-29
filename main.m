clear all;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Call sensorKinematics.m, process data to obtain some kinematics information %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% load data
load DataBase_KneeFlexion_shimmer ankleMotion ankleStatic triggerBound_shimmer 
sensorMotion = ankleMotion;
sensorStatic = ankleStatic;
load DataBase_KneeFlexion_mocap ankle_position_mocap ankle_acceleration_mocap ankle_velocity_mocap mocap_time triggerBound_mocap
ankle_position_mocap = ankle_position_mocap/1000;
ankle_acceleration_mocap = ankle_acceleration_mocap/1000;
ankle_velocity_mocap = ankle_velocity_mocap/1000;
%% parameters
para.dt = 1/100.51;             %sample step
% TD Filter
para.h1 = 7*para.dt;
para.r = 2000;
% Quaternion Madgwick
para.alpha = 30;
para.beta = 0.35;
% Quaternion Fourati
para.lamda = 2;                 %convergence factor
para.k = 0.3;                   %filter factor
% Frequency Domain Integration
para.fResolution = 0.0003;
% Frequency Filter
para.fmin_velocity = 0.14;                    %cutoff frequency, lower & upper bound
% para.fmin_velocity = fmin_velocity(count);
para.fmax_velocity = 50;
para.fTarget_velocity = 0.09;                 %target frequncy
para.integrateAccuracy_velocity = 0.95;       %integrate accuracy
para.fmin_position = 0.07;                 %cutoff frequency, lower & upper bound 0.1236
% para.fmin_position = fmin_position(count);
para.fmax_position = 100;
para.fTarget_position = 0.09;             %target frequncy
para.integrateAccuracy_position = 0.95;   %integrate accuracy
para.minPeakDistance = 0.002;
para.minPeakHeight = 10;
% Time Domain Integration
para.lamdaV = 0.001;             %threshold of variance
para.lamdaM = 20;               %threshold of interval
para.populationSize = 10;
%% mothed setting
methodSet.dataFilter = 1;         %1-TD
methodSet.quaternion = 1;         %1-Madgwick,2-Fourati
methodSet.accelIntegrate = 2;     %1-TimeDomain;2-FreqencyDomain
methodSet.frequencyFilter = 1;    %1-CutOff;2-Decay
%% data processing, calculate quaternion, motion accel/velocity/displacement
positionInitial = [0;0;0];
velocityInitial = [0;0;0];
[quatSeries,motionAccelSeries,motionVelocitySeries,motionPositionSeries,dataStaticFiltered,dataMotionFiltered] = SensorKinematics(sensorStatic,sensorMotion,positionInitial,velocityInitial,para,methodSet);
%% transfer accel by quaternion given by shimmer
motionAccelInReferenceInTimeSeries_Shimmer = zeros(size(quatSeries,1),3);
for i=1:size(quatSeries,1)
    motionAccelInReferenceInTimeSeries_Shimmer(i,:) = CoordinateTransfer(dataMotionFiltered(i,4:6).',sensorMotion.Quat9DOF_LowNoise(i,:).','b2r').';
%     motionAccelInReferenceInTimeSeries_Shimmer(i,3) = motionAccelInReferenceInTimeSeries_Shimmer(i,3) - 9.8;
end
%% get filtered accel obtained from mocap
ankle_acceleration_mocap(1:2,:) = 0;
ankle_velocity_mocap(1,:) = 0;
para.h1 = 7*para.dt;
para.r = 1500;
ankle_acceleration_mocap_filtered = FilterData(ankle_acceleration_mocap,para.dt,methodSet.dataFilter,para);
ankle_velocity_mocap_filtered = FilterData(ankle_velocity_mocap,para.dt,methodSet.dataFilter,para);
%%
figure(1)
plot(ankle_acceleration_mocap_filtered(triggerBound_mocap(1):end,3))
hold on
plot(motionAccelSeries(triggerBound_shimmer(1):end,3));
figure(2)
plot(motionPositionSeries(:,1))
%% get trigger motion displacement, and calculate the rotate angle or quaternion from shimmer system to mocap system
triggerAccel_shimmer = motionAccelSeries(triggerBound_shimmer(1):triggerBound_shimmer(2),:);
triggerAccel_mocap = ankle_acceleration_mocap_filtered(triggerBound_mocap(1):triggerBound_mocap(2),:);
a(:,1) = sqrt(sum(triggerAccel_shimmer.^2,2));
a(:,2) = sqrt(sum(triggerAccel_mocap.^2,2));
plot(a,'DisplayName','a')
b1 = sqrt(sum(motionAccelSeries.^2,2));
b2 = sqrt(sum(ankle_acceleration_mocap_filtered.^2,2));
figure(3)
plot(b1(triggerBound_shimmer(1):end),'DisplayName','b1')
hold on
plot(b2(triggerBound_mocap(1):end),'DisplayName','b2')
staticPosition_mocap = mean(ankle_position_mocap(1:triggerBound_mocap(1)-1,:),1);
% quatFromShimmer2Mocap_global = GlobalSystemAlignment(triggerAccel_shimmer,triggerAccel_mocap,staticPosition_mocap);
thetaFromShimmer2Mocap_global = GlobalSystemAlignment2(triggerAccel_shimmer,triggerAccel_mocap,staticPosition_mocap);
%% transfer global coordinates from shimmer to mocap
% [motionAccelSeries_mocap,motionVelocitySeries_mocap,motionPositionSeries_mocap] = TransferGlobalSystemFromShimmer2Mocap(quatFromShimmer2Mocap_global,motionAccelSeries,motionVelocitySeries,motionPositionSeries,staticPosition_mocap);
[motionAccelSeries_mocap,motionVelocitySeries_mocap,motionPositionSeries_mocap] = TransferGlobalSystemFromShimmer2Mocap2(thetaFromShimmer2Mocap_global,motionAccelSeries,motionVelocitySeries,motionPositionSeries,staticPosition_mocap);
%% plot
timeShimmer = 0:para.dt:((size(motionAccelSeries_mocap,1)-triggerBound_shimmer(1)+1)-1)*para.dt;
timeMocap = 0:0.01:((size(ankle_acceleration_mocap_filtered,1)-triggerBound_mocap(1)+1)-1)*0.01;
figure(4)
for i=1:3
    subplot(3,1,i)
    plot(timeShimmer,motionAccelSeries_mocap(triggerBound_shimmer(1):end,i))
    hold on
    plot(timeMocap,ankle_acceleration_mocap_filtered(triggerBound_mocap(1):end,i))
end
figure(5)
for i=1:3
    subplot(3,1,i)
    plot(timeShimmer,motionVelocitySeries_mocap(triggerBound_shimmer(1):end,i))
    hold on
    plot(timeMocap,ankle_velocity_mocap_filtered(triggerBound_mocap(1):end,i))
end
figure(6)
for i=1:3
    subplot(3,1,i) 
    plot(timeShimmer,motionPositionSeries_mocap(triggerBound_shimmer(1):end,i))
    hold on
    plot(timeMocap,ankle_position_mocap(triggerBound_mocap(1):end,i))
end
% figure(7)
% plot3(motionPositionSeries_mocap(triggerBound_shimmer(1):end,1),motionPositionSeries_mocap(triggerBound_shimmer(1):end,2),motionPositionSeries_mocap(triggerBound_shimmer(1):end,3))
% hold on
% plot3(ankle_position_mocap_mocap(triggerBound_mocap(1):end,1),ankle_position_mocap_mocap(triggerBound_mocap(1):end,2),ankle_position_mocap_mocap(triggerBound_mocap(1):end,3))