clear all;close all; clc;
t = 0:0.01:1;
dataNum = length(t);
signals = zeros(dataNum,1);
% signals(501:2001,1) = 5;
% signals(:,2) = 0.4*t;
signals(:,1) = sqrt(t);
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
para.fmin_velocity = 0.05;                    %cutoff frequency, lower & upper bound
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
%% mothed setting
methodSet.dataFilter = 1;         %1-TD
methodSet.quaternion = 1;         %1-Madgwick,2-Fourati
methodSet.accelIntegrate = 2;     %1-TimeDomain;2-FreqencyDomain
methodSet.frequencyFilter = 1;    %1-CutOff;2-Decay

%% 
results = SignalIntegrate4(signals,para,[0;0;0],methodSet,1);

idealResults = zeros(size(results));
% idealResults(501:dataNum,1) = 5*(t(501:dataNum)-t(500));
% idealResults(:,2) = 0.2*t.^2;
idealResults(:,1) = 2/3*t.^1.5;
plot(t,signals(:,1))
hold on
plot(t,results(:,1))
hold on
plot(t,idealResults(:,1))
legend('step','integrate','ideal integrate')
% subplot(131)
% plot(t,signals(:,1))
% hold on
% plot(t,results(:,1))
% hold on
% plot(t,idealResults(:,1))
% legend('step','integrate','ideal integrate')
% subplot(132)
% plot(t,signals(:,2))
% hold on
% plot(t,results(:,2))
% hold on
% plot(t,idealResults(:,2))
% legend('slope','integrate','ideal integrate')
% subplot(133)
% plot(t,signals(:,3))
% hold on
% plot(t,results(:,3))
% hold on
% plot(t,idealResults(:,3))
% legend('squart','integrate','ideal integrate')