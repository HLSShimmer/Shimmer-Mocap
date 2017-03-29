function [qInTimeSeries,motionAccelInReferenceInTimeSeries,motionVelocityInReferenceInTimeSeries,motionPositionInReferenceInTimeSeries] = IntegrateInTimeSeries(gyro,accel,magnetic,qInitial,positionInitial,velocityInitial,gravityInReference,magneticInRefernce,para,methodSet)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% integrate to get quaternion, accel, velocity, displacement %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% gyro                                    input      angular velocity, in sensor system, 3x1 vector
% accel                                   input      acceleration, in sensor system, 3x1 vector
% magnetic                                input      magnetic, in sensor system, 3x1 vector
% qInitial                                input      initial quaternion£¬[q0;q1;q2;q3]
% positionInitial                         input      initial position, global system (NED)
% accelIdeal                              input      ideal gravity in global system (NED)
% magneticIdeal                           input      ideal magnetic in global system (NED)
% para                                    input      some parameters be used
% methodSet                               input      method settings
% qInTimeSeries                           output     calculated quaternion in series
% motionAccelInReferenceInTimeSeries      output     calculated motion accel in global system
% motionVelocityInReferenceInTimeSeries   output     calculated motion velocity in global system
% motionPositionInReferenceInTimeSeries   output     calculated motion displacement in global system
%% declare some values
dataNum = size(gyro,1);             %length of data
qInTimeSeries = zeros(dataNum,4);
motionAccelInReferenceInTimeSeries = zeros(dataNum,3);
%% loop every step and integrate to get quaternion
qInTimeSeries(1,:) = qInitial.';
qTemp = qInitial;                   %represent the quaternion of current time
motionAccelInReferenceInTimeSeries(1,:) = CoordinateTransfer(accel(1,:).',qInitial,'b2r').';    
for i=2:dataNum
    %%calculate quaternion at current time
    if methodSet.quaternion == 1          % Madgwick
        qTemp = IntegrateQuat3(qTemp,gyro(i,:).',accel(i,:).',magnetic(i,:).',gravityInReference,magneticInRefernce,para);
    elseif methodSet.quaternion == 2      % Fourati
        qTemp = IntegrateQuat4(qTemp,gyro(i,:).',accel(i,:).',magnetic(i,:).',gravityInReference,magneticInRefernce,para);
    end
    qInTimeSeries(i,:) = qTemp.';
    %%at current time, transfer measured accel to global system
    motionAccelInReferenceInTimeSeries(i,:) = CoordinateTransfer(accel(i,:).',qInTimeSeries(i,:).','b2r');
end
motionAccelInReferenceInTimeSeries = motionAccelInReferenceInTimeSeries - repmat(gravityInReference.',dataNum,1);
%% double integration to get motion velocity & displacement
if methodSet.accelIntegrate == 1         % Time Domain
    [motionVelocityInReferenceInTimeSeries,motionPositionInReferenceInTimeSeries] = AccelIntegrate3(motionAccelInReferenceInTimeSeries,velocityInitial,positionInitial,para);
elseif methodSet.accelIntegrate == 2     % Frequency Domain
    motionVelocityInReferenceInTimeSeries = SignalIntegrate4(motionAccelInReferenceInTimeSeries,para,velocityInitial,methodSet,1);
    motionPositionInReferenceInTimeSeries = SignalIntegrate4(motionVelocityInReferenceInTimeSeries,para,positionInitial,methodSet,2);
end