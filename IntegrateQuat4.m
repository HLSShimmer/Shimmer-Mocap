function qEstimated = IntegrateQuat4(qInitial,w,accel,magnetic,accelIdeal,magneticIdeal,para)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%       Quaternion Integration         %%%%
%%%%  introduce drift rate fix (Fourati)  %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% qInitial      input     quaternion of previous time step, [q0;q1;q2;q3]
% w             input     measured angular velocity, sensor system, 3x1 vector
% accel         input     measured accel, sensor system, 3x1 vector
% magnetic      input     measured magnetic, sensor system, 3x1 vector
% accelIdeal    input     theoretically gravity vector in global system, 3x1 vector
% magneticIdeal input     theoretically magnetic vector in global system, 3x1 vector
% para          input     some parameters be used
% q1            output    estimated quaternion for current time step
%% use LMA algorithm to get estimated quaternion at current time step
qEstimated = LMA(qInitial,w,accel,magnetic,accelIdeal,magneticIdeal,para);
qEstimated = qEstimated/norm(qEstimated);   %normalise