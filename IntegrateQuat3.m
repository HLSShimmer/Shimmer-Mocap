function qEstimated = IntegrateQuat3(qLastTime,gyroMeasured,accelMeasured,magneticMeasured,gravityInReference,magneticInReference,para)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%       Quaternion Integration         %%%%
%%%%  introduce drift rate fix (Madgwick) %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% qLastTime               input     quaternion of previous time step, [q0;q1;q2;q3]
% gyroMeasured            input     measured angular velocity, sensor system, 3x1 vector
% accelMeasured           input     measured accel, sensor system, 3x1 vector
% magneticMeasured        input     measured magnetic, sensor system, 3x1 vector
% gravityInReference      input     theoretically gravity vector in global system, 3x1 vector
% magneticInReference     input     theoretically magnetic vector in global system, 3x1 vector
% para                    input     some parameters be used
% qEstimated              output    estimated quaternion for current time step
%% calculate the 1st order differential of quaternion at current time step
q_dot = CalculateQuatDot(qLastTime,gyroMeasured);
%% declare some values
u = para.alpha*norm(q_dot)*para.dt;
gama = para.beta/(u/para.dt+para.beta);
accelInBodyNormalised = accelMeasured/norm(gravityInReference);
gravityInReferenceNormalised = gravityInReference/norm(gravityInReference);
magneticMeasuredInReference = CoordinateTransfer(magneticMeasured,qLastTime,'b2r');
magneticMeasuredInReferenceNormalised = magneticMeasuredInReference/norm(magneticMeasuredInReference);
magneticMeasuredInReferenceNormalised(1) = sqrt(sum(magneticMeasuredInReferenceNormalised(1:2).^2));
magneticMeasuredInReferenceNormalised(2) = 0;
magneticInBodyNormalised = magneticMeasured/norm(magneticInReference);
% magneticInBodyNormalised = magneticMeasured/norm(magneticInReference);
% magneticInReferenceNormalised = magneticInReference/norm(magneticInReference);
%% calculate observation error, Jacobian Matrix and gradient of observation error
error = zeros(6,1);
error(1:3) = CoordinateTransfer(gravityInReferenceNormalised,qLastTime,'r2b') - accelInBodyNormalised;
error(4:6) = CoordinateTransfer(magneticMeasuredInReferenceNormalised,qLastTime,'r2b') - magneticInBodyNormalised;
J = GetJacobianMatrix(qLastTime,magneticMeasuredInReferenceNormalised);
errorGradient = J.'*error;
%% Gradient Descent Algorithm for one time
qGradientDescent = qLastTime - u*errorGradient/norm(errorGradient);
%% get estimated quaternion for current time step
qEstimated = gama*qGradientDescent + (1-gama)*(qLastTime+q_dot*para.dt);
qEstimated = qEstimated/norm(qEstimated);   %normalise