function qEstimated = LMA(q,gyroMeasured,accelMeasured,magneticMeasured,gravityInReference,magneticInReference,para)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% LMA implementation for etting estimated quaternion %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q                      input       initial value of quaternion, [q0;q1;q2;q3]
% w                      input       measured angular velocity, sensor system, 3x1 vector
% accelMeasured          input       measured accel, sensor system, 3x1 vector
% magneticMeasured       input       measured magnetic, sensor system, 3x1 vector
% gravityInReference     input       theoretically gravity vector in global system, 3x1 vector
% magneticInReference    input       theoretically magnetic vector in global system, 3x1 vector
% para                   input       some parameters be used
% qEstimated             output      estimated quaternion for current time step
%% declare some values
maxIter = 600;
tolerance = 1e-5;
count = 1;
qEstimated = q;
delta_q_lastTime = [inf;inf;inf];
error_lastTime = [inf;inf;inf];
motionAccelInBody = (accelMeasured - CoordinateTransfer(gravityInReference,q,'r2b'))/norm(gravityInReference);
motionAccelInReference = CoordinateTransfer(motionAccelInBody,qEstimated,'b2r');
%% mormalise measure accel & magnetic
accelMeasured = accelMeasured/norm(gravityInReference);
magneticMeasured = magneticMeasured/norm(magneticMeasured);
%% loop to get optimal quaternion
while count<maxIter
    %% transfer gravity&magnetic to sensor system, then calculate observation error
    gravityInBody = CoordinateTransfer(gravityInReference,qEstimated,'r2b');
    gravityInBody = gravityInBody/norm(gravityInBody);
    magneticInBody = CoordinateTransfer(magneticInReference,qEstimated,'r2b');
    magneticInBody = magneticInBody/norm(magneticInBody);
    error = [accelMeasured;magneticMeasured] - [gravityInBody;magneticInBody];
    %% calculate Jacobian Matrix, and increment of quaternion
    J = zeros(6,3);              %Jacobian Matrix
    J(1,1) = 0;  J(1,2) = accelMeasured(3); J(1,3) = -accelMeasured(2);
    J(2,1) = -accelMeasured(3);  J(2,2) = 0; J(2,3) = accelMeasured(1);
    J(3,1) = accelMeasured(2);  J(3,2) = -accelMeasured(1); J(3,3) = 0;
    J(4,1) = 0;  J(4,2) = magneticMeasured(3); J(4,3) = -magneticMeasured(2);
    J(5,1) = -magneticMeasured(3);  J(5,2) = 0; J(5,3) = magneticMeasured(1);
    J(6,1) = magneticMeasured(2);  J(6,2) = -magneticMeasured(1); J(6,3) = 0;
    J = -2*J;
    delta_q = (J.'*J+para.lamda*eye(3))\J.'*error;
    fixVector = [1;para.k*delta_q];
    %% calculate update value
    q_dot = QuatMultiplication(qEstimated,[0;gyroMeasured]);
    q_dot = QuatMultiplication(q_dot,fixVector);
    q_dot = q_dot/2;
    qNew = qEstimated + q_dot*para.dt;
%     qNew = qNew/norm(qNew);
    %% get the variety trend value 'rho'
    gravityInBody = CoordinateTransfer(gravityInReference,qNew,'r2b');
    gravityInBody = gravityInBody/norm(gravityInBody);
    magneticInBody = CoordinateTransfer(magneticInReference,qNew,'r2b');
    magneticInBody = magneticInBody/norm(magneticInBody);
    errorNew = [accelMeasured;magneticMeasured] - [gravityInBody;magneticInBody];
    rho = (error.'*error - errorNew.'*errorNew)/dot(J.'*error,delta_q);
%     rho = (delta_q_lastTime.'*delta_q_lastTime - delta_q.'*delta_q)/dot(J.'*error,delta_q);
    %% judgement
    if errorNew.'*errorNew<tolerance
        break;
    elseif rho>0 && rho<=0.25
        qEstimated = qNew;
    elseif rho>0.25
        qEstimated = qNew;
        para.lamda = 0.8*para.lamda;
    else
        para.lamda = 1.2*para.lamda;
    end
    count = count + 1;
end
qEstimated = qEstimated/norm(qEstimated);