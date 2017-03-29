function [q,magneticInReference,gravityInReference] = InitiateQuat(staticAccel,staticMagnetic)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% initialize quaternion depending on measured magnetic and gravoty signal %%%%%
%%%%%                 (sensor system to NED global system)                    %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% staticAccel            input      gravity signal measurement when standing still, in sensor system, 3x1 vector
% staticMagnetic         input      magnetic signal measurement when standing still, in sensor system, 3x1 vector
% q                      output     initial value of quaternion, 4x1 vector
% magneticInReference    output     magnetic measurement value transfered to NED system, 3x1 vector
% gravityInReference     output     gravity measurement value transfered to NED system, 3x1 vector
%% get estimated gravity and magnetic value in NED system, and then normalise
accelInBody = zeros(3,1);
magneticInBody = zeros(3,1);
accelInBody(1) = mean(staticAccel(:,1));
accelInBody(2) = mean(staticAccel(:,2));
accelInBody(3) = mean(staticAccel(:,3));
magneticInBody(1) = mean(staticMagnetic(:,1));
magneticInBody(2) = mean(staticMagnetic(:,2));
magneticInBody(3) = mean(staticMagnetic(:,3));
accelInBodyNormalised = accelInBody./sqrt(sum(accelInBody.^2));
magneticInBodyNormalised = magneticInBody./sqrt(sum(magneticInBody.^2));
%% some vector need to be used
yAxisOfReferenceInBody = cross(accelInBodyNormalised,magneticInBodyNormalised);       %yAxis of NED system that represented in sensor system
yAxisOfReferenceInBody = yAxisOfReferenceInBody/norm(yAxisOfReferenceInBody);
%% solve nonlinear equation set to get the quaternion transfer sensor system to NED system
options = optimset('fsolve');
options = optimset(options,'Display','iter','TolFun',1e-7,'MaxIter',1e10,'Tolx',1e-7);
while 1
%     q0 = [1;0;0;0];
    q0 = -1 + 2*rand(4,1);
    q0 = q0/norm(q0);
    [q,fval,exitflag] = fsolve(@(q)func(q,accelInBodyNormalised,yAxisOfReferenceInBody),q0,options);
    if exitflag>0
        break;
    end
end
magneticInReference = CoordinateTransfer(magneticInBody,q,'b2r');
gravityInReference = CoordinateTransfer(accelInBody,q,'b2r');