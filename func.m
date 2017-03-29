function F = func(q,accelInBody,yAxisOfReferenceInBody)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% nonlinear equation set for initial quaternion  %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% accelInBody             input      gravity measurement in sensor system, 3x1 vector
% yAxisOfReferenceInBody  input      yAxis of NED system represented in sensor system, 3x1 vector
% q                       input      value of quaternion, 4x1 vector
% F                       output     F=func(...)=[0;0;...;0;]
%% some vectors
zAxisOfReference = [0;0;1];     %unit vector of zAxis of NED system
yAxisOfReference = [0;1;0];     %unit vector of yAxis of NED system
zAxisOfReferenceTransferedByQuat = CoordinateTransfer(zAxisOfReference,q,'r2b');   %zAxis of NED system transfered by quaternion q
yAxisOfReferenceTransferedByQuat = CoordinateTransfer(yAxisOfReference,q,'r2b');   %yAxis of NED system transfered by quaternion q
%% equantion set
F = zeros(6,1);
F(1) = q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2 - 1;
F(2) = zAxisOfReferenceTransferedByQuat(1) - accelInBody(1);
F(3) = zAxisOfReferenceTransferedByQuat(2) - accelInBody(2);
F(4) = zAxisOfReferenceTransferedByQuat(3) - accelInBody(3);
F(5) = yAxisOfReferenceTransferedByQuat(1) - yAxisOfReferenceInBody(1);
F(6) = yAxisOfReferenceTransferedByQuat(2) - yAxisOfReferenceInBody(2);
% F(7) = yAxisOfReferenceTransferedByQuat(3) - yAxisOfReferenceInBody(3);