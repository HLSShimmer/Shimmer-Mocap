function J = GetJacobianMatrix(q,magneticInReference)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Get the Jacobian Matrix in Madgwick's Method %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q                      input     quaternion, [q0;q1;q2;q3]
% magneticInReference    input     magnetic in global system, 3x1 vector
% J                      output    Jacobian Matrix£¬6x4
%% declare some values
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
mx = magneticInReference(1);
my = magneticInReference(2);
mz = magneticInReference(3);
%% Jacobian Matrix
J = zeros(6,4);
% J(1,1) = 2*q2;  J(1,2) = 2*q3;  J(1,3) = 2*q0;  J(1,4) = 2*q1;
% J(2,1) = -2*q1; J(2,2) = -2*q0; J(2,3) = 2*q3;  J(2,4) = 2*q2;
% J(3,1) = 0;     J(3,2) = -4*q1; J(3,3) = -4*q2; J(3,4) = 0;
% 
% J(4,1) = 2*mz*q2;
% J(4,2) = 2*mz*q3;
% J(4,3) =  -4*mx*q2 + 2*mz*q0;
% J(4,4) =  -4*mx*q3 + 2*mz*q1;
% 
% J(5,1) = 2*mx*q3 - 2*mz*q1;
% J(5,2) = 2*mx*q2 - 2*mz*q0;
% J(5,3) = 2*mx*q1 + 2*mz*q3;
% J(5,4) = 2*mx*q0 + 2*mz*q2;
% 
% J(6,1) = -2*mx*q2;
% J(6,2) =  2*mx*q3 - 4*mz*q1;
% J(6,3) = -2*mx*q0 - 4*mz*q2;
% J(6,4) =  2*mx*q1;
J(1,1) = -2*q2;  J(1,2) = 2*q3;  J(1,3) = -2*q0;  J(1,4) = 2*q1;
J(2,1) = 2*q1; J(2,2) = 2*q0; J(2,3) = 2*q3;  J(2,4) = 2*q2;
J(3,1) = 0;     J(3,2) = -4*q1; J(3,3) = -4*q2; J(3,4) = 0;

J(4,1) = -2*mz*q2;
J(4,2) = 2*mz*q3;
J(4,3) =  -4*mx*q2 - 2*mz*q0;
J(4,4) =  -4*mx*q3 + 2*mz*q1;

J(5,1) = -2*mx*q3 + 2*mz*q1;
J(5,2) = 2*mx*q2 + 2*mz*q0;
J(5,3) = 2*mx*q1 + 2*mz*q3;
J(5,4) = -2*mx*q0 + 2*mz*q2;

J(6,1) = 2*mx*q2;
J(6,2) = 2*mx*q3 - 4*mz*q1;
J(6,3) = 2*mx*q0 - 4*mz*q2;
J(6,4) = 2*mx*q1;