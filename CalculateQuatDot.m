function q_dot = CalculateQuatDot(q,w)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% calculate quaternion 1st order differential %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q       input      quaternion at current time, [q0;q1;q2;q3]
% w       input      measured angular velocity in sensor system£¬w=[wx;wy;wz]
%% equation q_dot = 0.5*W*q (Matrix product) or q_dot = 0.5*q*w (quaternion product)
w_quat = [0;w];
q_dot = QuatMultiplication(q,w_quat)/2;
% W = zeros(4,4);
% W(1,1) = 0;     W(1,2) = -w(1);  W(1,3) = -w(2);  W(1,4) = -w(3);
% W(2,1) = w(1);  W(2,2) = 0;      W(2,3) = w(3);   W(2,4) = -w(2);
% W(3,1) = w(2);  W(3,2) = -w(3);  W(3,3) = 0;      W(3,4) = w(1);
% W(4,1) = w(3);  W(4,2) = w(2);   W(4,3) = -w(1);  W(4,4) = 0;
% q_dot = 0.5*W*q;