function M = GetQuatMatrix(q)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Get quatternion matrix %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q        input       quaternion，[q0;q1;q2;q3]
% M        ouput       quatternion matrix,4x4 matrix
%% 得出四元素矩阵
M = zeros(4,4);
M(1,1) = q(1); M(1,2) =-q(2); M(1,3) =-q(3); M(1,4) =-q(4);
M(2,1) = q(2); M(2,2) = q(1); M(2,3) =-q(4); M(2,4) = q(3);
M(3,1) = q(3); M(3,2) = q(4); M(3,3) = q(1); M(3,4) =-q(2);
M(4,1) = q(4); M(4,2) =-q(3); M(4,3) = q(2); M(4,4) = q(1);