function q = QuatMultiplication(q1,q2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% quaternion product %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q1,q2         input     q1*q2
% q             output    q = q1*q2

%% calculate quaternion product
Matrix_q1 = GetQuatMatrix(q1);
q = Matrix_q1*q2;