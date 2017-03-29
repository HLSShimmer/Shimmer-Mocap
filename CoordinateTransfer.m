function xTransfered = CoordinateTransfer(xOrigin,q,direction)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% transfer coordinates by quaternion and direction %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% xOrigin        input     coordiante value in original system, 3x1 vector
% q              input     quaternion representing the rotation from sensor system to global system, 4x1 vector
% direction      input     rotation direction, r2b: global to sensor£»b2r: sensor to global
% xTransfered    output    coordiante value sfter transfered, 3x1 vector
%% conjugate quaternion
q_ = [q(1);-q(2:4)];
%% transfer coordinate value based on direction
xOriginQuat = [0;xOrigin];
if strcmp(direction,'b2r')
    % transfered = q*origin*q_
    xTransfered = QuatMultiplication(q,xOriginQuat);
    xTransfered = QuatMultiplication(xTransfered,q_);
    xTransfered = xTransfered(2:4);
%     xTransfered = xTransfered/norm(xTransfered);
elseif strcmp(direction,'r2b')
    % transfered = q_*origin*q
    xTransfered = QuatMultiplication(q_,xOriginQuat);
    xTransfered = QuatMultiplication(xTransfered,q);
    xTransfered = xTransfered(2:4);
%     xTransfered = xTransfered/norm(xTransfered);
end