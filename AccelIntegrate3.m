function [motionVelocity,motionPosition] = AccelIntegrate3(motionAccel,initialVelocity,initialPosition,para)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% detect motion and do double integration %%%%
% 3-D Position Estimation from Inertial Sensing:
% Minimizing the Error from the Process of Double Integration of Acceleration 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% motionAccel                     input      motion accel,each row represent sample at each time step, each col represent xyz axis
% intialVelocity                  input      initial value of velocity
% initialPosition                 input      initial value of position
% para                            input      some parameters be used
% motionVelocity, motionPosition  output     result of double integration
%% extract motion
motionlAccelMasked = ExtractMotion(motionAccel,para);

%% by using the mask, doing the double integration, RK4 method
motionVelocity = SignalIntegration_RK4(motionlAccelMasked,para.dt,initialVelocity,'V');
% motionVelocity = detrend(motionVelocity);
motionPosition = SignalIntegration_RK4(motionVelocity,para.dt,initialPosition,'P');
% motionPosition = detrend(motionPosition);