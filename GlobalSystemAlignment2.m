function theta = GlobalSystemAlignment2(triggerAccel_shimmer,triggerAccel_mocap,staticPosition_mocap)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% according the position of trigger motion in two system %%%%
%%%% calculate the rotate angle between the systems %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% triggerPosition_shimmer               input          trigger motion coordinates in shimmer£¬nx3
% triggerPosition_mocap                 input          trigger motion coordinates in mocap,nx3
% staticPosition_mocap                  input          initial position of shimmer sensor in mocap global system,1x3
% theta                                 output         rotate angle from shimmer to mocap,3x1
%% get the trigger motion displacement in mocap system
% triggerAccel_mocap = triggerAccel_mocap - repmat(staticPosition_mocap,size(triggerAccel_mocap,1),1);
%% calculate rotate angle from shimmer to mocap
lowbound = -pi;
upbound = pi;
options = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt');      %% default£º'trust-region-reflective'£¬or can be set to 'levenberg-marquardt'
% options.FunctionTolerance = 1e-6;
% options.StepTolerance = 1e-7;
% options.MaxIterations = 600;
options.Display = 'iter';
triggerAccel_mocap_XY = triggerAccel_mocap(:,1:2);
flag = -1;
while flag <=0
    theta0 = (-1 + 2*rand)*pi;
    [theta,~,~,flag] = lsqcurvefit(@func3,theta0,triggerAccel_shimmer(:,1:2),triggerAccel_mocap_XY,lowbound,upbound,options);
end