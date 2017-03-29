function quat = GlobalSystemAlignment(triggerAccel_shimmer,triggerAccel_mocap,staticPosition_mocap)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% according the position of trigger motion in two system %%%%
%%%% calculate the rotate quaternion between the systems %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% triggerAccel_shimmer               input          trigger motion coordinates in shimmer£¬nx3
% triggerAccel_mocap                 input          trigger motion coordinates in mocap,nx3
% staticPosition_mocap               input          initial position of shimmer sensor in mocap global system,1x3
% quat                               output         quaternion£¬from shimmer system to mocap system,4x1

%% get the trigger motion displacement in mocap system
% triggerAccel_mocap = triggerAccel_mocap - repmat(staticPosition_mocap,size(triggerAccel_mocap,1),1);
%% calculate rotate quaternion from shimmer to mocap
lowbound = -1*ones(4,1);
upbound = ones(4,1);
options = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt');      %% default£º'trust-region-reflective'£¬or can be set to 'levenberg-marquardt'
% options.FunctionTolerance = 1e-6;
% options.StepTolerance = 1e-7;
% options.MaxIterations = 600;
triggerAccel_mocap = [triggerAccel_mocap,ones(size(triggerAccel_mocap,1),1)];
flag = -1;
while flag <=0
    quat0 = -1 + 2*rand(4,1);
    quat0 = quat0/norm(quat0);
    [quat,~,~,flag] = lsqcurvefit(@func2,quat0,triggerAccel_shimmer,triggerAccel_mocap,lowbound,upbound,options);
end