function [motionAccelSeriesAligned,motionVelocitySeriesAligned,motionPositionSeriesAligned] = TransferGlobalSystemFromShimmer2Mocap2(thetaFromShimmer2Mocap_global,motionAccelSeries,motionVelocitySeries,motionPositionSeries,staticPosition_mocap)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% transfer the motion data from shimmer to mocap, global system, rotate angle %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% thetaFromShimmer2Mocap_global       input          rotate angle from shimmer to mocap
% motionAccelSeries                   input          motion accel in shimmer global system£¬3x1
% motionVelocitySeries                input          motion velocity in shimmer global system,3x1
% motionPositionSeries                input          motion displacement in shimmer global system,3x1
% staticPosition_mocap                input          initial position of shimmer sensor in mocap global system,3x1
% motionAccelSeriesAligned            output         aligned motion accel£¬3x1
% motionVelocitySeriesAligned         ouptut         aligned velocity accel£¬3x1
% motionPositionSeriesAligned         ouptut         aligned displacement accel£¬3x1
%% declare some values
dataLength = size(motionAccelSeries,1);
motionAccelSeriesAligned = zeros(dataLength,3);
motionVelocitySeriesAligned = zeros(dataLength,3);
motionPositionSeriesAligned = zeros(dataLength,3);
%% frame transfer
transMatrix = [cos(thetaFromShimmer2Mocap_global),-sin(thetaFromShimmer2Mocap_global);sin(thetaFromShimmer2Mocap_global),cos(thetaFromShimmer2Mocap_global)];
for i=1:dataLength
    motionAccelSeriesAligned(i,1:2) = motionAccelSeries(i,1:2)*transMatrix.';
    motionAccelSeriesAligned(i,3) = motionAccelSeries(i,3);
    motionVelocitySeriesAligned(i,1:2) = motionVelocitySeries(i,1:2)*transMatrix.';
    motionVelocitySeriesAligned(i,3) = motionVelocitySeries(i,3);
    motionPositionSeriesAligned(i,1:2) = motionPositionSeries(i,1:2)*transMatrix.';
    motionPositionSeriesAligned(i,3) = motionPositionSeries(i,3);
end
motionPositionSeriesAligned = motionPositionSeriesAligned + repmat(staticPosition_mocap, dataLength, 1);