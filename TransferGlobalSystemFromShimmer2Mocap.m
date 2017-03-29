function [motionAccelSeriesAligned,motionVelocitySeriesAligned,motionPositionSeriesAligned] = TransferGlobalSystemFromShimmer2Mocap(quatFromShimmer2Mocap_global,motionAccelSeries,motionVelocitySeries,motionPositionSeries,staticPosition_mocap)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% transfer the motion data from shimmer to mocap, global system, rotate quaternion %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% quatFromShimmer2Mocap_global        input          rotate quaternion from shimmer to mocap£¬4x1
% motionAccelSeries                   input          motion accel in shimmer global system£¬3x1£¬3x1
% motionVelocitySeries                input          motion velocity in shimmer global system,3x1
% motionPositionSeries                input          motion displacement in shimmer global system£¬3x1
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
for i=1:dataLength
    motionAccelSeriesAligned(i,:) = CoordinateTransfer(motionAccelSeries(i,:).', quatFromShimmer2Mocap_global, 'b2r').';
    motionVelocitySeriesAligned(i,:) = CoordinateTransfer(motionVelocitySeries(i,:).', quatFromShimmer2Mocap_global, 'b2r').';
    motionPositionSeriesAligned(i,:) = CoordinateTransfer(motionPositionSeries(i,:).', quatFromShimmer2Mocap_global, 'b2r').';
end
motionPositionSeriesAligned = motionPositionSeriesAligned + repmat(staticPosition_mocap, dataLength, 1);