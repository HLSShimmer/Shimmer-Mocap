function [quatSeries,motionAccelSeries,motionVelocitySeries,motionPositionSeries,dataStaticFiltered,dataMotionFiltered] = SensorKinematics(sensorStatic,sensorMotion,positionInitial,velocityInitial,para,methodSet)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%      calculate a sensor's kinematics information         %%%
%%% including quaternion, motion accel/velocity/displacement %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sensorStatic,sensorMotion          input         data when keeping still & moving
% positionInitial,velocityInitial    input         initial position & velocity, global system
% para                               input         parameters for methods
% methodSet                          input         method settings
% quatSeries                         output        calculated quaternion in time series, from sensor system to global system
% motionAccelSeries                  output        calculated motion accel in time series, global system
% motionVelocitySeries               output        calculated motion velocity in time series, global system
% motionPositionSeries               output        calculated motion displacement in time series, global system
% dataStaticFiltered                 output        filtering result of dataStatic
% dataMotionFiltered                 output        filtering result of dataMotion

%% extract 9DOF data from data struct, and filter data. 9 cols of [Gyro_xyz,Accel_xyz,Magnetic_xyz]
dataMotion = [sensorMotion.Gyro/180*pi,sensorMotion.Accel_LowNoise,sensorMotion.Magnetic];
dataStatic = [sensorStatic.Gyro/180*pi,sensorStatic.Accel_LowNoise,sensorStatic.Magnetic];
dataMotionFiltered = FilterData(dataMotion,para.dt,methodSet.dataFilter,para);
dataStaticFiltered = FilterData(dataStatic,para.dt,methodSet.dataFilter,para);
%% Based on data when standing still, initialize quaternion and get magnetic and gravity vector in global system
[qInitial,magneticVectorInReference,gravityVectorInReference] = InitiateQuat(dataStaticFiltered(:,4:6),dataStaticFiltered(:,7:9));

%% calculate sensor's kinematics information
[quatSeries,motionAccelSeries,motionVelocitySeries,motionPositionSeries] = IntegrateInTimeSeries(dataMotionFiltered(:,1:3),dataMotionFiltered(:,4:6),dataMotionFiltered(:,7:9),qInitial,positionInitial,velocityInitial,gravityVectorInReference,magneticVectorInReference,para,methodSet);