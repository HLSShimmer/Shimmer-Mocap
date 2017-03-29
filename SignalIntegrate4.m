function result = SignalIntegrate4(signal,para,initialState,methodSet,type)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% integration in frequency domain %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% signal               input      signal to be integrate,  rows represent sample time step, cols represent how many type of signals need to be integrated
% para                 input      some parameters be used
% initialState         input      initial value
% methodSet            input      method settings
% type                 input      indicate accel-to-velocity or velocity-to-position, 1:velocity  2:position
% result               output     result of integration
%% declare some values
signalLength = size(signal,1);        %length of signals
%% FFT transform
[signalFFTValue,resultFFTValue,resultCutoffFFTValue] = FFTMethod2(signal,para,methodSet,type);
%% 
resultCutoff = IFFTMethod(resultCutoffFFTValue,signalLength);
resultCutoffDetrend = detrend(resultCutoff);
%% use IFFT to get integrated result
result = IFFTMethod(resultFFTValue,signalLength);
% result = detrend(result);
% result = result - repmat(result(1,:),signalLength,1);
% result_ = result + resultCutoffDetrend;