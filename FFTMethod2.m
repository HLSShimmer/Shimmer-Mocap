function [signalFFTValue,signalIntegrateFFTValue,signalCutoffFFTValue] = FFTMethod2(signal,para,methodSet,type)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                       FFT Method
%%% calculate FFT value of original signal, integrated signal and the
%%% information filtered by frequency filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% signal                         input      signal to be integrate,  rows represent sample time step, cols represent how many type of signals need to be integrated
% para                           input      some parameters be used
% methodSet                      input      method settings
% type                           input      indicate accel-to-velocity or velocity-to-position, 1:velocity  2:position
% signalFFTValue                 output     FFT value of origin signal
% signalIntegrateFFTValue        output     FFT value of integrated signal,been cut off
% signalCutoffFFTValue           output     the value of cutoff series
%% declare some values
if type==1
    fmin = para.fmin_velocity;                                %lower bound of cutoff frequency
    fmax = para.fmax_velocity;                                %upper bound of cutoff frequency
    fTarget = para.fTarget_velocity;                          %target frequncy
    integrateAccuracy = para.integrateAccuracy_velocity;      %integrate accuracy
elseif type==2
    fmin = para.fmin_position;                                %lower bound of cutoff frequency
    fmax = para.fmax_position;                                %upper bound of cutoff frequency
    fTarget = para.fTarget_position;                          %target frequncy
    integrateAccuracy = para.integrateAccuracy_position;      %integrate accuracy
end
dt = para.dt;                                %sample time step
sf = 1/dt;                                   %sample rate
signalLength = size(signal,1);               %length of signals
signalNum = size(signal,2);                  %number of signals in cols
fftLengthHalf = ceil(pi/para.fResolution);
% fftLengthHalf = 4*signalLength;
fftLength = 2*fftLengthHalf+1;               %length of fourier transform
%% some variables related to fourier transformation
signalExtended = [signal;zeros(fftLength-signalLength,signalNum)];
% signalFFTValue = zeros(fftLengthHalf+1,signalNum);
signalFFTValue = zeros(fftLength,signalNum);
signalIntegrateFFTValue = zeros(fftLength,signalNum);
signalCutoffFFTValue = zeros(fftLength,signalNum);
w = (2*pi*(0:fftLength-1)/fftLength).';
H = zeros(fftLength,1);
H(2:end) = 1./(1j*pi*(1:fftLength-1).'*sf/fftLength);
%% calculate fourier tranform, and make integration in frequency domain
for ii=1:signalNum
    %%fourier tranform
    Y = zeros(fftLength+1,1);
    for jj=1:fftLength
        signalFFTValue(jj,ii) = sum(signalExtended(:,ii).*exp(-w*1j*(jj-1)));
    end
    %%process of frequency filter
    if methodSet.frequencyFilter == 1        % CutOff
        fminIndex = ceil(fmin/sf*fftLengthHalf);
        fmaxIndex = ceil(fmax/sf*fftLengthHalf);
        signalCutoffFFTValue([1:fminIndex,end-fminIndex+2:end],ii) = signalFFTValue([1:fminIndex,end-fminIndex+2:end],ii);
        signalCutoffFFTValue(fmaxIndex:2*fftLengthHalf-fmaxIndex,ii) = signalFFTValue(fmaxIndex:2*fftLengthHalf-fmaxIndex,ii);
        signalFFTValue([1:fminIndex,end-fminIndex+2:end],ii) = 0;
        signalFFTValue(fmaxIndex:2*fftLengthHalf-fmaxIndex,ii) = 0;
    elseif methodSet.frequencyFilter == 2    % Deycay
        fmaxIndex = ceil(fmax/sf*fftLengthHalf);
        signalCutoffFFTValue(fmaxIndex:2*fftLengthHalf-fmaxIndex,ii) = signalFFTValue(fmaxIndex:2*fftLengthHalf-fmaxIndex,ii);
        fRelative = 2*pi.*w(1:fftLengthHalf+1)/fTarget;
        fRelative = [fRelative;fRelative(end:-1:2)];
        decayRate = fRelative.^4 ./ (fRelative.^4 + (1-integrateAccuracy)/integrateAccuracy);
        signalCutoffFFTValue(:,ii) = (1-decayRate).*signalFFTValue(:,ii);
        signalFFTValue(:,ii) = decayRate .* signalFFTValue(:,ii);
        signalFFTValue(fmaxIndex:end,ii) = 0;
    end
    %%integrate in frequency domain
    Y = signalFFTValue(:,ii).*H;
    signalIntegrateFFTValue(:,ii) = Y;
end