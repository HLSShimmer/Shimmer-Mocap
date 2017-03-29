function result = SignalIntegrate3(signal,para,initialState,methodSet,type)
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
if type==1
    fmin = para.fmin_velocity;                                %lower bound of cutoff frequency
    fmax = para.fmax_velocity;                                %upper bound of cutoff frequency
    fTarget = para.fTarget_velocity;                          %target frequncy
    integrateAccuracy = para.integrateAccuracy_velocity;      %integrate accuracy
elseif type==2
    fmin = para.fmin_position;                            %lower bound of cutoff frequency
    fmax = para.fmax_position;                            %upper bound of cutoff frequency
    fTarget = para.fTarget_position;                      %target frequncy
    integrateAccuracy = para.integrateAccuracy_position;  %integrate accuracy
end
dt = para.dt;                                %sample time step
sf = 1/dt;                                   %sample rate
signalLength = size(signal,1);               %length of signals
signalNum = size(signal,2);                  %number of signals in cols
fftLengthHalf = 3*signalLength;
fftLength = 2*fftLengthHalf+1;               %length of fourier transform
%% some variables related to fourier transformation
signalExtended = [signal;zeros(fftLength-signalLength,signalNum)];
% signalFFTValue = zeros(fftLengthHalf+1,signalNum);
signalFFTValue = zeros(fftLength,signalNum);
resultFFTValue = zeros(fftLength,signalNum);
w = zeros(fftLength,1);
for ii=1:fftLength
    w(ii) = 2*pi*(ii-1)/fftLength;
end
% H = zeros(fftLengthHalf+1,1);
% H(2:end) = 1./(1j*pi*(1:fftLengthHalf).'*sf/fftLength);
H = zeros(fftLength,1);
H(2:end) = 1./(1j*pi*(1:fftLength-1).'*sf/fftLength);
%% calculate fourier tranform, and make integration in frequency domain
for ii=1:signalNum
    Y = zeros(fftLength+1,1);
%     for jj=1:fftLengthHalf+1
%         signalFFTValue(jj,ii) = sum(signalExtended(:,ii).*exp(-w*1j*(jj-1)));
%     end
    for jj=1:fftLength
        signalFFTValue(jj,ii) = sum(signalExtended(:,ii).*exp(-w*1j*(jj-1)));
    end
%     Y(1:fftLengthHalf+1) = signalFFTValue(:,ii).*H;
%     Y(fftLengthHalf+2:end) = (Y(fftLengthHalf+1:-1:2)').';
    Y = signalFFTValue(:,ii).*H;
    resultFFTValue(:,ii) = Y;
end
%% process of frequency filter
if methodSet.frequencyFilter == 1        % CutOff
    fminIndex = ceil(fmin/sf*fftLength);
    fmaxIndex = ceil(fmax/sf*fftLength);
%     resultFFTValue([1:fminIndex,end-fminIndex+1:end],:) = 0;
%     resultFFTValue(fmaxIndex:fftLengthHalf+1+fmaxIndex,:) = 0;
    resultFFTValue(1:fminIndex,:) = 0;
    resultFFTValue(fmaxIndex:end,:) = 0;
elseif methodSet.frequencyFilter == 2    % Deycay
    fRelative = 2*pi.*w/fTarget;
    decayRate = fRelative.^4 ./ (fRelative.^4 + (1-integrateAccuracy)/integrateAccuracy);
    for ii=1:signalNum
        resultFFTValue(:,ii) = decayRate .* resultFFTValue(:,ii);
    end
    fmaxIndex = ceil(fmax/sf*fftLength);
    resultFFTValue(fmaxIndex:end,:) = 0;
end
%% inverse fourier transform to get integrated signal in time domain
result = zeros(signalLength,signalNum);
for ii=1:signalNum
    for jj=1:signalLength
        result(jj,ii) = sum(resultFFTValue(:,ii).*exp(w*1j*(jj-1)))/fftLength;
    end
    result(:,ii) = real(result(:,ii));
end
% result = result - repmat(result(1,:),signalLength,1);
% result = result + repmat(initialState.',signalLength,1);