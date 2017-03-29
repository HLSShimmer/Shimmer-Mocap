function signalIntegrate = SignalDoubleIntegrate(signal,para,initialState,methodSet)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% double integration in frequency domain %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% signal               input      signal to be integrate,  rows represent sample time step, cols represent how many type of signals need to be integrated
% para                 input      some parameters be used
% initialState         input      initial value
% methodSet            input      method settings
% result               output     result of double integration
%% declare some values
fmin = para.fmin_position;                      %lower bound of cutoff frequency
fmax = para.fmax_position;                      %upper bound of cutoff frequency
fTarget = para.fTarget_position;
integrateAccuracy = para.integrateAccuracy_position;
dt = para.dt;                          %sample time step
sf = 1/dt;                             %sample rate
signalLength = size(signal,1);         %length of signals
signalNum = size(signal,2);            %number of signals in cols
fftLengthHalf = ceil(pi/para.fResolution);
fftLength = 2*fftLengthHalf+1;               %length of fourier transform
%% some variables related to fourier transformation
signalExtended = [signal;zeros(fftLength-signalLength,signalNum)];
% signalFFTValue = zeros(fftLengthHalf+1,signalNum);
signalFFTValue = zeros(fftLength,signalNum);
signalIntegrateFFTValue = zeros(fftLength,signalNum);
signalIntegrateCutoffFFTValue = zeros(fftLength,signalNum);
w = (2*pi*(0:fftLength-1)/fftLength).';
% H = zeros(fftLengthHalf+1,1);
% H(2:end) = 1./(1j*pi*(1:fftLengthHalf).'*sf/fftLength);
H = zeros(fftLength,1);
H(2:end) = 1./(1j*pi*(1:fftLength-1).'*sf/fftLength);
% H(1) = 1./(1j*pi*para.fResolution/3);
H = H.*H;
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
    signalIntegrateFFTValue(:,ii) = Y;
end
%% process of frequency filter
% if methodSet.frequencyFilter == 1        % CutOff
%     fminIndex = ceil(fmin/sf*fftLength);
%     fmaxIndex = ceil(fmax/sf*fftLength);
% %     signalIntegrateCutoffFFTValue([1:fminIndex,end-fminIndex+1:end],:) = resultFFTValue([1:fminIndex,end-fminIndex+1:end],:);
% %     signalIntegrateCutoffFFTValue(fmaxIndex:fftLengthHalf+1+fmaxIndex,:) = resultFFTValue(fmaxIndex:fftLengthHalf+1+fmaxIndex,:);
% %     resultFFTValue([1:fminIndex,end-fminIndex+1:end],:) = 0;
% %     resultFFTValue(fmaxIndex:fftLengthHalf+1+fmaxIndex,:) = 0;
%     signalIntegrateCutoffFFTValue(2:fminIndex,:) = signalIntegrateFFTValue(2:fminIndex,:);
%     signalIntegrateCutoffFFTValue(fmaxIndex:end,:) = signalIntegrateFFTValue(fmaxIndex:end,:);
%     signalIntegrateFFTValue(2:fminIndex,:) = 0;
%     signalIntegrateFFTValue(fmaxIndex:end,:) = 0;
% %     for ii=1:signalNum
% %         signalIntegrateCutoffFFTValue(1:fminIndex(ii),:) = signalIntegrateFFTValue(1:fminIndex(ii),:);
% %         signalIntegrateCutoffFFTValue(fmaxIndex(ii):end,:) = signalIntegrateFFTValue(fmaxIndex(ii):end,:);
% %         signalIntegrateFFTValue(1:fminIndex(ii),:) = 0;
% %         signalIntegrateFFTValue(fmaxIndex(ii):end,:) = 0;
% %     end
% elseif methodSet.frequencyFilter == 2    % Deycay
%     fRelative = 2*pi.*w/fTarget;
%     decayRate = fRelative.^4 ./ (fRelative.^4 + (1-integrateAccuracy)/integrateAccuracy);
%     decayRate(1) = 1;
%     for ii=1:signalNum
%         signalIntegrateCutoffFFTValue(:,ii) = (1-decayRate).*signalIntegrateFFTValue(:,ii);
%         signalIntegrateFFTValue(:,ii) = decayRate .* signalIntegrateFFTValue(:,ii);
%     end
%     fmaxIndex = ceil(fmax/sf*fftLength);
%     signalIntegrateCutoffFFTValue(fmaxIndex:end,:) = signalIntegrateFFTValue(fmaxIndex:end,:);
%     signalIntegrateFFTValue(fmaxIndex:end,:) = 0;
% end
%% inverse fourier transform to get integrated signal in time domain
signalIntegrate = IFFTMethod(signalIntegrateFFTValue,signalLength);
% result = result - repmat(result(1,:),signalLength,1);
% result = result + repmat(initialState.',signalLength,1);