function signalValue = IFFTMethod(signalFFTValue,signalLength)
%%%%%%%%%%%%%%%%%%%
%%% inverse FFT %%%
%%%%%%%%%%%%%%%%%%%
% signalFFTValue           input        FFT value of original signal
% signalLength             input        length of origin signal
% signalValue              output       signal value in time domain

%% declare some values
fftLength = size(signalFFTValue,1);
signalNum = size(signalFFTValue,2);
w = (2*pi*(0:fftLength-1)/fftLength).';
%% inverse fourier transform to get integrated signal in time domain
signalValue = zeros(signalLength,signalNum);
for ii=1:signalNum
    for jj=1:signalLength
        signalValue(jj,ii) = sum(signalFFTValue(:,ii).*exp(w*1j*(jj-1)))/fftLength;
    end
    signalValue(:,ii) = real(signalValue(:,ii));
end