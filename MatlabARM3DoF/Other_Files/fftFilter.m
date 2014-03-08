% Was using FFT to cutoff high frequency content in acceleration data

%FFT filter acceleration data

accelFFTx = fft(xActualAccel);
accelFFTy = fft(yActualAccel);

%Zero out anything above 50 Hz
%What index is 50 Hz?
%(1:length(xAcualAccel))/5 -> 50*5 = index
cutoff = 40;
T = 5;
index_50Hz = cutoff*T;
accelFFTx(index_50Hz:end) = zeros(size(accelFFTx(index_50Hz:end)));

accelFFTy(index_50Hz:end) = zeros(size(accelFFTy(index_50Hz:end)));

%invert the FFT
fftFilteredAccelX = ifft(accelFFTx);
fftFilteredAccelY = ifft(accelFFTy);

plot(t,xControlAccel,'-b',t,real(fftFilteredAccelX),'-r');
figure
plot(t,yControlAccel,'-b',t,real(fftFilteredAccelY),'-r');