function viz_frames(frameTot, framesBB)
% viz_fft(captureFT, tagFT, frameRate)
%
% Function to visualize the FFT of Novelda radar data captures. This function reads in
% the pre-loaded radar data frames and the downconverted, i.e. baseband, and filtered IQ radar
% signal and visualizes the data.
%
% Inputs:
%   frameTot: raw radar data frames
%   framesBB: downconverted, i.e. baseband, and filtered IQ radar signal
%
% Outputs:
%   None

figure
subplot(2,1,1)
plot(median(frameTot, 2))
xlabel('Range Bins')
ylabel('DAC')
title('Raw radar frames')

subplot(2,1,2)
plot(median(abs(framesBB), 2))
xlabel('Range Bins')
ylabel('DAC')
title('Radar frames after digital downcovert')

end