function [captureFT, tagFT] = proc_fft(framesBB, frameRate, tagHz)
% [captureFT, tagFT] = proc_fft(framesBB, frameRate, tagHz)
%
% Function to process the FFT of Novelda radar data captures. This function reads in
% the already processed radar data frames and processes the data to extract the FFT
% of the radar frames and the FFT of the tag signal. This is a Stage 2 processing
% function.
%
% Inputs:
%   framesBB: Downconverted, i.e. baseband, and filtered IQ radar signal.
%   frameRate: Frame rate in Hz.
%
% Outputs:
%  captureFT: FFT of the radar frames.
%  tagFT: FFT of the tag signal.

frameCount = width(framesBB);

% Capture FT
captureFT = fft(framesBB, frameCount , 2); 

% Find Tag FT
[tagFT, ~] = tag_index(captureFT, frameRate, tagHz);

end