% This script serves as a demonstration of how to use the basic functions in the
% DSP module. This script will load a radar capture, process the frames, and
% then get the peak bin of the tag signal using the CWT and correlation methods.

close all; clear; clc

% You can change these parameters to match the radar capture you want to use
localDataPath = '/data';
captureName = '2024-4-17_DualTag7980_T3retest2_C1.frames';
tagHz = 79;

% First, we will process the radar frames to extract the baseband signal and the raw radar frames
[frameTot, framesBB, frameRate] = proc_frames('/data', '2024-4-17_DualTag7980_T3retest2_C1.frames');

% Next, we will process the FFT of the radar frames
[captureFT, tagFT] = proc_fft(framesBB, frameRate, tagHz);

[tagPeakBinCWT] = tag_cwt(tagFT, false);
[tagPeakBinCorrelation] = tag_correlation(tagFT, tagFT);

fprintf("CWT method results in peak bin #%d\n", tagPeakBinCWT)
fprintf("Correlation method results in peak bin #%d\n", tagPeakBinCorrelation)