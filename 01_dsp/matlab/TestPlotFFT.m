% This script serves as a demonstration of how to use the basic functions in the
% DSP module. This script will load a radar capture, process the frames, and
% then plot the raw radar frames and the FFT of the radar frames.

close all; clear; clc

% You can change these parameters to match the radar capture you want to
localDataPath = '/data';
captureName = 'test1.frames';
tagHz = 80;

% First, we will process the radar frames to extract the baseband signal and the raw radar frames
[frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);

% Next, we will process the FFT of the radar frames
[captureFT, tagFT] = ProcessFFT(framesBB, frameRate, tagHz);

% Finally, we will visualize the raw radar frames and the FFT of the radar frames
PlotFrames(frameTot, framesBB)
PlotTag(captureFT, tagFT, frameRate)