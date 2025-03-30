% live FFT test

clear all; close; clc

localDataPath = '/data';
captureName = '2024-5-3_DualTag8060_TConfig3_C2.frames';

[frameTot, ~, frameRate] = ProcessFrames(localDataPath, captureName);

frameCount = width(frameTot);

totalFramesBB = [];

for i = 1:1
    frameSnippet = frameTot(:, 1:1000);
    [peakStrength, peakLocation] = WIP_LiveFFT(frameSnippet);
    fprintf("1: %.2f %.2f\n", peakStrength, peakLocation)
    
    frameSnippet = frameTot(:, 1001:end);
    [peakStrength, peakLocation] = WIP_LiveFFT(frameSnippet);
    fprintf("3: %.2f %.2f\n", peakStrength, peakLocation)
    
end