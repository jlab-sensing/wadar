% live FFT test

clear all; close; clc

localDataPath = '/data';
captureName = '2024-4-17_DualTag7980_T3retest2_C1.frames';

[frameTot, ~, frameRate] = ProcessFrames(localDataPath, captureName);

frameCount = width(frameTot);

totalFramesBB = [];

for i = 1:10
    frameSnippet = reshape(frameTot, 1, []);
    response = WIP_LiveFFT(frameSnippet);
end