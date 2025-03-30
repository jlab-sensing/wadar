% live FFT test

clear all; close; clc

localDataPath = '/data';
captureName = 'comp1.frames';

[frameTot, ~, frameRate] = ProcessFrames(localDataPath, captureName);

frameCount = width(frameTot);

totalFramesBB = [];

for i = 1:10
    frameSnippet = reshape(frameTot, 1, []);
    response = WIP_LiveFFT(frameSnippet);
end