% live FFT test

clear all; close; clc

localDataPath = '/data';
captureName = '2024-5-3_DualTag8060_TConfig3_C2.frames';

[frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);

frameCount = width(framesBB);

totalFramesBB = [];

for i = 1:length(frameTot)
    newFrameBB = framesBB(:, i);
    WIP_LiveFFT(abs(newFrameBB));
    
    % totalFramesBB(:, end + 1) = newFrameBB;
    % if mod(i, 100) == 0
    %     captureFT = fft(totalFramesBB, frameCount , 2); 
    % 
    %     liveOutput = max(abs(captureFT));
    %     liveOutput(1:100) = zeros(1, 100);
    %     liveOutput(end-99:end) = zeros(1, 100);
    % 
    %     plot(liveOutput); hold on;
    % end
end