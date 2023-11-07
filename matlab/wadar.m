function vwc = wadar(airFileName, captureName)
% vwc = wadar(airFileName, captureName)
%
% Script calculates the volumetric water content (vwc) of the soil from 
% the capture
%
% Inputs:
%        airFileName: Radar capture with tag uncovered with soil
%        captureName: Radar capture with tag covered with soil
%
% Outputs:
%   vwc: Calculated volumetric water content
%


close all

d = 0.12; % in meters
localDataPath = "C:/jlab/wadar/matlab/data/";
tagHz = 80;
airCaptureFile = airFileName;

[freqTag, freqTagHar] = calculateTagFrequencies(tagHz, 200, 20000); 

%% Load Air Capture %%
[airRawFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath, airCaptureFile));
airFrameCount = size(airRawFrames, 2);
airFramesBB = zeros(size(airRawFrames));

for j = 1:airFrameCount
    airFramesBB(:,j) = NoveldaDDC(airRawFrames(:,j), chipSet, pgen, fs_hz);
end

% Find the bin corresponding to the largest peak 
airFT = fft(airFramesBB, 20000 , 2); 
airTagFT = abs(airFT(:, freqTag)); 
[vals, peaks] = findpeaks(airTagFT, 'MinPeakHeight', max(airTagFT) * 0.9);
airPeakBin = peaks(1);

%% Load soil capture %%
[frames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath,captureName));
% Baseband Conversion
frameCount = size(frames, 2);
framesBB = zeros(size(frames));
for i = 1:frameCount
    framesBB(:,i) = NoveldaDDC(frames(:,i), chipSet, pgen, fs_hz);
end

% FFT of signal for each bin
[freqTag, freqTagHar] = calculateTagFrequencies(tagHz, 200, 20000);
ft = fft(framesBB(:,1:frameCount),frameCount,2);
TagFT = abs(ft(:, freqTag));
[vals, peaks] = findpeaks(TagFT, 'MinPeakHeight', max(TagFT) * 0.90);
peak = peaks(1);

vwc = calculateSoilMoisture(airPeakBin, peak, "farm", d);

plot(TagFT)
xlabel('Range Bins')
ylabel('Magnitude')
title("80 Hz Isolated");

fprintf("Air peak located at %d\n", airPeakBin);
fprintf("Capture peak located at %d\n", peak);
fprintf("VWC = %f\n", vwc);

end