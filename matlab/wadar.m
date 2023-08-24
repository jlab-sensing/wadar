function vwc = wadar(templateFileName, airFileName, captureName)
% wadar(templateFileName, airFileName, captureName)
%
% Script calculates the volumetric water content (vwc) of the soil from 
% the capture
%
% Inputs:
%   templateFileName: 
%        airFileName: 
%        captureName:
%
% Outputs:
%   vwc: 
%


close all

d = 0.10; % in meters
method = "corr"; %"leftMost"
confidenceMethod = "joint"; 
localDataPath = "/Users/cjoseph/wadar/matlab/data";
fullDataPath = sprintf("cjoseph@192.168.7.1:%s",localDataPath);
radarType = "Chipotle";
numTrials = 2000;
frameRate = 200; 
tagHz = 80;
SNR = 0;
confidence = 0;
corrTemplateFile = templateFileName;
airCaptureFile = airFileName;
captureName = captureName; 

%% Load Correlation Template %%
[tempRawFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath, corrTemplateFile));
tempFrameCount = size(tempRawFrames, 2);
tempFramesBB = zeros(size(tempRawFrames));

for j = 1:tempFrameCount
    tempFramesBB(:,j) = NoveldaDDC(tempRawFrames(:,j), chipSet, pgen, fs_hz);
end

tempFT = fft(tempFramesBB(:,1:numTrials),numTrials,2); 

% choose the frequency based on the capture duration 
[freqTag, freqTagHar] = calculateTagFrequencies(tagHz, frameRate, numTrials); 

%TODO: improve this?
tempTagFT = abs(tempFT(:, freqTag)); 
% find the bin corresponding to the largest peak 
[val, binMax] = max(tempTagFT);
%templatePeakBin = [templatePeakBins binMax]; 
% find left-most peak matching criteria 
h1 = mean(findpeaks(tempTagFT(1:100)));
h2 = max(tempTagFT); 
thresholdAdjust = 0.9; % factor for adjusting which peaks are considered valid
threshold = thresholdAdjust * (h1 + h2) / 2; 
%threshold = 0.85 * max(tempTagFT);  
[peaks peakBins] = findpeaks(tempTagFT, 'MinPeakHeight', threshold); 
peakBins = peakBins(peakBins > 22); % assume no peak in first 22
templatePeakBin = peakBins(1);

%% loading air capture %%
[airRawFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath, airCaptureFile));
airFrameCount = size(airRawFrames, 2);
airFramesBB = zeros(size(airRawFrames));

for j = 1:airFrameCount
    airFramesBB(:,j) = NoveldaDDC(airRawFrames(:,j), chipSet, pgen, fs_hz);
end

airFT = fft(airFramesBB,numTrials,2); 

%TODO: improve this?
airTagFT = abs(airFT(:, freqTag)); 
% find the bin corresponding to the largest peak 
[val, binMax] = max(airTagFT);
%templatePeakBins = [templatePeakBins binMax]; 
% find left-most peak matching criteria 
h1 = mean(findpeaks(airTagFT(1:100)));
h2 = max(airTagFT); 
thresholdAdjust = 0.9; % factor for adjusting which peaks are considered valid
threshold = thresholdAdjust * (h1 + h2) / 2; 
%threshold = 0.85 * max(tempTagFT);  
[peaks peakBins] = findpeaks(airTagFT, 'MinPeakHeight', threshold); 
peakBins = peakBins(peakBins > 22); % assume no peak in first 22x
airPeakBin = peakBins(1);

%% loading soil capture %%
[frames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath,captureName));
% Baseband Conversion
frameCount = size(frames, 2);
framesBB = zeros(size(frames));
for i = 1:frameCount
    framesBB(:,i) = NoveldaDDC(frames(:,i), chipSet, pgen, fs_hz);
end

% FFT of signal for each bin
ft = fft(framesBB(:,1:frameCount),frameCount,2);
[peak confidence ftProcessed shiftedTemp SNR] = determinePeak(tempTagFT,templatePeakBin, ft, frameRate, tagHz, method, confidenceMethod); 
vwc = calculateSoilMoisture(airPeakBin, peak, "farm", d);

end