function [captureFT, tagFT, peakBin] = procRadarFrames(localDataPath, captureName)
% vwc = procCapture(localDataPath, captureName)
%
% Function processes radar frames
%
% Inputs:
%        
%
% Outputs:
%   captureSuccess: Flag to demonstrate if capture was successful

%% Processing parameters
tagHz = 80;
frameRate = 200;  

%% Load Capture
[rawFrames, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, captureName));

% Baseband Conversion
frameCount = size(rawFrames, 2);
framesBB = zeros(size(rawFrames));
for j = 1:frameCount
    framesBB(:,j) = NoveldaDDC(rawFrames(:,j), chipSet, pgen, fs_hz);
end

% Find Tag FT
freqTag = tagHz / frameRate * frameCount;
captureFT = fft(framesBB, frameCount , 2); 
tagFT = abs(captureFT(:, freqTag));
for i = (freqTag-2:1:freqTag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tagFT)
        tagFT = temp;
    end
end
tagFT = smoothdata(tagFT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(tagFT, 'MinPeakHeight', max(tagFT) * 0.9);
if (size(peaks, 1) > 1)
    if (peaks(2) - peaks(1) < 50)
        peakBin = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((tagFT(peaks(2)) - ...
            tagFT(peaks(1))) / max(tagFT) * (peaks(2) - peaks(1)));
    else
        peakBin = peaks(1);
    end
elseif (size(peaks, 1) == 1)
    peakBin = peaks(1);
else
    peakBin = -1;
end