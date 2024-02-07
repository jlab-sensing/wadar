function [procSuccess, captureFT, tagFT, peakBin, SNRdB] = procRadarFrames(localDataPath, captureName)
% [procResult, captureFT, tagFT, peakBin, SNRdB] = procCapture(localDataPath, captureName)
%
% Function processes radar frames for various purposes
%
% Inputs:
%       localDataPath: Path of radar frames storage
%       captureName: Name of radar frames file
%
% Outputs:
%       procResult: 'true' if the function loads the capture succesfully
%       captureFT: The FT of the radar frames
%       tagFT: The FT of the bin where the selected frequency peaks
%       peakBin: The bin where the selected frequency peaks
%       SNRdB: SNR of capture at frequency peak in dB

%% Processing parameters
tagHz = 80;
frameRate = 200;  

%% Load Capture
try
    [rawFrames, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, captureName));
catch
    procSuccess = false;
    captureFT = -1;
    tagFT = -1;
    peakBin = -1;
    SNRdB = -1;
    return
end

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

SNR = calculateSNR(captureFT, freqTag, peakBin);
SNRdB = 10 * log10(SNR);

procSuccess = true;

% %% Display Radar Frames
% figure(1)
% plot(tagFT)
% xline(peakBin)
% xlabel('Range Bins')
% ylabel('Magnitude')
% title("Capture - 80 Hz Isolated");

end