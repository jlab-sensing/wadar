function procTagTest(localDataPath, captureName)
% procTagTest(localDataPath, captureName)
%
% Function processes radar frames for various purposes
%
% Inputs:
%       localDataPath: Path of radar frames storage
%       captureName: Name of radar frames file
%
% Outputs:
%       

close all

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
% frameCount = 5
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

peakMagnitudes = tagFT(peakBin);

%% Display Results
fprintf("\n%s Testing Results\n\n", captureName)

fprintf("SNR Results:\n")
fprintf("%fdB\n\n", SNRdB)


fprintf("Peak Magnitude Results:\n")
fprintf("%f\n\n", peakMagnitudes)

close all

figure(1)
plot(tagFT)
xline(peakBin)
xlabel('Range Bins')
ylabel('Magnitude')
title(strcat("Capture", " - 80 Hz Isolated"));

figure(2)
% hold on
% for j = 2:1:frameCount
%     plot(abs(captureFT(:, j)))
% end
captureFT(:, 1:2) = ones(512, 2); % first 2 frames of capture is extremely noisy
x = (1:1:512)';
y = (1:1:frameCount) / frameCount * frameRate;
xMat = repmat(x, 1, length(y));
yMat = repmat(y, length(x), 1);
zMat = abs(captureFT(:, 1:frameCount));
plot3(xMat, yMat, zMat)

xlabel('Range Bins')
ylabel('Frequency')
zlabel('Magnitude')
title(strcat("Capture", " - FT of all peak bins"));

end