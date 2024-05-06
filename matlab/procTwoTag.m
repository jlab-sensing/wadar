function [procResult, captureFT, diffBins, tag1FT, tag2FT, tag1Peak, tag2Peak, tag1SNR, tag2SNR] = procTwoTag(localDataPath, captureName, tag1Hz, tag2Hz, resultFlag)
% procTagTest(localDataPath, captureName)
%
% Function processes radar frames for various purposes
%
% Inputs:
%       localDataPath: Path of radar frames storage
%       captureName: Name of radar frames file
%       tag1Hz: 
%       tag2Hz: 
%       resultFlag: 
%
% Outputs:
%       

%% Processing parameters
frameRate = 200;  

%% Load Capture
try
    [rawFrames, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, captureName));
catch
    procResult = false;
end
procResult = true;

% Baseband Conversion
frameCount = size(rawFrames, 2);
% frameCount = 5
framesBB = zeros(size(rawFrames));
for j = 1:frameCount
    framesBB(:,j) = NoveldaDDC(rawFrames(:,j), chipSet, pgen, fs_hz);
end


% Find Tag FT
captureFT = fft(framesBB, frameCount , 2); 

freq1Tag = tag1Hz / frameRate * frameCount;
tag1FT = abs(captureFT(:, freq1Tag));
for i = (freq1Tag-2:1:freq1Tag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tag1FT)
        tag1FT = temp;
    end
end
% tag1FT = smoothdata(tag1FT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(tag1FT, 'MinPeakHeight', max(tag1FT) * 0.9);
if (size(peaks, 1) > 1)
    if (peaks(2) - peaks(1) < 50)
        peakBin = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((tag1FT(peaks(2)) - ...
            tag1FT(peaks(1))) / max(tag1FT) * (peaks(2) - peaks(1)));
    else
        peakBin = peaks(1);
    end
elseif (size(peaks, 1) == 1)
    peakBin = peaks(1);
else
    peakBin = -1;
end

peakBins(1) = peakBin;

SNR = calculateSNR(captureFT, freq1Tag, peakBin);
SNRdB(1) = 10 * log10(SNR);

peakMagnitudes(1) = tag1FT(peakBin);

freq2Tag = tag2Hz / frameRate * frameCount;
tag2FT = abs(captureFT(:, freq2Tag));
for i = (freq2Tag-2:1:freq2Tag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tag2FT)
        tag2FT = temp;
    end
end
% tag2FT = smoothdata(tag2FT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(tag2FT, 'MinPeakHeight', max(tag2FT) * 0.9);
if (size(peaks, 1) > 1)
    if (peaks(2) - peaks(1) < 50)
        peakBin = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((tag2FT(peaks(2)) - ...
            tag2FT(peaks(1))) / max(tag2FT) * (peaks(2) - peaks(1)));
    else
        peakBin = peaks(1);
    end
elseif (size(peaks, 1) == 1)
    peakBin = peaks(1);
else
    peakBin = -1;
end

peakBins(2) = peakBin;

SNR = calculateSNR(captureFT, freq2Tag, peakBin);
SNRdB(2) = 10 * log10(SNR);

peakMagnitudes(2) = tag2FT(peakBin);

diffBins = abs(peakBins(2) - peakBins(1));
tag1Peak = peakBins(1);
tag2Peak = peakBins(2);
tag1SNR = SNRdB(1);
tag2SNR = SNRdB(2); % TODO, fix SNR for dual tags

%% Display Results
if (resultFlag == true) 
    
    % fprintf("\n%s Testing Results\n\n", captureName)
    % 
    % 
    % fprintf("Peak Magnitude Results:\n")
    % fprintf("%d Hz Tag: %f\n", tag1Hz, peakMagnitudes(1))
    % fprintf("%d Hz Tag: %f\n\n", tag2Hz, peakMagnitudes(2))
    % 
    % fprintf("Peak Results:\n")
    % fprintf("%d Hz Tag: %d\n", tag1Hz, peakBins(1))
    % fprintf("%d Hz Tag: %d\n\n", tag2Hz, peakBins(2))
    % 
    % fprintf("Peak Bin Difference:\n")
    % fprintf("|%d - %d| = %d", peakBins(1), peakBins(2), abs(peakBins(1) - peakBins(2)))
    
    close all
    figure(1)
    hold on;
    plot(tag1FT, 'displayname', sprintf('%i Hz FFT bin of tag 1',tag1Hz)); hold on;
    plot(tag2FT, 'DisplayName', sprintf('%i Hz FFT bin of tag 2',tag2Hz)); hold on; 
    plot(peakBins(1), tag1FT(peakBins(1)), 'o', 'DisplayName', sprintf('detected peak = %i', peakBins(1)));
    plot(peakBins(2), tag2FT(peakBins(2)), 'o', 'DisplayName', sprintf('detected peak = %i', peakBins(2)));
    legend(sprintf('%i Hz FFT bin of tag 1',tag1Hz), sprintf('%i Hz FFT bin of tag 2',tag2Hz))
    xlabel('Range Bins')
    ylabel('Magnitude')
    title(captureName);
    
    % figure(2)
    % % hold on
    % % for j = 2:1:frameCount
    % %     plot(abs(captureFT(:, j)))
    % % end
    % captureFT(:, 1:2) = ones(512, 2); % first 2 frames of capture is extremely noisy
    % x = (1:1:512)';
    % y = (1:1:frameCount) / frameCount * frameRate;
    % xMat = repmat(x, 1, length(y));
    % yMat = repmat(y, length(x), 1);
    % zMat = abs(captureFT(:, 1:frameCount));
    % plot3(xMat, yMat, zMat)
    % 
    % xlabel('Range Bins')
    % ylabel('Frequency')
    % zlabel('Magnitude')
    % title(strcat("Capture", " - FT of all peak bins"));
end

end