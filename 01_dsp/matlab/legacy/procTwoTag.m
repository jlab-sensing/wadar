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

% %% Processing parameters
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
% captureFT = fft(framesBB, frameCount , 2); 
captureFT = fft(framesBB(:,1:frameCount),frameCount,2);

freq1Tag = tag1Hz / frameRate * frameCount;
tag1FT = abs(captureFT(:, freq1Tag));
for i = (freq1Tag-2:1:freq1Tag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tag1FT)
        tag1FT = temp;
    end
end
% tag1FT = smoothdata(tag1FT, 'movmean', 10);

freq1TagHar = (frameRate - tag1Hz) / frameRate * frameCount;
tag1FTHar = abs(captureFT(:, freq1TagHar));
for i = (freq1TagHar-2:1:freq1TagHar+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tag1FTHar)
        tag1FTHar = temp;
    end
end
% tag1FTHar = smoothdata(tag1FT, 'movmean', 10);

tag1FT = tag1FT;


freq2Tag = tag2Hz / frameRate * frameCount;
tag2FT = abs(captureFT(:, freq2Tag));
for i = (freq2Tag-2:1:freq2Tag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tag2FT)
        tag2FT = temp;
    end
end
% tag2FT = smoothdata(tag2FT, 'movmean', 10);

freq2TagHar = (frameRate - tag2Hz) / frameRate * frameCount;
tag2FTHar = abs(captureFT(:, freq2TagHar));
for i = (freq2TagHar-2:1:freq2TagHar+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tag2FTHar)
        tag2FTHar = temp;
    end
end
% tag2FTHar = smoothdata(tag2FT, 'movmean', 10);

tag2FT = tag2FT;

peakBins(1) = procCaptureCWT(tag1FT, true);
peakBins(2) = procCaptureCWT(tag2FT, false);

diffBins = abs(peakBins(1) - peakBins(2));
tag1Peak = peakBins(1);
tag2Peak = peakBins(2);

tag1SNR = 0;
tag2SNR = 0;

%% Display Results
if (resultFlag == true) 
    
    close all

    figure(1)
    plot(tag2FT)
    hold on
    plot(tag2FT)
    scatter(peakBins(1), tag1FT(peakBins(1)), "x")
    scatter(peakBins(2), tag2FT(peakBins(2)), "x")
    legend(sprintf("%d Hz @ %d", tag1Hz, peakBins(1)), sprintf("%d Hz @ %d", tag2Hz, peakBins(2)))
    title(sprintf("%s FT", captureName))
    xlabel("Range Bins")
    ylabel("Normalized Magnitude")
    % title('Tag FT with optimal antenna alignment')
    
    figure(2)

    captureFT(:, 1:2) = ones(512, 2); % first 2 frames of capture are extremely noisy
    % processedFrames = frameCount/100;
    % captureFT(:, 1:processedFrames) = ones(512, processedFrames); 
    % captureFT(:, frameCount-processedFrames+1:frameCount) = ones(512, processedFrames); 
    x = (1:512)';
    y = (1:frameCount) / frameCount * frameRate;
    [xMat, yMat] = meshgrid(x, y);
    zMat = abs(captureFT(:, 1:frameCount))';
    surf(xMat, yMat, zMat, 'EdgeColor', 'none');
    ax = gca; % Get current axes
    ax.FontSize = 18; % Set the desired font size for the ticks
    xlim([0 512])


    xlabel('Range Bins', 'FontSize', 22)
    ylabel('Frequency', 'FontSize', 22)
    zlabel('Magnitude', 'FontSize', 22)
    title(strcat("Capture", " - FT of all peak bins"), 'FontSize', 30);
end

end