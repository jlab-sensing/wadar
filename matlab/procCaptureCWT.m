function [tagPeakBin] = procCaptureCWT(localDataPath, captureName, tagHz)

close all

%% Control variables
gapThreshold = 3;
slidingWindowThreshold = 1;

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

freqTag = tagHz / frameRate * frameCount;
tagFT = abs(captureFT(:, freqTag));
for i = (freqTag-2:1:freqTag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tagFT)
        tagFT = temp;
    end
end

% Continuous Wavelet Transform
cwtCoeffs = cwt(tagFT, 'amor', 200);
[N, ~] = size(cwtCoeffs);

locMax = {};
ridgeLines = {};

for scale_idx = 1:N
    [~, currLocMax] = findpeaks(abs(cwtCoeffs(scale_idx, :)));
    locMax{end+1} =  {[scale_idx currLocMax]};
end


gap = 0;
for scale_idx = N:-1:1
    currLocMax = cell2mat(locMax{scale_idx});
    for i = 2:length(currLocMax)
        currRidgeLine = [scale_idx currLocMax(i)];
        for scale_idx_2 = scale_idx-1:-1:1
            slidingWindow = max(1,ceil(slidingWindowThreshold*scale_idx_2));
            nextScaleLocMax = cell2mat(locMax{scale_idx_2});
            nextScaleLocMax = nextScaleLocMax(2:end);
            nextScaleDiffs = abs(nextScaleLocMax - currLocMax(i));
            [~, closest_idx] = min(nextScaleDiffs);
            if (nextScaleDiffs(closest_idx) < slidingWindow)
                currRidgeLine(end+1,:) = [scale_idx_2, nextScaleLocMax(closest_idx)];
                gap = 0;
            else
                gap = gap + 1;
            end
            if gap > gapThreshold
                break
            end
        end
    end
    ridgeLines{end+1} = currRidgeLine;
end
    
%% Identify the peaks based on the ridge lines
validRidges = ridgeLines;
duplicate_idx = 0;
for i = 1:length(ridgeLines)
    currRidgeLine = ridgeLines{i};
    duplicateFlag = 0;
    for j = 1:i-1
        ridgeLineDup = ridgeLines{j};
        if ismember(1, ismember(currRidgeLine(:,2), ridgeLineDup(:,2))) 
            for k = 1:length(currRidgeLine(:,1))
                ridgeLineDup(end+1,:) = currRidgeLine(k,:);
            end
            currRidgeLine = ridgeLineDup;
            validRidges{i} = sortrows(currRidgeLine);
            validRidges{j} = [];
        end
    end
    % if length(currRidgeLine(:,1)) >= 5
    %     if duplicateFlag == 0
    %         validRidges{end+1} = sortrows(currRidgeLine);
    %     end
    % end
end

validRidges = ridgeLines;
revalidRidges = {};
for i = 1:length(validRidges)
    currRidgeLine = validRidges{i};
    if length(currRidgeLine) > 0
        if length(currRidgeLine(:,1)) >= 5
            if duplicateFlag == 0
                revalidRidges{end+1} = sortrows(currRidgeLine);
            end
        end
    end
end


detectedPeaks = [];
coeffMaxes = [];
coeff_idx = [];
peakLocs = [];
for i = 1:length(revalidRidges)
    ridge = revalidRidges{i};
    % disp(ridge)
    [~, peakLoc] = max(max(abs(cwtCoeffs(ridge(:,1), ridge(:,2))))); % max coeff method
    % peakLoc = length(ridge(:,1));
    detectedPeaks(end+1) = ridge(peakLoc,2);
    coeffMaxes(end+1) = cwtCoeffs(ridge(peakLoc,1));
    coeff_idx(end+1) = peakLoc;
    peakLocs(end+1) = peakLoc;
end

[~, peakBinCoeff] = max(abs(coeffMaxes));
[~, peakBinAmpl] = max(tagFT(detectedPeaks));
tagPeakBin = detectedPeaks(peakBinAmpl);

figure(1);
subplot(2,1,1)
plot(tagFT);
hold on;
scatter(detectedPeaks, tagFT(detectedPeaks))
scatter(tagPeakBin, tagFT(tagPeakBin), 'filled')
title('Tag Bin Isolated');
xlabel('Peak Bins');
ylabel('Magnitude');
legend('Tag FT', 'Peaks');

subplot(2,1,2)
hold on;
xlim([0 600])
for i = length(revalidRidges):-1:1
    temp = revalidRidges{i};
    scatter(temp(:,2), temp(:,1))
end

end