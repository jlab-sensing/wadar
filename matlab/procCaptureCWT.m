function [peakBin] = procCaptureCWT(localDataPath, captureName, tag1Hz)

close all

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

% Continuous Wavelet Transform
cwtcoeffs = cwt(tag1FT, 'amor', 200);

[~, locMax] = findpeaks(abs(cwtcoeffs(end, :)));
[N, ~] = size(cwtcoeffs);

ridgeLines = zeros(length(locMax), 1);
for i = 1:length(locMax)
    ridgeLines(i) = locMax(i);
end

gapThreshold = 5;

removed = [];
ridgeLinesScales = N:-1:1;
ridgeLinesIden = {ridgeLines};

localMaximums = {};
ridgeLinesIdx = 1;
ridgeLines = {};

for scale_idx = 1:N
    [~, locMax] = findpeaks(abs(cwtcoeffs(scale_idx, :)));
    localMaximums{end+1} =  {[scale_idx locMax]};
end

gap = 0;
for scale_idx = N:-1:1
    locMax = cell2mat(localMaximums{scale_idx});
    for i = 2:length(locMax)
        % TODO: Why am I struggling to remove duplicates :|
        % duplicateFlag = 0;
        % if length(ridgeLines) > 0
        %     for dup_idx = 1:length(ridgeLines)
        %         duplicate = ridgeLines{dup_idx};
        %         % duplicate
        %         % locMax(i)
        %         % ismember(locMax(i), duplicate(:,2))
        %         if ismember(locMax(i), duplicate(:,2))
        %             duplicateFlag = 1;
        %             locMax(i)
        %         end
        %     end
        % end
        % if (duplicateFlag == 1)
        %     break
        % end
        ridgeLine = [scale_idx locMax(i)];
        % disp(ridgeLine)
        for scale_idx_2 = scale_idx-1:-1:1
            slidingWindowThreshold = max(1,ceil(1*scale_idx_2));
            nextScaleLocMax = cell2mat(localMaximums{scale_idx_2});
            nextScaleLocMax = nextScaleLocMax(2:end);
            nextScaleDiffs = abs(nextScaleLocMax - locMax(i));
            [~, closestIdx] = min(nextScaleDiffs);
            if (nextScaleDiffs(closestIdx) < slidingWindowThreshold)
                ridgeLine(end+1,:) = [scale_idx_2, nextScaleLocMax(closestIdx)];
                % disp(ridgeLine)
                gap = 0;
            else
                gap = gap + 1;
            end
            if gap > gapThreshold
                break
            end
        end
    end
    ridgeLines{end+1} = ridgeLine;
end
    
%% identify the peaks based on the ridge lines
% trim ridge lines
validRidges = {};
for i = 1:length(ridgeLines)
    ridgeLine = ridgeLines{i};
    duplicateFlag = 0;
    for j = 1:i-1
        ridgeLineDup = ridgeLines{j};
        % ismember(ridgeLine(:,2), ridgeLineDup(:,2))
        if ismember(1, ismember(ridgeLine(:,2), ridgeLineDup(:,2))) 
            duplicateFlag = 1;
        end
    end
    if duplicateFlag == 0
        if length(ridgeLine(:,1)) >= 5
            validRidges{end+1} = ridgeLine;
        end
    end
end

% find peaks
peaks = [];
for i = 1:length(validRidges)
    ridge = validRidges{i};
    disp(ridge)
    [~, peakLoc] = max(max(abs(cwtcoeffs(ridge(:,1), ridge(:,2)))));
    for j = 1:height(ridge)
        if ridge(j,1) == peakLoc
            peaks(end+1) = ridge(j,2);
        end
    end
end
[~, peakBin] = max(tag1FT(peaks));

figure(1);
subplot(2,1,1)
plot(tag1FT);
hold on;
scatter(peaks, tag1FT(peaks))
title('Tag Bin Isolated');
xlabel('Peak Bins');
ylabel('Magnitude');
legend('Tag FT', 'Peaks');

subplot(2,1,2)
hold on;
xlim([0 600])
for i = length(validRidges):-1:1
    temp = validRidges{i};
    scatter(temp(:,2), temp(:,1))
    % temp(:,2)
end

end