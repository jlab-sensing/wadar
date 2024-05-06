function procCaptureCWT(localDataPath, captureName, tag1Hz)

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

gapThreshold = 3;

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
        ridgeLine = [scale_idx locMax(i)];
        for scale_idx_2 = scale_idx-1:-1:1
            slidingWindowThreshold = max(1,ceil(0.1*i));
            nextScaleLocMax = cell2mat(localMaximums{scale_idx_2});
            nextScaleLocMax = nextScaleLocMax(2:end);
            nextScaleDiffs = abs(nextScaleLocMax - locMax(i));
            [~, closestIdx] = min(nextScaleDiffs);
            if (nextScaleDiffs(closestIdx) < slidingWindowThreshold)
                ridgeLine(end+1,:) = [scale_idx_2, nextScaleLocMax(closestIdx)];
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
    
% identify

% 
% for scale_idx = N-1:-1:1
%     cwtcoeffsAtJ = cwtcoeffs(scale_idx, :);
%     ridgeLines = [];
%     for i = 1:length(ridgeLineList)
%         currRidgeLine = ridgeLinesList{i};
%         if currRidgeLine(end:1)
%     end
% 
%     for i = 1:length(ridgeLinesList)
%         currRidgeLine = ridgeLinesList{i};
%         ridgeLines = currRidgeLine(end,2);
%         gapNum = 0;
% 
%         for j = scale_idx:-1:1
%             cwtcoeffsAtJ = cwtcoeffs(j, :);
% 
%             % find the closest max within the sliding window at next cwt scale
%             nearestLocMax = [];
%             slidingWindowThreshold = max(1,ceil(0.1*j));
%             point_idx = ridgeLines(i);
%             left_idx = max(1, point_idx - slidingWindowThreshold);
%             right_idx = min(length(cwtcoeffsAtJ), point_idx + slidingWindowThreshold);
%             [~, locMax] = findpeaks(abs(cwtcoeffsAtJ(left_idx:right_idx)));
%             nearestLocMax = [nearestLocMax, locMax + left_idx - 1];
% 
%             if length(nearestLocMax) ~= 0
%                 ridgeLinesIden{end+1} = nearestLocMax;
%                 ridgeLines = nearestLocMax;
%                 gapNum = 0;
%                 break; % because max is found
%             else
%                 ridgeLinesIden{end+1} = [];
%                 gapNum = gapNum + 1; % because no max is found
%             end
% 
%             if gapNum > gapThreshold
%                 removed(end+1) = ridgeLines(i);
%             end
%         end
% 
%     end
% end
% 
% 

figure(1);
plot(tag1FT);
hold on;
title('Ridge Lines After Step 3');
xlabel('Sample Index');
ylabel('Magnitude');
legend('Signal', 'Ridge Lines');

figure(2)
hold on;
xlim([0 512])
for i = length(ridgeLines):-1:1
    temp = ridgeLines{i};
    scatter(temp(:,2), temp(:,1))
    % temp(:,2)
end

end