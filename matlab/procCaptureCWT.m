function [tagPeakBin] = procCaptureCWT(localDataPath, captureName, tagHz)

clf

%% Control variables
gapThreshold = 5;
slidingWindowThreshold = 5;

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

% Pearson Correlation Method (for demo)

% Find the bin corresponding to the largest peak 
[~, corrPeakList] = findpeaks(tagFT, 'MinPeakHeight', max(tagFT) * 0.90);
corrPeakList = corrPeakList+0;
if (size(corrPeakList, 1) > 1)
    if (corrPeakList(2) - corrPeakList(1) < 50)  % if double peak, invalid radar capture
        procSuccess = false;
        fprintf("Double peak detected. Please reorient the radar.\n")
    end
end

% Select peak bin greater than air peak
peakBinCorr = corrPeakList(1);
j = 0;
while peakBinCorr < 0
    if j > length(corrPeakList) - 1
        break;
    end
    j = j + 1;
    peakBinCorr = corrPeakList(j);
end

% Select peak bin correlated to template capture
corrArray = zeros(1, 512);
for j = (1:1:512)
    shiftedtagFT = circshift(norm(tagFT), j);

    % Pearson Correlation
    meanTemplate = mean(norm(shiftedtagFT));
    meanWetTag = mean(norm(tagFT));

    corrArray(j) = sum((shiftedtagFT - meanTemplate) .* (norm(tagFT) - meanWetTag)) / sqrt(sum((shiftedtagFT - meanTemplate).^2) * sum((norm(tagFT) - meanWetTag).^2));
end
[~, peakIndex] = max(circshift(corrArray, peakBinCorr));
closestPeak = corrPeakList(1);
for j = 1:size(corrPeakList, 1)
    if abs(corrPeakList(j) - peakIndex) < closestPeak
        closestPeak = corrPeakList(j);
    end
end
peakBinCorrBin = closestPeak;

% Continuous Wavelet Transform
cwtCoeffs = cwt(tagFT, 'amor', 200);
[N, ~] = size(cwtCoeffs);

locMax = {};
ridgeLines = {};

for scale_idx = 1:N
    [~, currLocMax] = findpeaks(abs(cwtCoeffs(scale_idx, :)));
    locMax{end+1} =  {[scale_idx currLocMax]};
end


for scale_idx = N:-1:1
    currLocMax = cell2mat(locMax{scale_idx});
    for i = 2:length(currLocMax)
        currRidgeLine = [scale_idx currLocMax(i)];
        gap = 0;
        for scale_idx_2 = scale_idx-1:-1:1
            nextScaleLocs = cell2mat(locMax{scale_idx_2});
            nextScaleLocMax = nextScaleLocs(2:end);
            nextScaleDiffs = abs(nextScaleLocMax - currLocMax(i));
            % if currLocMax(i) == 219
            %     nextScaleLocs
            %     nextScaleLocs
            %     nextScaleDiffs
            % end
            [~, closest_idx] = min(nextScaleDiffs);
            if (nextScaleDiffs(closest_idx) <= slidingWindowThreshold)
                currRidgeLine(end+1,:) = [scale_idx_2, nextScaleLocMax(closest_idx)];
                nextScaleLocMax(closest_idx) = [];
                nextScaleLocs = [nextScaleLocs(1) nextScaleLocMax];
                locMax{scale_idx_2} = {nextScaleLocs}; % this is what removes duplicates
                gap = 0;
            else
                gap = gap + 1;
            end
            if gap > gapThreshold
                break
            end
        end
        ridgeLines{end+1} = currRidgeLine;
    end
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
peakLens = [];
coeffMaxes = [];
coeff_idx = [];
peakLocs = [];
for i = 1:length(revalidRidges)
    ridge = revalidRidges{i};
    [~, peakLoc] = max(max(abs(cwtCoeffs(ridge(:,1), ridge(:,2))))); % max coeff method
    % peakLoc = length(ridge(:,1));
    detectedPeaks(end+1) = ridge(peakLoc,2);
    peakLens(end+1) = length(ridge(:,1));
    coeffMaxes(end+1) = cwtCoeffs(ridge(peakLoc,1));
    coeff_idx(end+1) = peakLoc;
    peakLocs(end+1) = peakLoc;
end

validPeaks = [];
validPeakLens = [];
for i = 1:length(detectedPeaks)
    if tagFT(detectedPeaks(i)) > max(tagFT(detectedPeaks)) * 0.3
        validPeaks(end+1) = detectedPeaks(i);
        validPeakLens(end+1) = peakLens(i);
    end
end
detectedPeaks = validPeaks;
peakLens = validPeakLens;
[~, temp] = max(peakLens);

[~, peakBinCoeff] = max(abs(coeffMaxes));
[~, peakBinAmpl] = max(tagFT(detectedPeaks));
tagPeakBin = detectedPeaks(peakBinAmpl);
% tagPeakBin = detectedPeaks(temp);

figure(1);
subplot(2,1,1)
plot(tagFT);
hold on;
scatter(detectedPeaks, tagFT(detectedPeaks))
scatter(tagPeakBin, tagFT(tagPeakBin), 'filled')
scatter(peakBinCorrBin, tagFT(peakBinCorrBin), 'filled')
title('Tag Bin Isolated');
xlabel('Peak Bins');
ylabel('Magnitude');
legend('Tag FT', 'Ridges', sprintf('CWT Peak (%d)', tagPeakBin), sprintf('Pearson Correlation Peak (%d)', peakBinCorrBin));

subplot(2,1,2)
hold on;
xlim([0 600])
for i = length(revalidRidges):-1:1
    temp = revalidRidges{i};
    plot(temp(:,2), temp(:,1))
end

end