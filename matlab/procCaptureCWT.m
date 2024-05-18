function [tagPeakBin] = procCaptureCWT2(localDataPath, captureName, tagHz, displayPlot)

clf

%% Control variables
gapThreshold = 5;
slidingWindowThreshold = 10;
SNRThreshold = 1;
ridgeLengthThreshold = 24;

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

if displayPlot == true
    figure(1)
    subplot(3,1,1)
    plot(tagFT)
end


%% Continuous Wavelet Transform
scales = 1:2:64;
scales = [scales; 1:length(scales)];
scale_idxs = 1:length(scales);
cwtInfo = cwtft(tagFT, 'wavelet', 'mexh', 'scales', 1:2:64);
cwtCoeffs = cwtInfo.cfs;
amplThreshold = 0.1 * max(max(cwtCoeffs));

% Plot coeffs
if displayPlot == true
    for i = 1:height(cwtCoeffs)
        subplot(3,1,3)
        hold on;
        plot(cwtCoeffs(i,:))
    end
end

% Find local maximums in each cwt
locMax = {};
for i = scales
    scale = i(1);
    scale_idx = i(2);
    [~, currLocMax] = findpeaks(abs(cwtCoeffs(scale_idx, :)));
    locMax{end+1} =  [scale currLocMax];
end

% Find all ridge lines
ridgeLines = {};
gap = 0;
for scale_idx = flip(scale_idxs)
    localScaleMaxs = locMax{scale_idx};
    localScaleMaxs = localScaleMaxs(2:end);
    for ridgeLineTop = localScaleMaxs
        gap = 0;
        scale = scales(1,scale_idx);
        ridgeLine = [scale ridgeLineTop];
        for next_scale_idx = scale_idx-1:-1:1
            previousRidgeLine = ridgeLine(end);
            scale = scales(1,next_scale_idx);
            nextScaleMaxs = locMax{next_scale_idx};
            nextScaleMaxs = nextScaleMaxs(2:end);
            nextScaleGaps = abs(previousRidgeLine - nextScaleMaxs);
            [closestLocMaxGap, closest_locMax_idx] = min(nextScaleGaps);
            closestLocMax = nextScaleMaxs(closest_locMax_idx);
            if closestLocMaxGap < slidingWindowThreshold
                ridgeLine(end+1, :) = [scale, closestLocMax];
                temp = locMax{next_scale_idx};
                temp(closest_locMax_idx + 1) = [];
                locMax{next_scale_idx} = temp;
            else
                gap = gap + 1;
            end
            if gap > gapThreshold
                break
            end
        end
        ridgeLines{end + 1} = ridgeLine;
    end
end

% Plot ridges
% if (displayPlot == true)
%     for i = ridgeLines
%         subplot(3,1,2)
%         hold on;
%         temp = i{1};
%         plot(temp(:, 2), temp(:, 1))
%     end
% end

% Process ridges
validRidgeLines = {};
for i = ridgeLines
    ridgeLine = i{1};
    scale = scales(1,:);
    ridge_line_idxs = [];
    for j = 1:length(ridgeLine(:,1))
        ridge_line_idxs(end+1) = find(scale == ridgeLine(j,1));
    end
    % size(cwtCoeffs)
    ridgeLineCWTCoeffs = [];
    for j = 1:length(ridge_line_idxs)
        ridgeLineCWTCoeffs(end+1) = cwtCoeffs(ridge_line_idxs(j), ridgeLine(j, 2));
    end
    [maxRidgeLineCWTCoeff, max_ridge_line_cwt_coeff_idx] = max(ridgeLineCWTCoeffs);
    maxRidgeLineCWTCoeffScale = ridgeLine(max_ridge_line_cwt_coeff_idx, 2);

    % 1. scale with max amplitude > certain range
    if maxRidgeLineCWTCoeff > amplThreshold
        SNR = maxRidgeLineCWTCoeff / prctile(cwtCoeffs(max_ridge_line_cwt_coeff_idx,:), 95);
        SNRdB = mag2db(SNR);
        % 2. SNR > threshold
        if SNRdB > SNRThreshold
            % 3. ridge length > threshold
            if length(ridge_line_idxs) > ridgeLengthThreshold
                validRidgeLines{end+1} = ridgeLine;
            end
        end
    end
end

if (displayPlot == true)
    for i = validRidgeLines
        subplot(3,1,2)
        xlim([0 600])
        hold on;
        temp = i{1};
        plot(temp(:, 2), temp(:, 1))   
    end
end

largest_ridge_idx = max(length(validRidgeLines));
largestRidge = validRidgeLines{largest_ridge_idx};

peakMethod1 = largestRidge(end,end);

largestRidgeCWTCoeffMaxes = [];
for i = 1:height(largestRidge)
    largestRidgeCWTCoeffMaxes(end+1) = cwtCoeffs(find(scale == largestRidge(i,1)), largestRidge(i,2));
end
[~, max_largest_ridge_cwt_coeff_idx] = max(largestRidgeCWTCoeffMaxes);

peakMethod2 = largestRidge(max_largest_ridge_cwt_coeff_idx,2);

subplot(3,1,1)
hold on
scatter(peakMethod1, tagFT(peakMethod1), "x")
scatter(peakMethod2, tagFT(peakMethod2), "x")
legend("", sprintf("Method 1 (%d)", peakMethod1), sprintf("Method 2 (%d)", peakMethod2))

tagPeakBin = peakMethod2;

% cwtft2(tagFT,'wavelet','mexh','scales',1, 'angles',[0 pi/2]);
end