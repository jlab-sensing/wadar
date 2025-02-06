function [tagPeakBin] = procCaptureCWT(tagFT, displayPlot)

clf

%% Control variables
gapThreshold = 5;
slidingWindowThreshold = 0.3; % 0.5;
SNRThreshold = 3;
scales = 1:2:64; %1:2:64;
ridgeLengthThreshold = 12;

if displayPlot == true
    figure(1)
    subplot(3,1,1)
    plot(tagFT)
    ax = gca; % Get current axes
    ax.FontSize = 18; % Set the desired font size for the ticks
    title("80 Hz FT Isolated", 'FontSize', 30)
    ylabel("Magnitude", 'FontSize', 22)
    xlabel("Range Bin", 'FontSize', 22)
end


%% Continuous Wavelet Transform
scales = [scales; 1:length(scales)];
scale_idxs = 1:length(scales);
cwtInfo = cwtft(tagFT, 'wavelet', 'mexh', 'scales', scales);
cwtCoeffs = cwtInfo.cfs;
amplThreshold = 0.1 * max(max(cwtCoeffs));

% Plot coeffs
if displayPlot == true
    subplot(3,1,2)
    for i = 1:height(cwtCoeffs)
        title("CWT Coefficients", 'FontSize', 30)
        ylabel("CWT Coefficients", 'FontSize', 22)
        xlabel("Range Bin", 'FontSize', 22)
        hold on;
        plot(cwtCoeffs(i,:))
        ax = gca; % Get current axes
        ax.FontSize = 18; % Set the desired font size for the ticks
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
            if closestLocMaxGap <= ceil(slidingWindowThreshold * next_scale_idx)
                ridgeLine(end+1, :) = [scale, closestLocMax];
                temp = locMax{next_scale_idx};
                temp(closest_locMax_idx + 1) = [];
                locMax{next_scale_idx} = temp;
                gap = 0;
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
if (displayPlot == true)
    for i = ridgeLines
        subplot(3,1,3)
        title("Identified Ridge Lines", 'FontSize', 30)
        ylabel("Scales", 'FontSize', 22)
        xlabel("Range Bin", 'FontSize', 22)
        hold on;
        temp = i{1};
        plot(temp(:, 2), temp(:, 1))
        ax = gca; % Get current axes
        ax.FontSize = 18; % Set the desired font size for the ticks
    end
end

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
        SNR = maxRidgeLineCWTCoeff / prctile(cwtCoeffs(1,:), 95);
        SNRdB = mag2db(SNR);
        % 2. SNR > threshold
        if SNRdB > SNRThreshold
            % 3. ridge length > threshold
            if length(ridge_line_idxs) > ridgeLengthThreshold
                % disp(ridgeLine)
                validRidgeLines{end+1} = ridgeLine;
            end
        end
    end

end

% if (displayPlot == true)
%     for i = validRidgeLines
%         subplot(3,1,2)
%         xlim([0 600])
%         hold on;
%         temp = i{1};
%         plot(temp(:, 2), temp(:, 1))   
%     end
% end

maxHeight = 0;
largest_ridge_idx = -1;
for i = 1:length(validRidgeLines)
    % disp(validRidgeLines{i})
    % disp(height(validRidgeLines{i}))
    if (height(validRidgeLines{i}) > maxHeight)
        maxHeight = height(validRidgeLines{i});
        largest_ridge_idx = i;
    end
end
% largest_ridge_idx = max(height(validRidgeLines));
largestRidge = validRidgeLines{largest_ridge_idx};

peakMethod1 = largestRidge(end,end);

largestRidgeCWTCoeffMaxes = [];
for i = 1:height(largestRidge)
    largestRidgeCWTCoeffMaxes(end+1) = cwtCoeffs(find(scale == largestRidge(i,1)), largestRidge(i,2));
end
[~, max_largest_ridge_cwt_coeff_idx] = max(largestRidgeCWTCoeffMaxes);

peakMethod2 = largestRidge(max_largest_ridge_cwt_coeff_idx,2);

maxHeight = 0;
largest_ridge_idx = -1;
for i = 1:length(validRidgeLines)
    ridgeLine = validRidgeLines{i};
    if (tagFT(ridgeLine(end, 2)) > maxHeight)
        maxHeight = tagFT(ridgeLine(end, 2));
        largest_ridge_idx = i;
    end
end
largestRidge = validRidgeLines{largest_ridge_idx};
peakMethod3 = largestRidge(end, 2);

if (displayPlot)
    subplot(3,1,1)
    hold on
    % scatter(peakMethod1, tagFT(peakMethod1), "x")
    % scatter(peakMethod2, tagFT(peakMethod2), "x")
    scatter(peakMethod3, tagFT(peakMethod3), "o")
    % legend("", sprintf("Detected Peak @ %d", peakMethod1), sprintf("My method @ %d", peakMethod3))
end

tagPeakBin = peakMethod3;

% cwtft2(tagFT,'wavelet','mexh','scales',1, 'angles',[0 pi/2]);
end