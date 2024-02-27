% close all

%% Simulation Parameters
frameRate = 200; % frames per sec
adjustment = -40; % How many bins to adjust peak location

% Simulated tag parameters
tag1Frequency = 80; % Frequency of tag 1 in Hz
tag2Frequency = 79; % Frequency of tag 2 in Hz
tag1SNR = 20; % Signal-to-Noise Ratio (SNR) for tag 1
tag2SNR = 18; % Signal-to-Noise Ratio (SNR) for tag 2

% Number of frames to simulate
frameCount = 2000;

%% Simulate Radar Frames for Tag 1 and Tag 2
framesTag1 = simulateRadarFrames(tag1Frequency, tag1SNR, frameCount, 100);
framesTag2 = simulateRadarFrames(tag2Frequency, tag2SNR, frameCount, 150);

framesCombined = framesTag1 + framesTag2;

figure(1)
scanTimeSteps = 512;
c = 299792458.0; % speed of light

% TODO - fix resolution
resolution = 0.5 * c / 3.6046e+10;
range = linspace(0,512,512);

freqTag1 = tag1Frequency / frameRate * frameCount;
captureFT = fft(framesCombined, frameCount , 2); 
tag1FT = abs(captureFT(:, freqTag1));
for i = (freqTag1-2:1:freqTag1+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tag1FT)
        tag1FT = temp;
    end
end
tag1FT = smoothdata(tag1FT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(tag1FT, 'MinPeakHeight', max(tag1FT) * 0.9);
if (size(peaks, 1) > 1)
    if (peaks(2) - peaks(1) < 50)
        peakBin1 = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((tag1FT(peaks(2)) - ...
            tag1FT(peaks(1))) / max(tag1FT) * (peaks(2) - peaks(1)));
    else
        peakBin1 = peaks(1);
    end
elseif (size(peaks, 1) == 1)
    peakBin1 = peaks(1);
else
    peakBin1 = -1;
end

freqTag2 = tag2Frequency / frameRate * frameCount;
tag2FT = abs(captureFT(:, freqTag2));
for i = (freqTag2-2:1:freqTag2+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tag2FT)
        tag2FT = temp;
    end
end
tag2FT = smoothdata(tag2FT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(tag2FT, 'MinPeakHeight', max(tag2FT) * 0.9);
if (size(peaks, 1) > 1)
    if (peaks(2) - peaks(1) < 50)
        peakBin2 = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((tag2FT(peaks(2)) - ...
            tag2FT(peaks(1))) / max(tag2FT) * (peaks(2) - peaks(1)));
    else
        peakBin2 = peaks(1);
    end
elseif (size(peaks, 1) == 1)
    peakBin2 = peaks(1);
else
    peakBin2 = -1;
end

figure(10)
captureFT(:, 1:2) = ones(512, 2); % first 2 frames of capture is extremely noisy
x = (1:1:512)';
y = (1:1:frameCount) / frameCount * frameRate;
xMat = repmat(x, 1, length(y));
yMat = repmat(y, length(x), 1);
zMat = abs(captureFT(:, 1:frameCount));
plot3(xMat, yMat, zMat)

% %% Output SNR Statistics
% disp("SNR Statistics for Combined Frames:");
% disp("SNR (80Hz Tag):");
% disp(["Minimum", "Q1", "Median", "Q3", "Maximum"]);
% disp([min(snrBoth80), quantile(snrBoth80, 0.25), median(snrBoth80), quantile(snrBoth80, 0.75), max(snrBoth80)]);
% 
% disp("SNR (79Hz Tag):");
% disp(["Minimum", "Q1", "Median", "Q3", "Maximum"]);
% disp([min(snrBoth79), quantile(snrBoth79, 0.25), median(snrBoth79), quantile(snrBoth79, 0.75), max(snrBoth79)]);


function frames = simulateRadarFrames(tagFrequency, snr, numFrames, rangeBin)
    % Simulate radar frames for a single tag with given frequency and SNR
    % Each frame is represented by a column vector

    % Simulated radar parameters
    frameLength = 512; % Length of each frame (512 range bins)
    noisePower = 100; % Noise power

    % Frequency resolution
    signal = zeros(frameLength, numFrames);
    t = (0:numFrames-1) / 200;
    for i = rangeBin-30:rangeBin+50
        if i == rangeBin
            signalStrength = 1;
        else
            signalStrength = 1 - abs(i - rangeBin)/50;
        end
        signal(i, :) = signalStrength * (sqrt(10^(snr/10)) * sin(2*pi*tagFrequency*t')) + 10 * randn(numFrames, 1); % Set magnitude at the tag frequency according to SNR
    end

    % Simulate frames
    frames = zeros(frameLength, numFrames);
    for i = 1:numFrames
        % Generate noise
        noise = sqrt(noisePower) * randn(frameLength, 1);
        % Combine signal and noise
        frames(:, i) = signal(:, i) + noise;
    end
end