function volumetricWaterContent = wadar(displayAllFrames, airFramesName, localDataPath, trialName, tagHz, frameCount, captureCount, tagDepth)
% volumetricWaterContent = wadar(airFramesName, templateFramesName, localDataPath, trialName, captureCount)
%
% Script calculates the volumetric water content (vwc) of the soil from 
% the capture
%
% Inputs:
%       displayAllFrames: If true, displays each capture's frames for 10 
%           seconds
%       airFramesName: Radar capture with tag uncovered with soil
%       templateFramesName: Radar capture with obvious tag peak
%       localDataPath: Location of data storage
%       trialName: Trial name for file naming purposes
%       captureCount: Number of captures desired
%
% Outputs:
%       volumetricWaterContent: Calculated volumetric water content
%


close all

% Capture parameters
frameRate = 200;   
radarType = 'Chipotle';

% Processing parameters
soilType = 'farm';

%% Process Air Capture
[procResult, ~, airTagFT, airPeakBin, ~] = procRadarFrames(localDataPath, airFramesName, tagHz);
if (procResult == false)
    error("ERROR: Air Frames Invalid")
end

%% Load soil capture %%

[year, month, date] = ymd(datetime("now"));

% File name generation
[~, hostname] = system('whoami');
hostname(end) = '';
fullDataPath = sprintf("%s@192.168.7.1:%s", hostname, localDataPath);
if isnumeric(trialName)
    trialName = num2str(trialName);
end

captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_', num2str(tagDepth*1000), 'mmDepth_', num2str(trialName), '_C');

% Check for existing files with the same name
existingFiles = dir(localDataPath);

for i = 1:length(existingFiles)
    for j = 1:1:10
        if strcmp(existingFiles(i).name, strcat(captureName, num2str(j), '.frames'))
            error("Files under this trial index already exist. Iterate the trial index.")
            return
        end
    end
end

frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
    captureName, frameCount, captureCount, frameRate, radarType, fullDataPath);
frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
    frameLoggerOptions);
[status,~] = system(frameLoggerCommand);

%% Load and Process Captures

% SNR = zeros(1, captureCount);
SNRdB = zeros(1, captureCount);
peakMagnitudes = zeros(1, captureCount);
peakBin = zeros(1, captureCount);
vwc = zeros(1, captureCount);
failedCaptures = [];

for i = 1:1:captureCount

    fprintf("Please wait. Capture %d is proceeding\n", i)
    pause(frameCount/frameRate);
    fprintf("Waiting for data to be transferred...\n")
    
    checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(i), '.frames')));
    checkmd5File = dir(fullfile(localDataPath, strcat(captureName, num2str(i), '.md5')));
    tic
    while (length(checkFile) ~= 1) || (length(checkmd5File) ~= 1)
        checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(i), '.frames')));
        checkmd5File = dir(fullfile(localDataPath, strcat(captureName, num2str(i), '.md5')));
        if (toc > 30)
            error('There is a data transfer issue. Please verify your capture settings and scp directory. Ensure that your are already scp into the radar')
        end
    end
    fileName = checkFile(1).name;
    md5Name = checkmd5File(1).name;
    
    md5command = sprintf('md5 %s', fullfile(localDataPath, fileName));
    [status, cmdout] = system(md5command);
    localchecksum = char(strsplit(cmdout));
    localchecksum = lower(strtrim(localchecksum(4,:)));
    localchecksum = deblank(localchecksum);
    
    md5checksum = fileread(fullfile(localDataPath, md5Name));
    md5checksum = char(strsplit(md5checksum));
    md5checksum = lower(md5checksum(1,:));
    md5checksum = deblank(localchecksum);
    
    if (~strcmp(localchecksum, md5checksum))
        fprintf('Failure on framelogger check.\n', runCount);
        fprintf('Local checksum is %s.\n', localchecksum);
        fprintf('BBB checksum is %s.\n', md5checksum);
        error('Uh oh. There has been an error in the file transfer. The md5 hashes do not match.\n')
    else
        fprintf('Framelogger captured frame succesfully!\n\n')
    end
    
    wetFramesName = strcat(captureName, num2str(i), '.frames');
    [procSuccess, captureFT, wetTagFT, wetPeakBin, SNRdB(i)] = procRadarFrames(localDataPath, wetFramesName, tagHz);
    if (procSuccess == false)
        failedCaptures = [failedCaptures i];
        fprintf("Capture %d will not be processed.\n", i)
        continue
    end
    
    peakBin(i) = wetPeakBin;
    peakMagnitudes(i) = wetTagFT(peakBin(i));

    vwc(i) = procSoilMoisture(wetPeakBin, airPeakBin, soilType, tagDepth);

    if (displayAllFrames == true)
        figure(1)
        clf(1)
        plot(normalize(wetTagFT, "range"))
        hold on;
        plot(normalize(airTagFT, "range"))
        xline(airPeakBin)
        xline(wetPeakBin)
        legend('Capture (80 Hz)', 'Air (80 Hz)')
        xlabel('Range Bins')
        ylabel('Magnitude')
        title("VWC = ", num2str(vwc(i)));
    end

end

% Remove failed captures
for i = failedCaptures
    SNRdB(i) = [];
    peakMagnitudes(i) = [];
    peakBin(i) = [];
end

figure(1)
clf(1)
plot(normalize(wetTagFT, "range"))
hold on;
plot(normalize(airTagFT, "range"))
xline(airPeakBin)
xline(wetPeakBin)
legend('Capture (80 Hz)', 'Air (80 Hz)')
xlabel('Range Bins')
ylabel('Magnitude')
title("80 Hz Isolated");

fprintf("\nTesting Results\n\n")

trialNumbers = 1:captureCount;
results = table(trialNumbers, peakMagnitudes, SNRdB, peakBin, vwc);
disp(results)

fprintf("VWC:\nMean = %f\nMedian = %f\n\n", mean(vwc), median(vwc));

fprintf("SNR:\n")
fprintf("Median: %fdB\nMean: %fdB\n\n", median(SNRdB), mean(SNRdB))


fprintf("Peak Magnitude Results:\n")
fprintf("Median: %f\nMean: %f\n\n", median(peakMagnitudes), mean(peakMagnitudes))

fprintf("Maximum difference between peak bins: %d\n\n", max(diff(sort(peakBin))))

volumetricWaterContent = median(vwc);

figure(3)
% hold on
% for j = 2:1:frameCount
%     plot(abs(captureFT(:, j)))
% end
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
% ax.FontSize = 18; % Set the desired font size for the ticks
xlim([0 512])

xlabel('Range Bins')
ylabel('Frequency')
zlabel('Magnitude')
title(strcat("Capture", num2str(i), " - FT bins"));

end