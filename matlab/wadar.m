function volumetricWaterContent = wadar(displayAllFrames, airFramesName, templateFramesName, localDataPath, trialName, captureCount, tagDepth)
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
frameCount = 200;
radarType = 'Chipotle';
fullDataPath = sprintf("ericdvet@192.168.7.1:%s",localDataPath);

% Processing parameters
soilType = 'farm';

%% Process Template Capture
[procResult, ~, templateTagFT, ~, ~] = procRadarFrames(localDataPath, templateFramesName);
if (procResult == false)
    error("ERROR: Template Frames Invalid")
end

%% Process Air Capture
[procResult, ~, airTagFT, airPeakBin, ~] = procRadarFrames(localDataPath, airFramesName);
if (procResult == false)
    error("ERROR: Air Frames Invalid")
end

%% Load soil capture %%

[year, month, date] = ymd(datetime("now"));

% File name generation
if isnumeric(trialName)
    trialName = num2str(trialName);
end

captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_WetUnder', num2str(tagDepth*1000), 'mm_T', num2str(trialName), '_C');

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
        if (toc > 20)
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
    [procSuccess, wetPeakBin, airPeakBin, SNRdB(i), wetTagFT] = procCaptureCorrelation(templateFramesName, airFramesName, wetFramesName, localDataPath);
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
        plot(normalize(templateTagFT, "range"))
        plot(normalize(airTagFT, "range"))
        xline(airPeakBin)
        xline(wetPeakBin)
        legend('Capture (80 Hz)', 'Template (80 Hz)', 'Air (80 Hz)')
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

% for i = 1:1:captureCount
%     try
%         [rawFrames, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, strcat(captureName, num2str(i), '.frames')));
%     catch
%         failedCaptures = [failedCaptures i];
%         continue
%     end
% 
%     % Baseband Conversion
%     frameCount = size(rawFrames, 2);
%     framesBB = zeros(size(rawFrames));
%     for j = 1:frameCount
%         framesBB(:,j) = NoveldaDDC(rawFrames(:,j), chipSet, pgen, fs_hz);
%     end
% 
%     % Find Tag FT
%     freqTag = tagHz / frameRate * frameCount;
%     captureFT = fft(framesBB, frameCount , 2); 
% 
%     % Find the bin corresponding to the largest peak 
%     TagFT = abs(captureFT(:, freqTag));
%     for j = (freqTag-2:1:freqTag+2)
%         temp = abs(captureFT(:, j));
%         if max(temp(1:airPeakBin, :)) > max(temp(airPeakBin:end, :)) % if greatest peak is before air peak, skip
%             continue
%         end
%         if max(temp(airPeakBin:end)) > max(TagFT(airPeakBin:end))
%             TagFT = temp;
%         end
%     end
%     TagFT = smoothdata(TagFT, 'movmean', 10);
% 
%     % TagHarFT = abs(captureFT(:, freqTagHar));
%     % for i = (freqTagHar-2:1:freqTagHar+2)
%     %     temp = abs(captureFT(:, i));
%     %     if max(temp(1:airPeakBin, :)) > max(temp(airPeakBin:end, :)) % if greatest peak is before air peak, skip
%     %         continue
%     %     end
%     %     if max(temp(airPeakBin:end)) > max(TagFT(airPeakBin:end))
%     %         TagHarFT = temp;
%     %     end
%     % end
% 
%     % Find the bin corresponding to the largest peak 
%     [~, peaks] = findpeaks(TagFT(airPeakBin:end), 'MinPeakHeight', max(TagFT(airPeakBin:end)) * 0.90);
%     peaks = peaks+airPeakBin;
% 
%     % Select peak bin greater than air peak
%     peak = peaks(1);
%     j = 0;
%     while peak < airPeakBin
%         if j > length(peaks) - 1
%             break;
%         end
%         j = i + 1;
%         peak = peaks(j);
%     end
% 
%     % Select peak bin correlated to template capture
%     % corrArray = zeros(length(peaks),2);
%     corrArray = zeros(1, 512);
%     for j = (1:1:512)
%         corrArray(j) = corr(normalize(circshift(templateTagFT, j)), normalize(TagFT), 'Type', 'Pearson');
%         % corrArray(j) = TagFT(j);
%     end
%     [~, peakIndex] = max(circshift(corrArray, templatePeakBin));
%     closestPeak = peaks(1);
%     for j = 1:size(peaks, 1)
%         if abs(peaks(j) - peakIndex) < closestPeak
%             closestPeak = peaks(j);
%         end
%     end
%     peakBin(i) = closestPeak;
% 
%     vwc(i) = calculateSoilMoisture(airPeakBin, closestPeak, "farm", tagDepth);
% 
%     SNR(i) = calculateSNR(captureFT, freqTag, peakBin(i));
%     SNRdB(i) = 10 * log10(SNR(i));
% end
% 
% % Remove failed captures
% for i = failedCaptures
%     SNRdB(i) = [];
%     peakMagnitudes(i) = [];
%     peakBin(i) = [];
% end

figure(1)
clf(1)
plot(normalize(wetTagFT, "range"))
hold on;
plot(normalize(templateTagFT, "range"))
plot(normalize(airTagFT, "range"))
xline(airPeakBin)
xline(wetPeakBin)
legend('Capture (80 Hz)', 'Template (80 Hz)', 'Air (80 Hz)')
xlabel('Range Bins')
ylabel('Magnitude')
title("80 Hz Isolated");

fprintf("Air peak located at %d\n", airPeakBin);
fprintf("Capture peak located at %d\n\n", wetPeakBin);

fprintf("\nTesting Results\n\n")

fprintf("VWC:\nMean = %f\nMedian = %f\n\n", mean(vwc), median(vwc));

fprintf("SNR:\n")
fprintf("Median: %fdB\nMean: %fdB\n\n", median(SNRdB), mean(SNRdB))


fprintf("Peak Magnitude Results:\n")
fprintf("Median: %f\nMean: %f\n\n", median(peakMagnitudes), mean(peakMagnitudes))

fprintf("Maximum difference between peak bins: %d\n\n", max(diff(sort(peakBin))))

volumetricWaterContent = median(vwc);


end