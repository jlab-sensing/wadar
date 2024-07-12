function wadarDualTag(displayAllFrames, localDataPath, trialName, captureCount, tag1Hz, tag2Hz)
% wadarDualTag(localDataPath, tagName, trialName, captureCount)
%
% Function captures radar frames to test tag SNR and display capture
% fourier transform
%
% Inputs:
%       displayAllFrames: If true, displays each capture's frames for 10 
%           seconds
%       localDataPath: Location to store capture on local machine
%       tagName: Name of tag for file naming purposes
%       trialName: Trial name for file naming purposes
%       captureCount: Number of captures desired
%
% Outputs:
%       None

close all;

% Capture parameters
frameRate = 200;   
frameCount = 20000;
radarType = 'Chipotle';
fullDataPath = sprintf("ericdvet@192.168.7.1:%s",localDataPath);

% File name generation
if isnumeric(trialName)
    trialName = num2str(trialName);
end
[year, month, date] = ymd(datetime("now"));
% captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_', 'DualTag', num2str(tag1Hz), num2str(tag2Hz), '_', trialName, '_C');
captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_', '', num2str(tag1Hz), num2str(tag2Hz), '_', trialName, '_C');

% Check for existing files with the same name to prevent overwrite
existingFiles = dir(localDataPath);

% for i = 1:length(existingFiles)
%     for j = 1:1:10
%         if strcmp(existingFiles(i).name, strcat(captureName, num2str(j), '.frames'))
%             error("Files under this trial index already exist. Iterate the trial index.")
%             return
%         end
%     end
% end

%% Commit Radar Capture
    
% Send Frame Logger command with appropriate parameters
frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
    captureName, frameCount, captureCount, frameRate, radarType, fullDataPath);
frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
    frameLoggerOptions);
[status,~] = system(frameLoggerCommand);

%% Process Capture Frames
SNRdB1 = zeros(1, captureCount);
SNRdB2 = zeros(1, captureCount);
peakMagnitudes1 = zeros(1, captureCount);
peakMagnitudes2 = zeros(1, captureCount);
peakBin1 = zeros(1, captureCount);
peakBin2 = zeros(1, captureCount);
diffBins = zeros(1, captureCount);
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
        if (toc > 60)
            error('There is a data transfer issue. Please verify your capture settings and scp directory. Ensure that your are already scp into the radar')
        end
    end
    fileName = checkFile(1).name;
    md5Name = checkmd5File(1).name;

    % Verify that the md5 file checks out
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
        fprintf('Framelogger captured frames succesfully!\n\n')
    end

    % [procResult, captureFT, tagFT, peakBin(i), SNRdB(i)] = procRadarFrames(localDataPath, strcat(captureName, num2str(i), '.frames'));
    [procResult, captureFT, diffBins(i), tag1FT, tag2FT, peakBin1(i), peakBin2(i), SNRdB1(i), SNRdB2(i)] = procTwoTag(localDataPath, strcat(captureName, num2str(i), '.frames'), tag1Hz, tag2Hz, displayAllFrames);

    peakMagnitudes1(i) = tag1FT(peakBin1(i));
    peakMagnitudes2(i) = tag2FT(peakBin2(i));

    if (procResult == false)
        failedCaptures = [failedCaptures i];
        fprintf("Capture %d faced a processing issue.\n", i)
        peakBin1(i) = 0;
        SNRdB1(i) = 0;
        peakMagnitudes1(i) = 0;
        peakBin2(i) = 0;
        SNRdB2(i) = 0;
        peakMagnitudes2(i) = 0;
        continue
    end

end

% Remove failed captures
for j = failedCaptures
    SNRdB1(j) = [];
    peakMagnitudes1(j) = [];
    peakBin1(j) = [];
    SNRdB2(j) = [];
    peakMagnitudes2(j) = [];
    peakBin2(j) = [];
end

% figure(3)
% % hold on
% % for j = 2:1:frameCount
% %     plot(abs(captureFT(:, j)))
% % end
% captureFT(:, 1:2) = ones(512, 2); % first 2 frames of capture is extremely noisy
% x = (1:1:512)';
% y = (1:1:frameCount) / frameCount * frameRate;
% xMat = repmat(x, 1, length(y));
% yMat = repmat(y, length(x), 1);
% zMat = abs(captureFT(:, 1:frameCount));
% plot3(xMat, yMat, zMat)
% 
% xlabel('Range Bins')
% ylabel('Frequency')
% zlabel('Magnitude')
% title(strcat("Capture", " - FT bins"));

fprintf("Raw Data\n\n")
disp("Peak Magnitudes:")
disp(peakMagnitudes1)
disp(peakMagnitudes2)
disp("Peak Bins:")
disp(peakBin1)
disp(peakBin2)
disp("SNR:")
disp(SNRdB1)
disp(SNRdB2)

fprintf("\n%s Testing Results\n\n", captureName)

fprintf("Peak Magnitude Results:\n")
fprintf("%d Hz Tag: %.2f\n", tag1Hz, mean(peakMagnitudes1))
fprintf("%d Hz Tag: %.2f\n\n", tag2Hz, mean(peakMagnitudes2))

fprintf("SNR Results:\n")
fprintf("%d Hz Tag: %.2f dB\n", tag1Hz, mean(SNRdB1))
fprintf("%d Hz Tag: %.2f dB\n\n", tag2Hz, mean(SNRdB2))

fprintf("Peak Results:\n")
fprintf("%d Hz Tag: %.0f\n", tag1Hz, mean(peakBin1))
fprintf("%d Hz Tag: %.0f\n\n", tag2Hz, mean(peakBin2))

fprintf("Peak Variance:\n")
fprintf("|%.0f - %.0f| = %.0f\n\n", max(abs(peakBin1 - peakBin2)), min(abs(peakBin1 - peakBin2)), ...
    max(abs(peakBin1 - peakBin2)) - min(abs(peakBin1 - peakBin2)))

fprintf("Peak Bin Difference:\n")
fprintf("|%.0f - %.0f| = %.0f\n\n", mean(peakBin1), mean(peakBin2), mean(abs(peakBin1 - peakBin2)))


end