function wadarTagTest(displayAllFrames, localDataPath, tagName, trialName, captureCount)
% wadarTagTest(localDataPath, tagName, trialName, captureCount)
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
frameCount = 200;
radarType = 'Chipotle';
fullDataPath = sprintf("ericdvet@192.168.7.1:%s",localDataPath);

% File name generation
if isnumeric(trialName)
    trialName = num2str(trialName);
end
if isnumeric(tagName)
    tagName = num2str(tagName);
end
[year, month, date] = ymd(datetime("now"));
captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_', tagName, '_T', trialName, '_C');

% Check for existing files with the same name to prevent overwrite
existingFiles = dir(localDataPath);

for i = 1:length(existingFiles)
    for j = 1:1:10
        if strcmp(existingFiles(i).name, strcat(captureName, num2str(j), '.frames'))
            error("Files under this trial index already exist. Iterate the trial index.")
            return
        end
    end
end

%% Commit Radar Capture
    
% Send Frame Logger command with appropriate parameters
frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
    captureName, frameCount, captureCount, frameRate, radarType, fullDataPath);
frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
    frameLoggerOptions);
[status,~] = system(frameLoggerCommand);

%% Process Capture Frames
SNRdB = zeros(1, captureCount);
peakMagnitudes = zeros(1, captureCount);
peakBin = zeros(1, captureCount);
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

    [procResult, captureFT, tagFT, peakBin(i), SNRdB(i)] = procRadarFrames(localDataPath, strcat(captureName, num2str(i), '.frames'));

    if (procResult == false)
        failedCaptures = [failedCaptures i];
        fprintf("Capture %d faced a processing issue.\n", i)
        peakBin(i) = 0;
        SNRdB(i) = 0;
        peakMagnitudes(i) = 0;
        continue
    end

    peakMagnitudes(i) = tagFT(peakBin(i));

    % Display each capture if demanded
    if (displayAllFrames == true)
        figure(1)
        plot(tagFT)
        xline(peakBin(i))
        xlabel('Range Bins')
        ylabel('Magnitude')
        title(strcat("Capture ", num2str(i), " - 80 Hz Isolated"));
        
        figure(2)
        clf(2)
        hold on
        for j = 2:1:frameCount
            plot(abs(captureFT(:, j)))
        end
        xlabel('Range Bins')
        ylabel('Magnitude')
        title(strcat("Capture ", num2str(i), " - FT of all peak bins"));
    end

end

% Remove failed captures
for i = failedCaptures
    SNRdB(i) = [];
    peakMagnitudes(i) = [];
    peakBin(i) = [];
end

%% Display Results
fprintf("\n%s Testing Results (Trial %s)\n\n", tagName, trialName)

fprintf("SNR Results:\n")
fprintf("Median: %fdB\nMean: %fdB\n\n", median(SNRdB), mean(SNRdB))


fprintf("Peak Magnitude Results:\n")
fprintf("Median: %f\nMean: %f\n\n", median(peakMagnitudes), mean(peakMagnitudes))

fprintf("Maximum difference between peak bins: %d\n\n", max(diff(sort(peakBin))))

close all

figure(1)
plot(tagFT)
xline(peakBin(captureCount))
xlabel('Range Bins')
ylabel('Magnitude')
title(strcat("Capture ", num2str(captureCount), " - 80 Hz Isolated"));

figure(2)
hold on
for j = 2:1:frameCount
    plot(abs(captureFT(:, j)))
end
xlabel('Range Bins')
ylabel('Magnitude')
title(strcat("Capture ", num2str(captureCount), " - FT of all peak bins"));

end