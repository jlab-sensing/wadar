% demoDualTag.m
%
% Live demo of the dual tag in action

close all;

% Capture parameters
frameRate = 200;   
frameCount = 2000;
displayAllFrames = false;
localDataPath = '/home/ericdvet/jlab/wadar/matlab/data/';
captureCount = 1;
captureNum = 1;
tag1Hz = 79;
tag2Hz = 80;

radarType = 'Chipotle';
fullDataPath = sprintf("ericdvet@192.168.7.1:%s",localDataPath);

% File name generation
[year, month, date] = ymd(datetime("now"));
captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_', 'demoDualTagScan', num2str(captureNum), '_C');

% Check for existing files with the same name to prevent overwrite
existingFiles = dir(localDataPath);

% for i = 1:length(existingFiles)
%     for j = 1:1:10
%         if strcmp(existingFiles(i).name, strcat(captureName, num2str(j), '.frames'))
%             error("Files under this name already exist in the set directory.")
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

vwcList = [];

while true
  
    fprintf("Please wait. Capture %d is proceeding\n", captureNum)
    pause(frameCount/frameRate + 5);
    fprintf("Waiting for data to be transferred...\n")
    
    checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
    checkmd5File = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.md5')));
    tic
    while (length(checkFile) ~= 1) || (length(checkmd5File) ~= 1)
        checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
        checkmd5File = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.md5')));
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
    [procResult, captureFT, diffBins, tag1FT, tag2FT, peakBin1, peakBin2, SNRdB1, SNRdB2] = procTwoTag(localDataPath, strcat(captureName, num2str(1), '.frames'), tag1Hz, tag2Hz, displayAllFrames);

    if (procResult == false)
        fprintf("Capture %d faced a processing issue.\n", captureNum-1)
    end

    vwc = procSoilMoisture(max(peakBin1, peakBin2), min(peakBin1, peakBin2), 'farm', 0.1524);
    vwcList = [vwcList vwc];

    %% display data %%
    % correlation plot
    subplot(2,2,1);
    cla
    plot(tag1FT, 'displayname', sprintf('%i Hz FFT bin of tag 1',tag1Hz)); hold on;
    plot(tag2FT, 'DisplayName', sprintf('%i Hz FFT bin of tag 2',tag2Hz)); hold on; 
    plot(peakBin1, tag1FT(peakBin1), 'o', 'DisplayName', sprintf('detected peak = %i', peakBin1));
    plot(peakBin2, tag2FT(peakBin2), 'o', 'DisplayName', sprintf('detected peak = %i', peakBin2));
    title('Detected Peaks') 
    xlabel('Bins')
    ylabel('Magnitude') 
    legend();
    grid on
    
    % VWC plot
    subplot(2,2,3);
    plot(vwcList, '-o')
    ylim([0 0.6])
    ylabel("water content")
    xlabel("time")
    xlim([0 length(vwcList)+5])
    grid on
    title("VWC history")
    
    % stats
    subplot(2,2,2);
    cla
    text(0.1,1, sprintf('Measurement #%i', captureNum),'fontsize', 20);axis off
    text(0.1,0.65, sprintf('Capture duration: %is', frameCount/frameRate), 'fontsize',18); 
    text(0.1,0.50, sprintf('SNR: %f dB, %f dB', SNRdB1, SNRdB2), 'fontsize',18); 
    
    subplot(2,2,4);
    bar(vwc)
    ylim([0 0.6])
    grid on
    title(sprintf("Current VWC = %.3f", vwc))

    % Send Frame Logger command with appropriate parameters
    captureNum = captureNum + 1;
    captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_', 'demoDualTagScan', num2str(captureNum), '_C');

    frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
        captureName, frameCount, captureCount, frameRate, radarType, fullDataPath);
    frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
        frameLoggerOptions);
    [status,~] = system(frameLoggerCommand);

end