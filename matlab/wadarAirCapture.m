function [captureSuccess, airPeakBin] = wadarAirCapture(localDataPath, trialIndex)
% vwc = wadarAirCapture(airFileName, captureName)
%
% Function captures the air capture for peak bin comparing
%
% Inputs:
%        
%
% Outputs:
%   captureSuccess: Flag to demonstrate if capture was successful

close all;

captureSuccess = 0;

% Capture parameters
frameRate = 200;   
frameCount = 200;
radarType = 'Chipotle';
fullDataPath = sprintf("ericdvet@192.168.7.1:%s",localDataPath);

% Processing parameters
tagHz = 80;

[year, month, date] = ymd(datetime("now"));
captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_Air_T', num2str(trialIndex), '_C');

% Check for existing files with the same name
existingFiles = dir(localDataPath);

% for i = 1:length(existingFiles)
%     for j = 1:1:10
%         if strcmp(existingFiles(i).name, strcat(captureName, num2str(j), '.frames'))
%             error("Files under this trial index already exist. Iterate the trial index.")
%             captureSuccess = 0;
%             return
%         end
%     end
% end

frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r 1 -f %d -t %s -c %s', ...
    captureName, frameCount, frameRate, radarType, fullDataPath);
frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
    frameLoggerOptions);
[status,~] = system(frameLoggerCommand);
  
fprintf("Please wait. The radar is collecting data.\n")
pause(frameCount/frameRate);
fprintf("Waiting for data to be transferred...\n")
pause(5)

checkFile = dir(fullfile(localDataPath, strcat(captureName, '1.frames')));
checkmd5File = dir(fullfile(localDataPath, strcat(captureName, '1.md5')));
if (length(checkFile) ~= 1) || (length(checkmd5File) ~= 1)
    error('There is a data transfer issue. Please verify your capture settings and scp directory.')
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
    fprintf('Framelogger captured frames succesfully!\n\n')
end

%% Load Air Capture
[airRawFrames, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, strcat(captureName, '1.frames')));

% Baseband Conversion
airFrameCount = size(airRawFrames, 2);
airFramesBB = zeros(size(airRawFrames));
for j = 1:airFrameCount
    airFramesBB(:,j) = NoveldaDDC(airRawFrames(:,j), chipSet, pgen, fs_hz);
end

% Find Tag FT
airFreqTag = tagHz / frameRate * airFrameCount;
airFT = fft(airFramesBB, airFrameCount , 2); 
airTagFT = abs(airFT(:, airFreqTag));
for i = (airFreqTag-2:1:airFreqTag+2)
    temp = abs(airFT(:, i));
    if max(temp) > max(airTagFT)
        airTagFT = temp;
    end
end
airTagFT = smoothdata(airTagFT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(airTagFT, 'MinPeakHeight', max(airTagFT) * 0.9);
if (size(peaks, 1) > 1)
    if (peaks(2) - peaks(1) < 50)
        airPeakBin = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((airTagFT(peaks(2)) - ...
            airTagFT(peaks(1))) / max(airTagFT) * (peaks(2) - peaks(1)));
    else
        airPeakBin = peaks(1);
    end
elseif (size(peaks, 1) == 1)
    airPeakBin = peaks(1);
else
    airPeakBin = 0;
    fprintf("ERROR: No peak detected. Please ensure backscatter tag is actively powered.\n")
end

figure(1)
plot(airTagFT)
xline(airPeakBin)
xlabel('Range Bins')
ylabel('Magnitude')
title("Air Capture - 80 Hz Isolated");


validAirCapture = input("\nDoes this capture follow the following requirements: (Y/N)\n" + ...
    "     - A clear and obvious peak is visible\n" + ...
    "     - There is no double peak\n", ...
    "s");

if (strcmp(validAirCapture, "Y"))
    captureSuccess = 1;
    return
elseif (strcmp(validAirCapture, "N"))
    fprintf("\n")
    delete(fullfile(localDataPath, strcat(captureName, '1.frames')))
    delete(fullfile(localDataPath, strcat(captureName, '1.md5')))
    wadarAirCapture(localDataPath, trialIndex);
else
    error("Invalid input")
end

end