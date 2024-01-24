function [captureSuccess, peakBin, SNR, SNRdB] = tagTest(localDataPath, tagName, trialIndex)
% vwc = tagTest(airFileName, captureName)
%
% Function captures radar frames to test tag SNR and display capture
% fourier transform
%
% Inputs:
%        localDataPath: Location to store capture on local machine
%        tagName: Name of tag for file name
%        trialIndex: trialNumber for file name
%
% Outputs:
%   vwc: Calculated volumetric water content

close all;

captureSuccess = 0;

% Capture parameters
frameRate = 200;   
frameCount = 6000;
radarType = 'Chipotle';
fullDataPath = sprintf("ericdvet@192.168.7.1:%s",localDataPath);

% Processing parameters
tagHz = 80;

[year, month, date] = ymd(datetime("now"));
captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '-', tagName, '-Trial', num2str(trialIndex), '-Capture');

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
pause(10)

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

%% Load Template Capture
[rawFrames, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, strcat(captureName, '1.frames')));

% Baseband Conversion
frameCount = size(rawFrames, 2);
framesBB = zeros(size(rawFrames));
for j = 1:frameCount
    framesBB(:,j) = NoveldaDDC(rawFrames(:,j), chipSet, pgen, fs_hz);
end

% Find Tag FT
freqTag = tagHz / frameRate * frameCount;
captureFT = fft(framesBB, frameCount , 2); 
tagFT = abs(captureFT(:, freqTag));
for i = (freqTag-2:1:freqTag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tagFT)
        tagFT = temp;
    end
end
tagFT = smoothdata(tagFT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(tagFT, 'MinPeakHeight', max(tagFT) * 0.9);
if (size(peaks, 1) > 1)
    if (peaks(2) - peaks(1) < 50)
        peakBin = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((tagFT(peaks(2)) - ...
            tagFT(peaks(1))) / max(tagFT) * (peaks(2) - peaks(1)));
    else
        peakBin = peaks(1);
    end
elseif (size(peaks, 1) == 1)
    peakBin = peaks(1);
else
    peakBin = 0;
    fprintf("ERROR: No peak detected. Please ensure backscatter tag is actively powered.\n")
end

figure(1)
plot(tagFT)
xline(peakBin)
xlabel('Range Bins')
ylabel('Magnitude')
title("Capture - 80 Hz Isolated");

figure(2)
hold on
for i = 2:1:2000
    plot(abs(captureFT(:, i)))
end
xlabel('Range Bins')
ylabel('Magnitude')
title("FT of all peak bins");

SNR = calculateSNR(captureFT, freqTag, peakBin);
SNRdB = 10 * log10(SNR)

end