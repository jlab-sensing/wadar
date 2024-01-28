function [captureSuccess, templatePeakBin] = wadarTemplateCapture(localDataPath, trialIndex)
% vwc = wadarTemplateCapture(airFileName, captureName)
%
% Function captures the template capture for correlation analysis
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
frameCount = 2000;
radarType = 'Chipotle';
fullDataPath = sprintf("ericdvet@192.168.7.1:%s",localDataPath);

% Processing parameters
tagHz = 80;

[year, month, date] = ymd(datetime("now"));
captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_Template_T', num2str(trialIndex), '_C');

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

%% Load Template Capture
[templateRawFrames, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, strcat(captureName, '1.frames')));

% Baseband Conversion
templateFrameCount = size(templateRawFrames, 2);
templateFramesBB = zeros(size(templateRawFrames));
for j = 1:templateFrameCount
    templateFramesBB(:,j) = NoveldaDDC(templateRawFrames(:,j), chipSet, pgen, fs_hz);
end

% Find Tag FT
templateFreqTag = tagHz / frameRate * templateFrameCount;
templateFT = fft(templateFramesBB, templateFrameCount , 2); 
templateTagFT = abs(templateFT(:, templateFreqTag));
for i = (templateFreqTag-2:1:templateFreqTag+2)
    temp = abs(templateFT(:, i));
    if max(temp) > max(templateTagFT)
        templateTagFT = temp;
    end
end
templateTagFT = smoothdata(templateTagFT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(templateTagFT, 'MinPeakHeight', max(templateTagFT) * 0.9);
if (size(peaks, 1) > 1)
    if (peaks(2) - peaks(1) < 50)
        templatePeakBin = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((templateTagFT(peaks(2)) - ...
            templateTagFT(peaks(1))) / max(templateTagFT) * (peaks(2) - peaks(1)));
    else
        templatePeakBin = peaks(1);
    end
elseif (size(peaks, 1) == 1)
    templatePeakBin = peaks(1);
else
    templatePeakBin = 0;
    fprintf("ERROR: No peak detected. Please ensure backscatter tag is actively powered.\n")
end

figure(1)
plot(templateTagFT)
xline(templatePeakBin)
xlabel('Range Bins')
ylabel('Magnitude')
title("Template Capture - 80 Hz Isolated");


validTemplateCapture = input("\nDoes this capture follow the following requirements: (Y/N)\n" + ...
    "     - A clear and obvious peak is visible\n" + ...
    "     - There is no double peak\n", ...
    "s");

if (strcmp(validTemplateCapture, "Y"))
    captureSuccess = 1;
    return
elseif (strcmp(validTemplateCapture, "N"))
    fprintf("\n")
    delete(fullfile(localDataPath, strcat(captureName, '1.frames')))
    delete(fullfile(localDataPath, strcat(captureName, '1.md5')))
    wadarTemplateCapture(localDataPath, trialIndex);
else
    error("Invalid input")
end
validTemplateCapture = input("\nDoes this capture follow the following requirements: (Y/N)\n" + ...
    "     - A clear and obvious peak is visible\n" + ...
    "     - There is no double peak\n", ...
    "s");

if (strcmp(validTemplateCapture, "Y"))
    captureSuccess = 1;
    return
elseif (strcmp(validTemplateCapture, "N"))
    fprintf("\n")
    delete(fullfile(localDataPath, strcat(captureName, '1.frames')))
    delete(fullfile(localDataPath, strcat(captureName, '1.md5')))
    wadarTemplateCapture(localDataPath, trialIndex);
else
    error("Invalid input")
end

end