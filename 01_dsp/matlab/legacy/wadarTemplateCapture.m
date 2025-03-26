    function wadarTemplateCapture(localDataPath, trialName)
% wadarTemplateCapture(localDataPath, trialName)
%
% wadarTemplateCapture captures the template capture for correlation 
% analysis
%
% Inputs:
%       localDataPath: Path to store template capture
%       trialName: Trial name for file naming purposes
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
captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_Template_T', trialName, '_C');

% Check for existing files with the same name to prevent overwrite
existingFiles = dir(localDataPath);

for i = 1:length(existingFiles)
    for j = 1:1:10
        if strcmp(existingFiles(i).name, strcat(captureName, num2str(j), '.frames'))
            error("Files under this trial name already exist. Iterate the trial name")
        end
    end
end

%% Commit Radar Capture

% Send Frame Logger command with appropriate parameters
frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r 1 -f %d -t %s -c %s', ...
    captureName, frameCount, frameRate, radarType, fullDataPath);
frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
    frameLoggerOptions);
[status,~] = system(frameLoggerCommand);

fprintf("Please wait. The radar is collecting data.\n")
pause(frameCount/frameRate);
fprintf("Waiting for data to be transferred...\n")

% Verify that the capture is saved
i = 1;
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
    fprintf('Framelogger captured frames succesfully!\n\n')
end

%% Process Capture Frames
[procResult, ~, templateTagFT, templatePeakBin, ~] = procRadarFrames(localDataPath, strcat(captureName, '1.frames'));

if (procResult == false)
    error("ERROR: Template processing error. Please try again.")
end

if (templatePeakBin == -1)
    error("ERROR: No peak detected. Please ensure backscatter tag is actively powered.\n")
end

%% Display Radar Frames
figure(1)
plot(templateTagFT)
xline(templatePeakBin)
xlabel('Range Bins')
ylabel('Magnitude')
title("Template Capture - 80 Hz Isolated");

% Manually verify template frames using user input
validTemplateCapture = input("\nDoes this capture follow the following requirements: (Y/N)\n" + ...
    "     - A clear and obvious peak is visible\n" + ...
    "     - There is no double peak\n", ...
    "s");

if (strcmp(validTemplateCapture, "Y"))
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