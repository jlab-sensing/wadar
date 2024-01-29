function wadarAirCapture(localDataPath, trialName)
% wadarAirCapture(airFileName, captureName)
%
% wadarAirCapture captures the backscatter tag buried with no soil covering
% it
%
% Inputs:
%       localDataPath: Path to store air capture
%       trialName: Trial name for file naming purposes
%
% Outputs:
%       None

close all;

% Capture parameters
frameRate = 200;   
frameCount = 2000;
radarType = 'Chipotle';
fullDataPath = sprintf("ericdvet@192.168.7.1:%s",localDataPath);

% File name generation
if isnumeric(trialName)
    trialName = num2str(trialName);
end

[year, month, date] = ymd(datetime("now"));
captureName = strcat(num2str(year), '-', num2str(month), '-', num2str(date), '_Air_T', num2str(trialName), '_C');

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
frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r 1 -f %d -t %s -c %s', ...
    captureName, frameCount, frameRate, radarType, fullDataPath);
frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
    frameLoggerOptions);
[status,~] = system(frameLoggerCommand);
  
fprintf("Please wait. The radar is collecting data.\n")
pause(frameCount/frameRate);
fprintf("Waiting for data to be transferred...\n")
pause(5)

% Verify that the capture is saved
checkFile = dir(fullfile(localDataPath, strcat(captureName, '1.frames')));
checkmd5File = dir(fullfile(localDataPath, strcat(captureName, '1.md5')));
if (length(checkFile) ~= 1) || (length(checkmd5File) ~= 1)
    error('There is a data transfer issue. Please verify your capture settings and scp directory.')
end

% Verify that the md5 file checks out
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
[procResult, ~, airTagFT, airPeakBin, ~] = procRadarFrames(localDataPath, strcat(captureName, '1.frames'));

if (procResult == false)
    error("ERROR: Template processing error. Please try again.")
end

if (airPeakBin == -1)
    error("ERROR: No peak detected. Please ensure backscatter tag is actively powered.\n")
end

%% Display Radar Frames
figure(1)
plot(airTagFT)
xline(airPeakBin)
xlabel('Range Bins')
ylabel('Magnitude')
title("Air Capture - 80 Hz Isolated");

% Manually verify template frames using user input
validAirCapture = input("\nDoes this capture follow the following requirements: (Y/N)\n" + ...
    "     - A clear and obvious peak is visible\n" + ...
    "     - There is no double peak\n", ...
    "s");

if (strcmp(validAirCapture, "Y"))
    return
elseif (strcmp(validAirCapture, "N"))
    fprintf("\n")
    delete(fullfile(localDataPath, strcat(captureName, '1.frames')))
    delete(fullfile(localDataPath, strcat(captureName, '1.md5')))
    wadarAirCapture(localDataPath, trialName);
else
    error("Invalid input")
end

end