function fileName = CaptureData(captureName, localDataPath, frameCount, frameRate, radarType)
% fileName = CaptureData(captureName, localDataPath, frameCount, frameRate, radarType)
%
% Function to acquire a radar capture and save it to a file. This function
% sends a command to the radar to start a capture and then waits for the
% capture to be saved to the specified directory. The function will check
% for the existence of the file and then return the name of the file.
%
% Inputs:
%   captureName: Name of the data capture file.
%   localDataPath: Path to the data capture file.
%   frameCount: Number of frames to capture.
%   frameRate: Frame rate in Hz.
%   radarType: Type of radar to capture data from.
%
% Outputs:
%   fileName: Name of the saved data capture file.

% Setting default parameters
arguments 
    captureName = 'untitled'
    localDataPath = strcat(pwd, '/data')
    frameCount = 2000
    frameRate = 200
    radarType = 'Chipotle'
end

if localDataPath(1) == '/'
    error("ERROR: Do not preface local data path with /")
end

% Data path generation
[~, hostname] = system('whoami');
hostname(end) = '';
fullDataPath = sprintf("%s@192.168.7.1:%s/%s", hostname, pwd, localDataPath);

% Check for existing files with the same name to prevent overwrite
existingFiles = dir(localDataPath);
for i = 1:length(existingFiles)
    if strcmp(existingFiles(i).name, strcat(captureName, '1.frames'))
        error("ERROR: File with name already exists.")
        return
    end
end

%% Commit Radar Capture

% Send Frame Logger command with appropriate parameters
frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r 1 -f %d -t %s -c %s', ...
    captureName, frameCount, frameRate, radarType, fullDataPath);
frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
    frameLoggerOptions);
[status,~] = system(frameLoggerCommand);

waitTime = frameCount/frameRate + frameCount/frameRate;
fprintf("Radar scan in progress. Expected wait time is %.2f seconds.\n", waitTime);
pause(waitTime);
fprintf("Checking data transfer success.\n")

% Verify that the capture is saved
i = 1;
checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(i), '.frames')));
checkmd5File = dir(fullfile(localDataPath, strcat(captureName, num2str(i), '.md5')));
tic
while (length(checkFile) ~= 1) || (length(checkmd5File) ~= 1)
    checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(i), '.frames')));
    checkmd5File = dir(fullfile(localDataPath, strcat(captureName, num2str(i), '.md5')));
    if (toc > 20)
        error('ERROR: There is a data transfer issue. Please verify your capture settings and scp directory.')
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
    error('ERROR: There has been an error in the file transfer. The md5 hashes do not match.\n')
end

fprintf("Done =)\n\n")

end