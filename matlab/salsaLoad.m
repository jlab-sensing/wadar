% TODO: (optional) - output a useful summary of parameters (e.g. in text or .mat file)...
% this could be done instead of returning pgen, fs_hz, etc.

% USAGE: Load radar data from a binary file (captured from frameLogger.c on BBB)
% REQUIRED ARGS: 
% fileName
% chipSet
function [frameTot pgen fs_hz chipSet times] = salsaLoad(fileName) 

fprintf('Loading saved data from %s...\n', fileName)

FRAME_LOGGER_MAGIC_NUM = hex2dec('FEFE00A2');
fid = fopen(fileName,'r');
% Check the magic#
magic = fread(fid,1,'uint32');
if magic ~= FRAME_LOGGER_MAGIC_NUM
    fprintf("Wrong data format: %s!\n", dataLogFile);
    fclose(dataLog);
    return;
end

% Next are the sweep controller settings
iterations = fread(fid,1,'int');
pps = fread(fid,1,'int');
dacMin = fread(fid,1,'int');
dacMax = fread(fid,1,'int');
dacStep = fread(fid,1,'int');

% The radar type is next 
radarSpecifier = fread(fid,1,'int'); % 2 for X2 (Ancho), 10 for X1-IPGO (Cayenne), 11 for X1-IPG1 (Chipotle)
switch radarSpecifier
    case 2
        chipSet = "X2";
        % The measured sampling rate is next
        samplesPerSecond = fread(fid,1,'float'); %float
        % The radarSpecifier-specific settings are next
        pgen = fread(fid,1,'int');
        offsetDistance = fread(fid,1,'float');
        sampleDelayToReference = fread(fid,1,'float');
    
    case 10 
        chipSet = "X1-IPGO";
        % The measured sampling rate is next
        samplesPerSecond = fread(fid,1,'double'); %double
        % The radarSpecifier-specific settings are next
        pgen = fread(fid,1,'int');
        samplingRate = fread(fid,1,'int');
        clkDivider = fread(fid,1,'int');
    case 11 
        chipSet = "X1-IPG1";
        % The measured sampling rate is next
        samplesPerSecond = fread(fid,1,'double'); %double
        % The radarSpecifier-specific settings are next
        pgen = fread(fid,1,'int');
        samplingRate = fread(fid,1,'int');
        clkDivider = fread(fid,1,'int');
        
end

% Next is the #samplers in a frame
numberOfSamplers = fread(fid,1,'int');
% Determine #frames and #runs in capture
numFrames = fread(fid,1,'int');
numRuns = fread(fid,1,'int');
% Next is the frames per second 
frameRate = fread(fid,1,'int');
% Next is an array of time data
times = fread(fid, numFrames, 'double');
% Here are the radar frames 
frameTot = fread(fid, numFrames*numberOfSamplers, 'uint32');
% Do the DAC normalization
frameTot = double(frameTot)/(1.0*pps*iterations)*dacStep + dacMin;
temp = frameTot;
frameTot = reshape(frameTot, numberOfSamplers, numFrames);


% Process out the weird spike
for i = 1:numFrames
    if max(frameTot(:,i)) > 8191
            if (i > 1)
                frameTot(:, i) = frameTot(:, i-1);
            else 
                frameTot(:, i) = frameTot(:, i+1);
            end
    end
end

% Estimated FPS (good to check against frameRate)
fpsEst = fread(fid, 1, 'float');

% TODO - insert comment here
[A,count] = fread(fid);
fclose(fid);
if count ~= 0
    fprintf("FILE READ ERROR: %i data remains! Check that file format matches read code\n",count)
    return
end
% Calculate some useful radar parameters that can be returned if needed
[fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] = NoveldaChipParams(chipSet, pgen,'4mm');

% temp = diff(times);
% for i = 1:length(temp)
%     if temp(i) > 0.006
%         % i
%     end
% end

close all

figure(3)
plot(diff(times))
xlabel("Frame")
ylabel("Frame time difference")
% 
% close all
figure(1)
plot(frameTot)
figure(2)
plot(frameTot)
ylim([0 8191])

end
