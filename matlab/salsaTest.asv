% This script compules the loads radar data from a binary file (captured from frameLogger.c on BBB)
% and displays the data as a BScan plot

% REQUIRED ARGS: 
% fileName

function salsaLoad(fileName)

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
% temp = frameTot;
% Do the DAC normalization
frameTot = double(frameTot)/(1.0*pps*iterations)*dacStep + dacMin;
frameTot = reshape(frameTot, numberOfSamplers, numFrames);

% Estimated FPS (good to check against frameRate)
fpsEst = fread(fid, 1, 'float');

% TODO - insert comment here
[A,count] = fread(fid);
fclose(fid);
if count ~= 0
    fprintf("FILE READ ERROR: %i data remains! Check that file format matches read code\n",count)
    return
end 



for i = 1:size(frameTot,1)
    frameAvg(i) = mean(frameTot(i,:));
end
figure(1)
scanTimeSteps = 512;
myScan = zeros(scanTimeSteps, numberOfSamplers);
myScanAvg = zeros(scanTimeSteps, numberOfSamplers);
imagesc(myScan)
colormap(flipud(colormap(gray)));
c = 299792458;
resolution = 1/double(samplesPerSecond)*c/2;
% resolution = 2 / numberOfSamplers
% resolution = 0.003790984152165;
% range = linspace(0,numberOfSamplers*resolution,numberOfSamplers);
range = linspace(0,numberOfSamplers*resolution*39.37,numberOfSamplers);

% numberOfSamplers*resolution

for i = 1:width(myScan)
    myScan(i,:) = frameTot(:,i);
    myScanAvg(i,:) = frameAvg;
end
% myScan(end,:) = frameAvg;
imagesc(1:scanTimeSteps, range, myScan');
% xlabel('Time Steps');
ylabel('Range [in]');
title('Average Raw Radar BScan');
drawnow();
xticks([0 256 512])
xticklabels({0, 1000, 2000})
figure(2)
subplot(2,1,1)
plot(range, frameAvg);
xlabel('Range [in]');
ylabel('');
title('Average Raw Radar Scan');

% figure(3)
subplot(2,1,2)
plot(range, frameTot);
title('Total Raw Radar Scan');
ylabel('')
xlabel('Range [in]');

end