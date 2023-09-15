% This script compiles the loads radar data from a binary file (captured
% from frameLogger.c on BBB) and displays the data and the fourier 
% transform

% REQUIRED ARGS: 
% fileName

function salsaTest(fileName)

close all

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

[fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] = NoveldaChipParams(chipSet, pgen,'4mm');

% Next is the #samplers in a frame
numberOfSamplers = fread(fid,1,'int');
% Determine #frames and #runs in capture
numFrames = fread(fid,1,'int')
numRuns = fread(fid,1,'int');
% Next is the frames per second 
frameRate = fread(fid,1,'int');
% Next is an array of time data
times = fread(fid, numFrames, 'double');
% Here are the radar frames 
frameTot = fread(fid, numFrames*numberOfSamplers, 'uint32');
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

% Removes any
for i = 1:numFrames
    if max(frameTot(:,i)) > (dacMax + dacStep) | min(frameTot(:,i)) < (dacMin - dacStep)
            % frameTot(:, i) = zeros(1, numberOfSamplers);
            frameTot(:, i) = frameTot(:, i-1);
    end
end

for i = 1:size(frameTot,1)
    frameAvg(i) = mode(frameTot(i,:));
end
figure(1)
scanTimeSteps = 512;
c = 299792458.0; % speed of light

% TODO - fix resolution
resolution = 0.5 * c / samplesPerSecond;
range = linspace(0,numberOfSamplers,numberOfSamplers);

figure(1)
subplot(2,2,2)
plot(range, frameAvg);
xlabel('Range Bin');
ylabel('');
title('Average Raw Radar Scan');
subplot(2,2,1)
plot(range, frameTot);
xlabel('Range Bin');
ylabel('');
title('Raw Radar Scan');

frameFreq = zeros(512, numFrames);

for i = 1:height(frameTot)
    frameBin = frameTot(i,:) - mean(frameTot(i,:));
    frameFreq(i,:) = abs(fft(frameBin));
end

% frameFreq = abs(fft(frameBin));

% Ts = mean(diff(times));
% Fs = 1 / Ts
% F = (0:length(frameFreq)-1)*Fs/length(frameFreq);

F = zeros(1, numFrames);
F(1) = times(1) - 0;
for i = 2:length(times)
    F(i) = (i-1) * 1 / (times(i) - times(i-1)) / length(frameFreq);
end
F = sort(F);

tagFrequency = interp1(F, F, 80, "nearest");
tagIndex = find(F == tagFrequency);
[tagMagnitude, tagRangeBin] = max(frameFreq(:,tagIndex));

subplot(2,2,3)
plot(F, frameFreq)
xlim([0 max(F)/2])
xlabel('Frequency (Hz)')
ylabel('Magnitude')
title('FFT of Radar Frames');

subplot(2,2,4)
plot(F, frameFreq(tagRangeBin, :))
xlim([20 max(F)/2])
xlabel('Frequency (Hz)')
ylabel('Magnitude')
title("Range Bin " + tagRangeBin + " Isolated");

subplot(2,2,1)
hold on
plot(range(tagRangeBin), frameAvg(tagRangeBin),'rx')

fprintf("A frequency of %f is detected at %f inches\n", interp1(F, F, 80, "nearest"), tagRangeBin*resolution*39.17)

end