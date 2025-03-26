function [frameTot, framesBB, frameRate] = proc_frames(localDataPath, captureName)
% [frameTot, framesBB, captureFT, tagFT] = proc_frames(localDataPath, captureName, tagHz, frameRate)
%
% Function to process Novelda radar data captures. This function reads in a 
% radar data capture file and processes the data to extract the radar frames
% and the baseband signal. 
%
% Inputs:
%   localDataPath: Path to the data capture file.
%   captureName: Name of the data capture file.
%   tagHz: Tag frequency in Hz.
%   frameRate: Frame rate in Hz.
%
% Outputs:
%   frameTot: Raw radar data frames.
%   basebandSignal: Downconverted, i.e. baseband, and filtered IQ radar 
%                   signal, note: use abs() on output to get envelope
%                   magnitude.

% Setting default parameters
arguments 
    localDataPath = '/data'
    captureName = 'untitled1.frames'
end

% To make the MATLAB Coder stop complaining about execution paths.
frameTot = 0;
framesBB = 0;
frameRate = 0;

%% Load Capture

fullDataPath = strcat(pwd, localDataPath);

fileName = fullfile(fullDataPath, captureName);

FRAME_LOGGER_MAGIC_NUM = hex2dec('FEFE00A2');
fid = fopen(fileName,'r');

magic = fread(fid,1,'uint32');              % Check the magic number
if magic ~= FRAME_LOGGER_MAGIC_NUM
    fprintf("Wrong data format: %s!\n", captureName);
    fclose(fileName);
    return;
end

iterations = fread(fid,1,'int');            % Sweep controller settings
pps = fread(fid,1,'int');
dacMin = fread(fid,1,'int');
dacMax = fread(fid,1,'int');
dacStep = fread(fid,1,'int');

radarSpecifier = fread(fid,1,'int');        % 2 for X2 (Ancho), 10 for X1-IPGO (Cayenne), 11 for X1-IPG1 (Chipotle)

% Dummy variables because the MATLAB  Coder keeps yelling at me
chipSet = "XX-XXXX";
samplesPerSecond = 0;      
pgen = 0;                      
samplingRate = 0;
clkDivider = 0;

switch radarSpecifier
    case 2
        chipSet = "X2";
        samplesPerSecond = fread(fid,1,'float');        % Sampling Rate
        pgen = fread(fid,1,'int');                      % Radar Specifier settings
        offsetDistance = fread(fid,1,'float');
        sampleDelayToReference = fread(fid,1,'float');
    
    case 10 
        chipSet = "X1-IPGO";
        samplesPerSecond = fread(fid,1,'double');        % Sampling Rate
        pgen = fread(fid,1,'int');                       % Radar Specifier settings
        samplingRate = fread(fid,1,'int');
        clkDivider = fread(fid,1,'int');
    case 11 
        chipSet = "X1-IPG1";
        samplesPerSecond = fread(fid,1,'double');        % Sampling Rate
        pgen = fread(fid,1,'int');                       % Radar Specifier settings
        samplingRate = fread(fid,1,'int');
        clkDivider = fread(fid,1,'int');
end

numberOfSamplers = fread(fid,1,'int'); 
numberOfSamplers = numberOfSamplers(1,1);                       % Number of samplers in a frame
numFrames = fread(fid, 1, 'int'); 
numFrames = numFrames(1,1);                                     % Number of frames in a capture
numRuns = fread(fid,1,'int');                                   % Number of runs in capture
frameRate = fread(fid,1,'int');                                 
frameRate = frameRate(1,1);                                     % Frames per second
times = fread(fid, numFrames, 'double');
frameTot = fread(fid, numFrames*numberOfSamplers, 'uint32');    % Radar frames

% DAC normalization
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

fpsEst = fread(fid, 1, 'float');                % Estimated FPS (good to check against frameRate)

% TODO - insert comment here
[A,count] = fread(fid);
fclose(fid);
if count ~= 0
    fprintf("FILE READ ERROR: %f data remains! Check that file format matches read code\n", floor(count))
    return
end

% Unused but potentially useful radar parameters
[fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] = NoveldaChipParams(chipSet, pgen,'4mm');

% Baseband Conversion
frameCount = size(frameTot, 2);
framesBB = complex(zeros(size(frameTot)), zeros(size(frameTot)));
for j = 1:frameCount
    framesBB(:,j) = NoveldaDDC(frameTot(:,j), chipSet, pgen, fs_hz);
end

end