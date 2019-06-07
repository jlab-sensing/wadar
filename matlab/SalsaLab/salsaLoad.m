%TODO: output a useful summary .txt or similar file with capture info 
%(be sure to delete old files)
% OR return the parameters that are needed using the function return

function [frameTot pgen fs_hz] = salsaLoad(fileName, radarSpecifier) 

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
radarType = fread(fid,1,'int'); 
switch radarType
    case 2
        % The measured sampling rate is next
        samplesPerSecond = fread(fid,1,'float');
        % The NVA6201 specific settings are next
        pgen = fread(fid,1,'int');
        offsetDistance = fread(fid,1,'float');
        sampleDelayToReference = fread(fid,1,'float');
    otherwise 
        % The measured sampling rate is next
        samplesPerSecond = fread(fid,1,'double');
        % The NVA6201 specific settings are next
        pgen = fread(fid,1,'int');
        samplingRate = fread(fid,1,'int');
        clkDivider = fread(fid,1,'int');
end

% Next is the #samplers in a frame
numberOfSamplers = fread(fid,1,'int');
% Determine #frames in capture
numFrames = fread(fid,1,'int');
numRuns = fread(fid,1,'int');
frameRate = fread(fid,1,'int');

times = fread(fid, numFrames, 'double');
frameTot = fread(fid, numFrames*numberOfSamplers, 'uint32');
% Do the DAC normalization
frameTot = double(frameTot)/(1.0*pps*iterations)*dacStep + dacMin;
frameTot = reshape(frameTot, numberOfSamplers, numFrames);

fpsEst = fread(fid, 1, 'float');
[A,count] = fread(fid);
fclose(fid);
if count ~= 0
    fprintf("FILE READ ERROR: %i data remains! Check that file format matches read code\n",count)
    return
end
frameCount = size(frameTot,2);

[fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] = NoveldaChipParams(radarSpecifier, pgen,'4mm');
timeTot = (numFrames - 1) / frameRate; 

end
