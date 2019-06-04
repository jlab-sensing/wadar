% USAGE: Captures or loads radar data from C or MATLAB and displays results  

% REQUIRED ARGS: 
% isDataCaptured: Boolean specifying whether data should be loaded from file or newly acquired

% OPTIONAL ARGS:
% file string: string to specify path to load or save data 
    % required if isDataCaptured == 0
% radar type: Ancho or Cayenne or Chipotle 
    % required if isDataCaptured == 1

% EXAMPLE 1: Capture and display data from Ancho, with save
% salsaMain(1, '/home/bradley/Documents/research/radar/matlab/Data/, 'Ancho')
    
% EXAMPLE 2: Capture and display data from Cayenne, don't save
% salsaMain(1, 'Cayenne')
    
% EXAMPLE 3: Load data and display
% salsaMain(0, '/home/bradley/Documents/research/radar/matlab/Data/myData.mat')

function salsaMain(isDataCaptured, varargin)
%% Process arguments 
userRadarType = -1; 
savePath = 'default'; 
fileName = 'default'; 

if isDataCaptured == 1
    for i = 1:length(varargin)
        arg = varargin{i}; 
        if (strcmp(lower(arg),'ancho') || strcmp(lower(arg),'cayenne') || strcmp(lower(arg),'chipotle'))
            userRadarType = lower(arg);
        else
            savePath = arg; 
        end
    end
        
    if ~(strcmp(userRadarType,'ancho') || strcmp(userRadarType,'cayenne') || strcmp(userRadarType,'chipotle'))
            error('Invalid radar type')
    end
    
else 
    for i = 1:length(varargin)
        arg = varargin{i}; 
        if (strcmp(lower(arg),'ancho') || strcmp(lower(arg),'cayenne') || strcmp(lower(arg),'chipotle'))
            % Do not update radar type to load data
        else 
            fileName = arg; 
        end
    end
        
    if (strcmp(fileName, 'default'))
        error('No file name provided to load data from')
    end
end

%% Load, capture, or capture & save data
if  isDataCaptured == 0 % load
    fprintf('Loading saved data from %s...\n', fileName)
    
    if (strcmp(fileName(end-3:end),'.mat')) %MATLAB
        load(fileName);
        
    else %C
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
                % The frame offset, reference delay
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
    end
    
    if radarType == 2
        radarSpecifier = "X2"; 
    elseif radarType == 10
        radarSpecifier = "X1-IPG0"; 
    else 
        radarSpecifier = "X1-IPG1"; 
    end
    
    [fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] = NoveldaChipParams(radarSpecifier, pgen,'4mm');
    timeTot = (numFrames - 1) / frameRate; 

else %capture
      % TODO: Call C code 
%     % INITIALIZE RADAR SETTINGS --------------------------------------------------------------------
%     fprintf('Initializing radar settings'); 
% 
%     % Create the radar object
%     radar = radarWrapper('192.168.7.2'); %USB Cable
% 
%     % Get a list of the connected modules
%     modules = radar.ConnectedModules;
% 
%     % Open a connection to the radar module
%     radar.Open(modules{1});
% 
%     % Set some register values 
%     radar.TryUpdateChip('Iterations','16');
%     radar.TryUpdateChip('DACMin','4300');
%     radar.TryUpdateChip('DACMax','4500');
%     radar.TryUpdateChip('DACStep','8');
%     radar.TryUpdateChip('PulsesPerStep','8');
%     radar.TryUpdateChip('FrameStitch','1');
%     % Changing PRF: PRFDivide divides the default
%     % 100Mhz PRF. So setting  to 2 yields PRF = 50Mhz
%     % radar.TryUpdateChip('PRFDivide','6');
% 
%     % Set some specific radarlib3 settings
%     if (strcmp(radarType,'ancho'))
%         pgen = 0; %Ancho center frequency 
%         radar.TryUpdateChip('PGSelect', pgen);
%         radar.SetVoltage(1.2);
% 
%         sampleDelayToReference = 3.687e-9; 
%         radarSpecifier = 'X2'; 
%     elseif (strcmp(radarType, 'cayenne')) 
%         pgen = 0; %Cayenne center frequency???
%         radar.TryUpdateChip('PulseGen', '4.3GHz'); %1.5GHz, 4.3GHz 
%         radar.TryUpdateChip('PulseGenFineTune', pgen); 
%         radar.TryUpdateChip('SamplingRate', 0); %0 for 26 ps
% 
%         sampleDelayToReference = 1.5e-9; 
%         radarSpecifier = 'X1-IPGO'; 
%     end
% 
%     % Calibrate the radar module
%     tic
%     result = radar.ExecuteAction('MeasureAll');
%     toc
% 
%     % Now set the OffsetDistanceFromReference and/or SampleDelayToReference 
%     % NOTE -- these requires a calibration first!
% 
%     % Set the SampleDelayToReference (a value effected by antenna/cable choice)
%     radar.TryUpdateChip('SampleDelayToReference', sampleDelayToReference);
% 
%     % Get some radar values
%     iterations = radar.Item('Iterations');
%     offsetdistance = radar.Item('OffsetDistanceFromReference');
%     samplers = radar.Item('SamplersPerFrame');
%     [fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] = NoveldaChipParams(radarSpecifier, pgen,'4mm');
% 
%     resolution = radar.SamplerResolution;
%     range = linspace(0, samplers * resolution, samplers);
% 
%     % COLLECT FRAMES FOR DESIRED MAXTIME -----------------------------------------------------------
%     fpsTarget = 200; 
%     Ts = 1 / fpsTarget; 
%     maxTime = 10.00;  
%     maxFrames = fpsTarget * maxTime + 1; 
% 
%     frameCount = 0; 
%     timeStart = tic;
% 
%     fprintf('Capturing frames for %0.2f seconds', maxTime);
%     while (toc(timeStart) < maxTime) 
%         % Collect radar frame 
%         newFrame1 = radar.GetFrameNormalizedDouble;
%         captureTime = toc(timeStart); 
%         newFrame1 = newFrame1'; %column vector 
% 
%         if (frameCount == 0)
%             frameTot = zeros(size(newFrame1,1), maxFrames);
%             timeTot = zeros(1, maxFrames); 
%         end
% 
%         % Store frame and time data
%         frameCount = frameCount + 1;
%         frameTot(:,frameCount) = newFrame1; 
%         timeTot(frameCount) = captureTime;
% 
%         % Control fps
%         while (toc(timeStart) < frameCount * Ts)
%             %wait 
%         end
%     end
% 
%     % Truncate to last captured frame
%     frameTot = frameTot(:,1:frameCount); 
%     timeTot = timeTot(1:frameCount); 
%     timeTot = timeTot - timeTot(1); % set initial capture to t=0
% 
%     % Calculate fps 
%     timeElapsed = timeTot(end);
%     fpsEst = (size(frameTot,2) - 1) / timeElapsed;
%     fprintf('Estimated FPS: %f',fpsEst);
%     
%     % SAVE THE DATA --------------------------------------------------------------------------------
%     % TODO: save data 
%     if ~strcmp(savePath, 'default')
%         fprintf('Saving output to %s...\n',savePath)
%         save(sprintf(strcat(savePath,'expData%s.mat'),datestr(now,30)), ...
%             'maxTime','frameCount','frameTot','timeTot','fpsEst','radarType','pgen')
%     end
end
    
%% Post process the raw data 
% Baseband Conversion
frameTot_bb = zeros(size(frameTot)); 
for i = 1:frameCount
    frameTot_bb(:,i) = NoveldaDDC(frameTot(:,i), radarSpecifier, pgen, fs_hz); 
end

% FFT of signal for each bin
framesFFT = db(abs(fft(frameTot_bb,frameCount,2)));

% Analyze FPS
%analyzeFPS(timeTot, fpsEst); 

%% Plotting 
% Figure 1: FFT for each bin
figure(1); im = imagesc(framesFFT);
title('Radar response across all frequencies'); 
ylabel('Range bin'); 
xlabel('Frequency');

% Figure 2: FFT of bin with largest DC response???
[~,maxRangeIndex] = max(framesFFT(:,222)); 
figure(2); plot(framesFFT(maxRangeIndex,:));
title(sprintf('Radar response of bin %i across all frequencies', maxRangeIndex)); 
ylabel('Magnitude (dB)'); 
xlabel('Frequency');

% Figure 3: ???
framesDiff = diff(frameTot_bb,[],2); 
figure(3); imagesc(db(abs(fft(framesDiff,(frameCount-1),2)))); 
title('Differential radar response across all frequencies');
ylabel('Range bin');
xlabel('Frequency'); 

% Figure 4: FFT plot for bins ranging from firstBin to lastBin 
% TODO: Add ancho vs cayenne 
firstBin = 110;
lastBin = 240; 
figure(4); plot(framesFFT(firstBin:lastBin,:)') 
title(sprintf('Radar response, bins %i-%i', firstBin, lastBin))
ylabel('Magnitude (dB)')
xlabel('Frequency');

end
    

 