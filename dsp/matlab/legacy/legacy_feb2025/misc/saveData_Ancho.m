% Based off Ancho Example 1 test and Xethru XEP_X4_plot_frame.m

% usage: displays newly acquired or previous radar frame data

% Required args: 
% string to specify path to load or save data 
% bool to specify whether to load or save data 

% example 1: saveData_Ancho('/home/bradley/Documents/research/radar/matlab/AnchoData/', 1)
% captures frames for user-specified amount of time and displays data 

% example 2: saveData_Ancho('/home/bradley/Documents/research/radar/matlab/AnchoData/switchoff_openair_10s.mat', 0)
% loads frameTot, timeTot, and maxTime from .mat file and displays data

function saveData_Ancho(fileStr, saveOption)
    close all;
    clc;
    
    fs_hz = 39e9; 
    %% Get Data
    if (saveOption == 1) %Collect New Data
        % Create the radar object
        radar = radarWrapper('192.168.7.2');        %USB Cable
        %radar = radarWrapper('192.168.7.2', 1)      %USB Cable -- Force a software update
        %radar = radarWrapper('192.168.0.198');      %Ethernet IP Address example

        % Get a list of the connected modules
        modules = radar.ConnectedModules;

        % Open a connection to the radar module
        radar.Open(modules{1});

        % Set some register values 
        % Default settings: (common radarlib3 settings)
        % radar.TryUpdateChip('Iterations','50');
        % radar.TryUpdateChip('DACMin','0');
        % radar.TryUpdateChip('DACMax','8191');
        % radar.TryUpdateChip('DACStep','4');ans
        % radar.TryUpdateChip('PulsesPerStep','16');
        % radar.TryUpdateChip('FrameStitch','1');

        % Settings from Justin @ flatearthinc
        radar.TryUpdateChip('Iterations','16');
        radar.TryUpdateChip('DACMin','4100');
        radar.TryUpdateChip('DACMax','4700');
        radar.TryUpdateChip('DACStep','8');
        radar.TryUpdateChip('PulsesPerStep','8');
        radar.TryUpdateChip('FrameStitch','1');
        % Changing PRF: PRFDivide divides the default
        % 100Mhz PRF. So setting  to 2 yields PRF = 50Mhz
        %radar.TryUpdateChip('PRFDivide','6');
        
        % Set some Cayenne-specific radarlib3 settings (comment out if not using Cayenne)
        pgen = 0; 
        radar.TryUpdateChip('PGSelect', pgen);
        radar.SetVoltage(1.2);
        
        % Calibrate the radar module
        tic
        result = radar.ExecuteAction('MeasureAll');
        toc

        % Now set the OffsetDistanceFromReference and/or SampleDelayToReference 
        % NOTE -- these requires a calibration first!

        % Set the SampleDelayToReference (a value effected by antenna/cable choice)
        radar.TryUpdateChip('SampleDelayToReference',3.687e-9); % Ancho

        % Set the OffsetDistanceFromReference (frame begins at this distance from the reference)
        radar.TryUpdateChip('OffsetDistanceFromReference', 0.0);

        % Get some radar values
        iterations = radar.Item('Iterations');
        offsetdistance = radar.Item('OffsetDistanceFromReference');
        samplers = radar.Item('SamplersPerFrame');
        [fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] = NoveldaChipParams('X2', pgen, '4mm');
        
        resolution = radar.SamplerResolution;
        range = linspace(0, samplers * resolution, samplers);
        
        %Collect frames for desired amount of time 
        fpsTarget = 200; 
        tSample = 1 / fpsTarget; 
        maxTime = 10; %desired runtime 
        maxFrames = fpsTarget * maxTime + 1; 
        
        frameCount = 0; 
        timeStart = tic;
          
        while (1)
            newFrame1 = radar.GetFrameNormalizedDouble;
            newFrame1 = newFrame1'; %column vector 
            
            if (frameCount==0)
                frameTot = zeros(size(newFrame1,1), maxFrames);
                timeTot = zeros(1, maxFrames); 
            end
            
            if (toc(timeStart) >= maxTime)
                break
            end
            
            while (toc(timeStart) < frameCount * tSample)
                %wait 
            end

            frameCount = frameCount + 1; 
            frameTot(:,frameCount) = newFrame1; 
            timeTot(frameCount) = toc(timeStart);

        end

        %Truncate zero-entries (no data) 
        frameTot = frameTot(:,1:frameCount); 
        timeTot = timeTot(1:frameCount); 
        
        %Calculate fps 
        timeElapsed = timeTot(end);
        fps = (size(frameTot,2) - 1)/(timeElapsed);
        disp(['Estimated FPS: ' num2str(fps)]);

        %save the raw data
        fprintf('Saving output to %s...\n',fileStr)
        save(sprintf(strcat(fileStr,'expData%s.mat'),datestr(now,30)),'maxTime','frameTot','timeTot','fps','pgen')
        
        disp(['Read ' num2str(frameCount) ' frames']); 
    
    else %Load Data
        fprintf('Loading saved data from %s...\n', fileStr)
        %load(fileStr);
        FRAME_LOGGER_MAGIC_NUM = hex2dec('FEFE00A2');
        fid = fopen(fileStr,'r');
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

        % The measured sampling rate is next
        samplesPerSecond = fread(fid,1,'float');
        % The NVA6201 specific settings are next
        pgen = fread(fid,1,'int');
        % The frame offset, reference delay
        offsetDistance = fread(fid,1,'float');
        sampleDelayToReference = fread(fid,1,'float');
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

    %% Post Processing  
    
    %Baseband Conversion
    frameTot_bb = zeros(size(frameTot)); 
    for i = 1:frameCount
        frameTot_bb(:,i) = NoveldaDDC(frameTot(:,i), 'X2', pgen, fs_hz); 
    end

    %FFT of signal for each bin
    framesFFT = db(abs(fft(frameTot_bb,frameCount,2)));
    
    %% Plotting 
    
    %Figure 1: FFT for each bin
    figure(1); im = imagesc(framesFFT);
    title('Radar response across all frequencies'); 
    ylabel('Range bin'); 
    xlabel('Frequency');
    
    %Figure 2: FFT of bin with largest DC response???
    [~,maxRangeIndex] = max(framesFFT(:,222)); 
    figure(2); plot(framesFFT(maxRangeIndex,:));
    title(sprintf('Radar response of bin %i across all frequencies', maxRangeIndex)); 
    ylabel('Magnitude (dB)'); 
    xlabel('Frequency');
    
    %Figure 3: ???
    framesDiff = diff(frameTot_bb,[],2); 
    figure(3); imagesc(db(abs(fft(framesDiff,(frameCount-1),2)))); 
    title('Differential radar response across all frequencies');
    ylabel('Range bin');
    xlabel('Frequency'); 
    
    %Figure 4: FFT plot for bins ranging from firstBin to lastBin 
    firstBin = 200;
    lastBin = 256; 
    figure(4); plot(framesFFT(firstBin:lastBin,:)') 
    title(sprintf('Radar response, bins %i-%i', firstBin, lastBin))
    ylabel('Magnitude (dB)')
    xlabel('Frequency');
    delta = zeros(length(times)-1, 1);
    for i=length(times)-1:-1:1
        delta(i) = times(i+1)-times(i);
    end
    sum(abs(delta-1/frameRate))
end



