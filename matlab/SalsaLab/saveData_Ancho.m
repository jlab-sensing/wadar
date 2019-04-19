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
        radar.TryUpdateChip('DACMin','3800');
        radar.TryUpdateChip('DACMax','4900');
        radar.TryUpdateChip('DACStep','8');
        radar.TryUpdateChip('PulsesPerStep','8');
        radar.TryUpdateChip('FrameStitch','1');

        % Set some Ancho-specific radarlib3 settings 
        radar.TryUpdateChip('PGSelect', 7);
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
        
        resolution = radar.SamplerResolution
        range = linspace(0, samplers * resolution, samplers)

        % Get the CDF
        % cdf = radar.getCDF();
        
        %Collect frames for desired amount of time 
        %subplot(1,1,1);
        fpsMax = 500; %Should be > maximum possible fps
        maxTime = 5; %desired runtime 

        frameCount=1; 
        timeStart = tic;
         
        while (1)
            newFrame1 = radar.GetFrameNormalizedDouble;
            newFrame1 = newFrame1'; %column vector 

            %plot(newFrame1)
            %drawnow
            
            if (frameCount==1)
                frameTot = zeros(size(newFrame1,1), fpsMax * maxTime);
                timeTot = zeros(1, fpsMax * maxTime); 
            end

            frameTot(:,frameCount) = newFrame1; 
            timeTot(frameCount) = toc(timeStart);

            if (toc(timeStart) > maxTime)
                break
            end

            frameCount = frameCount + 1;
        end

        %Truncate zero-entries (no data) 
        frameTot = frameTot(:,1:frameCount); 
        timeTot = timeTot(1:frameCount); 

        %save the raw data
        %fprintf('Saving output to %s...\n',path)
        %save(sprintf(strcat(path,'expData%s.mat'),datestr(now,30)),'maxTime','frameTot','timeTot')
        
        disp(['Read ' num2str(frameCount) ' frames']); 
    
    else %Load Data
        fprintf('Loading saved data from %s...\n', fileStr)
        load(fileStr);
        frameCount = length(frameTot);
    end

    %% Post Processing  
    %Calculate fps 
    timeElapsed = timeTot(end);
    fpsRaw = size(frameTot,2)/(timeElapsed);
    disp(['Estimated FPS: ' num2str(fpsRaw)]);
    
    %Baseband Conversion???

    %FFT of signal for each bin
    framesFFT = db(abs(fft(frameTot,frameCount,2)));
    Fs = fpsRaw; %average FPS approximates sampling rate, assuming consistent time between samples
    
    %% Plotting 
    %Figure 1: FFT for each bin
    figure(1); im = imagesc(framesFFT);
    title('Radar response across all frequencies'); 
    ylabel('Range bin'); 
    xlabel('Frequency (units?)');
    
    %Figure 2: FFT of bin with largest DC response???
    [~,maxRangeIndex] = max(framesFFT(:,1)); 
    figure(2); plot(framesFFT(maxRangeIndex,:));
    title(sprintf('Radar response of bin %i across all frequencies', maxRangeIndex)); 
    ylabel('Magnitude (dB)'); 
    xlabel('Frequency (units?)');
    
    %Figure 3: ???
    framesDiff = diff(frameTot,[],2); 
    figure(3); imagesc(db(abs(fft(framesDiff,(frameCount-1),2)))); 
    title('Differential radar response across all frequencies');
    ylabel('Range bin');
    xlabel('Frequency (units?)'); 
    
    %Figure 4: FFT plot for bins ranging from firstBin to lastBin 
    firstBin = 11;
    lastBin = 20; 
    figure(4); plot(framesFFT(firstBin:lastBin,:)') 
    title(sprintf('Radar response, bins %i-%i', firstBin, lastBin))
    ylabel('Magnitude (dB)')
    xlabel('Frequency (units?)');
end

