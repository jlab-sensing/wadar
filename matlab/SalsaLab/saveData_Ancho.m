% Based off Ancho Example 1 test and Xethru XEP_X4_plot_frame.m

% usage: 

% Brad's current savePath: savepath = '/home/bradley/Documents/research/radar/matlab/AnchoData/'

function saveData_Ancho(savePath)
    close all;

    %% Create the radar object
    radar = radarWrapper('192.168.7.2');        %USB Cable
    %radar = radarWrapper('192.168.7.2', 1)      %USB Cable -- Force a software update
    %radar = radarWrapper('192.168.0.198');      %Ethernet IP Address example

    %% Get a list of the connected modules
    modules = radar.ConnectedModules;

    %% Open a connection to the radar module
    radar.Open(modules{1});

    %% Set some register values 
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

    %% Set some Ancho-specific radarlib3 settings 
    radar.TryUpdateChip('PGSelect', 7);
    radar.SetVoltage(1.2);

    %% Calibrate the radar module
    tic
    result = radar.ExecuteAction('MeasureAll');
    toc

    %% Now set the OffsetDistanceFromReference and/or SampleDelayToReference 
    % NOTE -- these requires a calibration first!

    % Set the SampleDelayToReference (a value effected by antenna/cable choice)
    radar.TryUpdateChip('SampleDelayToReference',3.687e-9); % Ancho

    % Set the OffsetDistanceFromReference (frame begins at this distance from the reference)
    radar.TryUpdateChip('OffsetDistanceFromReference', 0.0);

    %% Get some register values
    iterations = radar.Item('Iterations');
    offsetdistance = radar.Item('OffsetDistanceFromReference');
    samplers = radar.Item('SamplersPerFrame');

    %% Get the CDF
    % cdf = radar.getCDF();

    %% Collect a bunch of raw frames and compute the average FPS
    %subplot(1,1,1);

    % Loop for desired time 
    fpsMax = 500; %maximum expected fps
    maxTime = 5;  

    i=1; 
    timeStart = tic;
    while (1)
        %Get normalized radar frame 
        newFrame1 = radar.GetFrameNormalizedDouble;
        newFrame1 = newFrame1'; %column vector 

        %Plot radar frame 
        %plot(newFrame1)
        %drawnow

        %On first iteration, initialize matrix to store frames and times
        if (i==1)
            frameTot = zeros(size(newFrame1,1), fpsMax * maxTime);
            timeTot = zeros(1, fpsMax * maxTime); 
        end

        %Store frame and time data
        frameTot(:,i) = newFrame1; 
        timeTot(i) = toc(timeStart);

        %Exit if maxTime is exceeded
        if (toc(timeStart) > maxTime)
            break
        end

        i = i + 1;
    end
    
    %Get rid of non-data entries 
    frameTot = frameTot(:,1:i); 
    timeTot = timeTot(1:i); 
    timeElapsed = timeTot(end);
    
    %Calculate FPS 
    fpsRaw = size(frameTot,2)/(timeElapsed);
    
    %save the raw data
    %fprintf('Saving output to %s...\n',savePath)
    %save(sprintf(strcat(savepath,'expData%s.mat'),datestr(now,30)),'maxTime','frameTot','timeTot')

    %% Post Processing  
    %TO-DO- Edit this description 
    %frameTot((end/2 + 1):end, 1:i) =  1i* frameTot((end/2 + 1):end, 1:i);
    
    %FFT of signal for each bin
    framesFFT = db(abs(fft(frameTot,i,2)));
    Fs = fpsRaw; %average FPS approximates the frame sampling rate
    
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
    
    %Figure 3: ???
    framesDiff = diff(frameTot,[],2); 
    figure(3); imagesc(db(abs(fft(framesDiff,(i-1),2)))); 
    title('Differential radar response across all frequencies');
    ylabel('Range bin');
    xlabel('Frequency (units?)'); 
    
    %Figure 4: FFT plot for bins ranging from firstBin to lastBin 
    firstBin = 11;
    lastBin = 20; 
    figure(4); plot(framesFFT(firstBin:lastBin,:)') 
    title(sprintf('Radar response, bins %i-%i', firstBin, lastBin))
    ylabel('Magnitude')
    xlabel('Frequency (units?)');
end

