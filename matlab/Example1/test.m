% This script initializes the radar and collects and displays radar frames
% as a BScan plot.  There is not processing or clutter removal in this
% demo application.
%
% Copyright Flat Earth Inc. 2016

%%Create the radar object
radar = radarWrapper('192.168.7.2');        %USB Cable
%radar = radarWrapper('192.168.7.2', 1)      %USB Cable -- Force a software update
%radar = radarWrapper('192.168.0.198');      %Ethernet IP Address example

%% Get a list of the connected modules
modules = radar.ConnectedModules;

%% Open a connection to the radar module
radar.Open(modules{1});

%% Set some register values (common radarlib3 settings)
radar.TryUpdateChip('Iterations','50');
radar.TryUpdateChip('DACMin','0');
radar.TryUpdateChip('DACMax','8191');
radar.TryUpdateChip('DACStep','4');
radar.TryUpdateChip('PulsesPerStep','16');
radar.TryUpdateChip('FrameStitch','1');

%% Set some radar cape specific settings

% Set some Ancho-specific radarlib3 settings (comment out if not using Ancho)
% radar.TryUpdateChip('PGSelect', 7);
% radar.SetVoltage(1.2);

% Set some Cayenne-specific radarlib3 settings (comment out if not using Cayenne)
%radar.TryUpdateChip('PulseGen', '4.3GHz');
%radar.TryUpdateChip('SamplingRate', 0);

% Set some Chipotle-specific radarlib3 settings (comment out if not using Chipotle)
%radar.TryUpdateChip('PulseGen', '1.5GHz');
%radar.TryUpdateChip('SamplingRate', 0);

%% Calibrate the radar module
% tic
% result = radar.ExecuteAction('MeasureAll');
% toc

%% Now set the OffsetDistanceFromReference and/or SampleDelayToRefernce 
% NOTE -- these requires a calibration first!

% Set the SampleDelayToReference (a value effected by antenna/cable choice)
radar.TryUpdateChip('SampleDelayToReference',3.687e-9); % Ancho
%radar.TryUpdateChip('SampleDelayToReference',1.5e-9);   % Cayenne
%radar.TryUpdateChip('SampleDelayToReference',1.5e-9);   % Chipotle -- todo (update this!)

% Set the OffsetDistanceFromReference (frame begins at this distance from
% the reference)
radar.TryUpdateChip('OffsetDistanceFromReference', 0.0);

%% Get some register values
iterations = radar.Item('Iterations');
offsetdistance = radar.Item('OffsetDistanceFromReference');
samplers = radar.Item('SamplersPerFrame');

%% Get the CDF
% cdf = radar.getCDF();

%% Collect a bunch of raw frames and compute the average FPS
tic;
t1=toc;
subplot(1,1,1);
plotTime = 20;      %Run the plot for this many seconds
fpsFrames = 0;      %Number of frames collected in the time period
clutter = zeros(1,samplers);
while (1)
    fpsFrames= fpsFrames+1;
    newFrame1 = double(radar.GetFrameRaw);
    plot(newFrame1)
    drawnow
    if (toc>plotTime)
        break
    end
    
end
t2=toc;
FPS_RAW = fpsFrames/(t2-t1)
