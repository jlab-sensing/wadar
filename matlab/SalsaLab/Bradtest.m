% This script initializes the radar and collects and displays radar frames
% as a BScan plot.  There is not processing or clutter removal in this
% demo application.
%
% Copyright Flat Earth Inc. 2016
clc; 
clear all;
close all;

%% Specify the savepath
savepath = '/home/bradley/Documents/research/radar/matlab/AnchoData/'; 

%% Create the radar object
radar = radarWrapper('192.168.7.2');        %USB Cable
%radar = radarWrapper('192.168.7.2', 1)      %USB Cable -- Force a software update
%radar = radarWrapper('192.168.0.198');      %Ethernet IP Address example

%% Get a list of the connected modules
modules = radar.ConnectedModules;

%% Open a connection to the radar module
radar.Open(modules{1});

%% Set some register values (common radarlib3 settings)
% radar.TryUpdateChip('Iterations','50');
% radar.TryUpdateChip('DACMin','0');
% radar.TryUpdateChip('DACMax','8191');
% radar.TryUpdateChip('DACStep','4');
% radar.TryUpdateChip('PulsesPerStep','16');
% radar.TryUpdateChip('FrameStitch','1');

radar.TryUpdateChip('Iterations','16');
radar.TryUpdateChip('DACMin','3800');
radar.TryUpdateChip('DACMax','4900');
radar.TryUpdateChip('DACStep','8');
radar.TryUpdateChip('PulsesPerStep','8');
radar.TryUpdateChip('FrameStitch','1');

%% Set some radar cape specific settings

% Set some Ancho-specific radarlib3 settings (comment out if not using Ancho)
radar.TryUpdateChip('PGSelect', 7);
radar.SetVoltage(1.2);

% Set some Cayenne-specific radarlib3 settings (comment out if not using Cayenne)
%radar.TryUpdateChip('PulseGen', '4.3GHz');
%radar.TryUpdateChip('SamplingRate', 0);

% Set some Chipotle-specific radarlib3 settings (comment out if not using Chipotle)
%radar.TryUpdateChip('PulseGen', '1.5GHz');
%radar.TryUpdateChip('SamplingRate', 0);

%% Calibrate the radar module
tic
result = radar.ExecuteAction('MeasureAll');
toc

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
timeStart = tic;
subplot(1,1,1);
clutter = zeros(1,samplers);

% Loop for desired time 
fpsMax = 500; %maximum expected fps
maxTime = 10;  

i=1; 
while (1)
    newFrame1 = radar.GetFrameNormalizedDouble;
    newFrame1 = newFrame1'; %column vector 
    %plot(newFrame1)
    %drawnow
    
    if (i==1)
        frameTot = zeros(size(newFrame1,1), fpsMax * maxTime);
        timeTot = zeros(1, fpsMax * maxTime); 
    end
    
    frameTot(:,i) = newFrame1; 
    timeTot(i) = toc(timeStart);
    
    if (toc(timeStart) > maxTime)
        break
    end
    i = i + 1;
end
frameTot = frameTot(:,1:i); 
timeTot = timeTot(:,1:i); 
timeElapsed = timeTot(end); 

timeTot = toc; 
fpsRaw = size(frameTot,2)/(timeElapsed)

%save the data
fprintf('Saving output to %s...\n',savepath)
save(sprintf(strcat(savepath,'expData%s.mat'),datestr(now,30)),'maxTime','frameTot','timeTot')
    
