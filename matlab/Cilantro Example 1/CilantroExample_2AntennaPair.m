% Example to switch b/w 2 Antenna Pairs Using the Cilantro Switching Cape
% Connect to BBB and Radar Capes
radar = radarWrapper;
modules = radar.ConnectedModules;
radar.Open(modules{1});

% Initializes Cilantro Cape
radar.Cilantro_Init();

% Push Reference Plan out by 0.5 meters
radar.TryUpdateChip('OffsetDistanceFromReference', '0.5');

% Framestitch 2 frames
radar.TryUpdateChip('FrameStitch', '2');

%Calibrate the radar module
radar.ExecuteAction('MeasureAll');

% Get Radar Framesize
N = radar.Item('SamplersPerFrame');

% Initialize Plot
figure;
sp1 = subplot(211);
h1 = plot(sp1, 1:N, nan(1, N));
sp2 = subplot(212);
h2 = plot(sp2, 1:N, nan(1, N));
title(sp1, 'Cilantro Raw Data: Tx1Rx1');
title(sp2, 'Cilantro Raw Data: Tx2Rx2');
xlim(sp1, [1 N]);
xlim(sp2, [1 N]);

% Switch between TxRx1/TxRx2 @ ~0.5Hz for ~10 secs
tic;
for i = 1:20
    % Sets Tx1Rx1
    radar.Cilantro_SelectOutputChannel(1);
    
    % Get Frame and Plot
    set(h1, 'ydata', radar.GetFrameRawDouble(), 'color', 'k'); 
    pause(0.25);
    
    % Sets Tx2Rx2
    radar.Cilantro_SelectOutputChannel(2)   
    
    % Get Frame and Plot
    set(h2, 'ydata', radar.GetFrameRawDouble(), 'color', 'r');  
    pause(0.25)
end
toc;