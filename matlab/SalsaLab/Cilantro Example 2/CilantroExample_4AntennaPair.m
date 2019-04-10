% For modified Cilantro, with 4 Antenna Pair Combinations
% Connect to BBB and Radar Capes
radar = radarWrapper;
modules = radar.ConnectedModules;
radar.Open(modules{1});

% Initializes Cilantro Cape
radar.Cilantro_Init();    
radar.TryUpdateChip('OffsetDistanceFromReference', '0');
radar.TryUpdateChip('SampleDelayToReference', '0');
radar.TryUpdateChip('FrameStitch', '2');

%Calibrate the radar module
radar.ExecuteAction('MeasureAll');

% Get Vp max/min
frame = radar.GetFrameRawDouble();
frameSize = length(frame);
Vpmax = max(frame) * 1.5;
Vpmin = min(frame) * 0.4;

% Set up Figure
figure;
sp1 = subplot(221);
sp2 = subplot(222); 
sp3 = subplot(223);
sp4 = subplot(224); 

% Switch between TxRx @ ~1Hz
tic;
for i = 1:10
    % For Cilantro (Set Rx1 & Tx1)
    radar.WriteIOPin(9, 25, 'low');
    radar.WriteIOPin(9, 27, 'low');
    
    % Get Frame & Plot
    plot(sp1, radar.GetFrameRawDouble()); 
    ylim(sp1, [Vpmin Vpmax]);
    xlim(sp1, [1 frameSize]);
    grid(sp1, 'on');  
    title(sp1, 'Rx1-Tx1');
    pause(0.2);
    
    % For Cilantro (Set Rx2 & Tx2)
    radar.WriteIOPin(9, 25, 'high');
    radar.WriteIOPin(9, 27, 'high');
    
    % Get Frame & Plot
    plot(sp2, radar.GetFrameRawDouble());
    ylim(sp2, [Vpmin Vpmax]);
    xlim(sp2, [1 frameSize]);
    grid(sp2, 'on');  
    title(sp2, 'Rx2-Tx2');
    pause(0.2);
    
    % For Cilantro (Set Rx1 & Tx2)
    radar.WriteIOPin(9, 25, 'low');
    radar.WriteIOPin(9, 27, 'high');
    
    % Get Frame & Plot
    plot(sp3, radar.GetFrameRawDouble()); 
    ylim(sp3, [Vpmin Vpmax]);
    xlim(sp3, [1 frameSize]);
    grid(sp3, 'on');  
    title(sp3, 'Rx1-Tx2');
    pause(0.2)
    
    % For Cilantro (Set Rx2 & Tx1)
    radar.WriteIOPin(9, 25, 'high');
    radar.WriteIOPin(9, 27, 'low');
    
    % Get Frame & Plot
    plot(sp4, radar.GetFrameRawDouble());
    ylim(sp4, [Vpmin Vpmax]);
    xlim(sp4, [1 frameSize]);
    grid(sp4, 'on');  
    title(sp4, 'Rx2-Tx1');
    pause(0.2)
end
toc;
