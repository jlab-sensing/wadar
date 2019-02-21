% This example demonstrates the use of the BasicRadarClassX4 example class
% to configure the chip and start streaming.
%
% To complete the following example you need:
% - An X4M200/X4M300/X4M03 module
% - The ModuleConnector library
% - MATLAB

try
    radar.close();
catch
    
end

%add paths
addpath('../../matlab/');
addpath('../../include/');
addpath('../../lib64/');
% or addpath('../../lib32/');

clc
close all
clear

% Load the library
Lib = ModuleConnector.Library;
Lib.libfunctions;

% Input parameters
COM = 'COM19';
FPS = 450; %400; %500;
dataType = 'bb';

% Chip settings
PPS = 10;
DACmin = 949;
DACmax = 1100;
Iterations = 16;
FrameStart = 0.2; % meters.
FrameStop = 9.4; % meters.
% default values for FrameStart and FrameStop are 0.2 m and 9.4 m.



% % Initialize radar.
exFlag = 0;  % flag the occurence of failed radar initialization. The radar frames obtained after the failed initialization are good.
while exFlag == 0
    try 
        [radar, app] = startRadar(COM,FPS,dataType);
        radar.init();
        radar.close();
    catch
        exFlag = 1
        [radar, app] = startRadar(COM,FPS,dataType);
        radar.init();
    end
end

% Configure X4 chip.
radar.radarInstance.x4driver_set_pulsesperstep(PPS);
radar.radarInstance.x4driver_set_dac_min(DACmin);
radar.radarInstance.x4driver_set_dac_max(DACmax);
radar.radarInstance.x4driver_set_iterations(Iterations);

% Configure frame area
radar.radarInstance.x4driver_set_frame_area(FrameStart,FrameStop);
% Read back actual set frame area
[frameStart, frameStop] = radar.radarInstance.x4driver_get_frame_area();

% Start streaming and subscribe to message_data_float.
radar.start();

tstart = tic;
tspent = toc(tstart);

i = 0;
maxTime = 100;
% frameTot = zeros(181, FPS*maxTime);
while (tspent < maxTime)
    % Peek message data float
    numPackets = radar.bufferSize();
    if numPackets > 0
        i = i+1;
        % Get frame (uses read_message_data_float)
        [frame, ctr] = radar.GetFrameNormalized();
        
        % timestamp for each collected frame
        if i == 1
            timeTot = zeros(1, FPS*maxTime); 
        end
        timeTot(i) = toc(tstart);
        
        if i == 1
            numBins = length(frame);
            numBins = numBins/2;
            binLength = (frameStop-frameStart)/(numBins-1);
            rangeVec = (0:numBins-1)*binLength + frameStart;
        end
        
        frame = frame(1:end/2) + 1i*frame(end/2 + 1:end);
       
        if i == 1
           frameTot = zeros(size(frame,1), FPS*maxTime);
        end
        frameTot(:,i) = frame;
    end
       
    tspentPrev = tspent;
    tspent = toc(tstart);
end

% Stop streaming.
radar.stop();

frameTot = frameTot(:,1:i);
timeTot = timeTot(:,1:i);
figure(1); imagesc(db(abs(fft(frameTot,i,2))))
a = (db(abs(fft(frameTot,i,2))));
[~,maxRangeIndex] = max(a(:,1)); 
figure(2); plot(a(maxRangeIndex,:));
figure(3); imagesc(db(abs(fft(diff(frameTot,[],2),i,2))))

% Output short summary report.
framesRead = i;
totFramesFromChip = ctr;

FPS_est = framesRead/tspentPrev;

framesDropped = ctr-i;

disp(['Read ' num2str(framesRead) ' frames. A total of ' num2str(totFramesFromChip) ' were sent from chip. Frames dropped: ' num2str(framesDropped)]);
disp(['Estimated FPS: ' num2str(FPS_est) ', should be: ' num2str(FPS)]);

radar.close();
clear radar frame

save(sprintf('C:\\Users\\snsgOvr\\Dropbox\\XethruData\\ExpData%s.mat',datestr(now,30)),'-v7.3','frameTot','timeTot')


%% plot
figure(5); plot(a(7:15,:)')
% figure(6); plot(a(:,5121)')
% figure(6); plot(a(:,2561)')
% figure(6); plot(a(:,25600)')
% figure(6); plot(a(:,255:257))
% figure(6); plot(a(:,255994)')
figure(6); plot(a(:,25600)')

    rangeBinConsider = 10;
    iRx = 1;
    dopplerIndExpected = 25600;
    shiftRadTot = linspace(-0.3*pi, 2.3*pi, 197);
    radarDataOversampleForMaxIdx = zeros(size(frameTot,1), length(shiftRadTot));
    for iAngShift = 1:length(shiftRadTot)
        cdma_sig = sign(sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + shiftRadTot(iAngShift)));
        correlatedFrameTmp = frameTot*cdma_sig.'; %sum(frameTot.*cdma_sig_rep,2);
        radarDataOversampleForMaxIdx(:,iAngShift) = correlatedFrameTmp;
    end
    radarDataOversampleForMaxIdxAllRx{iRx} = radarDataOversampleForMaxIdx;
    radarIqCorrelated =  radarDataOversampleForMaxIdxAllRx{iRx};
    [~,timeShiftConsider] = max(abs(radarIqCorrelated(rangeBinConsider,:)));
    radarDataConsider = radarDataOversampleForMaxIdxAllRx{iRx}(:,timeShiftConsider);
    figure(7); plot(abs(radarDataConsider)); hold on; plot(abs(radarDataOversampleForMaxIdxAllRx{iRx}(:,1))); hold off;
    
    
    
    
