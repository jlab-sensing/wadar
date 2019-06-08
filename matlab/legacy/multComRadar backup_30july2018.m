% Collecting radar measurements from multiple RADARs

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

portNoAll = [7:10  11 14:16];  % port numbers where Radars are seen
nRx = length(portNoAll);
COM_portsAll = cell(nRx,1);
frameTotCell = cell(nRx,1);
timeTotCell = cell(nRx,1);
for iRx = 1:nRx
    COM_portsAll{iRx} = sprintf('COM%d',portNoAll(iRx));
end

for iRx = 1:nRx
    % Input parameters
    COM = COM_portsAll{iRx};
    FPS = 400; %500;
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

    FPS_est = framesRead/tspent;

    framesDropped = ctr-i;
    
    disp(['Read ' num2str(framesRead) ' frames. A total of ' num2str(totFramesFromChip) ' were sent from chip. Frames dropped: ' num2str(framesDropped)]);
    disp(['Estimated FPS: ' num2str(FPS_est) ', should be: ' num2str(FPS)]);

    radar.close();
    clear radar frame

    % save(sprintf('C:\\Users\\snsgOvr\\Dropbox\\XethruData\\ExpData%s.mat',datestr(now,30)),'-v7.3','frameTot','timeTot')


    %% plot
    figure(5); plot(a(11:23,:)')
    % figure(6); plot(a(:,5121)')
    % figure(6); plot(a(:,256)')
    figure(6); plot(a(:,25600)')
    % figure(6); plot(a(:,255994)')
    title(sprintf('for COM%d',portNoAll(iRx)))

    frameTotCell{iRx} = frameTot;
    timeTotCell{iRx} = timeTot;
    
    pause(1);
end

load audio48, sound(signal48kHz, Fs48); clear Fs48 signal48kHz
save(sprintf('C:\\Users\\snsgOvr\\Dropbox\\XethruData\\ExpData%s.mat',datestr(now,30)),'-v7.3','frameTotCell','timeTotCell')
load audio48, sound(signal48kHz, Fs48)