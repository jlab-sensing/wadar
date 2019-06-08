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

portNoAll = [7 8 9 10  11 14:16];  % port numbers where Radars are seen
nRx = length(portNoAll);
COM_portsAll = cell(nRx,1);
frameTotCell = cell(nRx,1);
timeTotCell = cell(nRx,1);
for iRx = 1:nRx
    COM_portsAll{iRx} = sprintf('COM%d',portNoAll(iRx));
end


% % % saving only certain frequencies
freqIndSave = [4858 6401 7944 16115 19201 20743 23829 32000 33543 35086 ... % frequencies for 64 Hz tag
                    3087 14344 17999 21945 25600 36858];
limitedFftMat = cell(nRx,1);
correlatedFrame = cell(nRx,1);
correlatedFrame_sin = cell(nRx,1);
correlatedFrame_cos = cell(nRx,1);
correlatedFrame_64 = cell(nRx,1);
correlatedFrame_sin_64 = cell(nRx,1);
correlatedFrame_cos_64 = cell(nRx,1);


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
    
    fftSpeedDim = fft(frameTot,[],2);
    limitedFftMat{iRx} = fftSpeedDim(:,freqIndSave);
    % for 256 Hz
    dopplerIndExpected = 25600;
    cdma_sig_way6 = sign(sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + 1e-8));
    cdma_sig_way7 = (sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + 1e-8));
    cdma_sig_way8 = (cos(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + 1e-8));
    cdma_sig_rep = repmat(cdma_sig_way6, size(frameTot,1), 1);
    correlatedFrame{iRx} = sum(frameTot.*cdma_sig_rep,2);
    cdma_sig_rep_sin = repmat(cdma_sig_way7, size(frameTot,1), 1);
    correlatedFrame_sin{iRx} = sum(frameTot.*cdma_sig_rep_sin,2);
    cdma_sig_rep_cos = repmat(cdma_sig_way8, size(frameTot,1), 1);
    correlatedFrame_cos{iRx} = sum(frameTot.*cdma_sig_rep_cos,2);
    % for 64 Hz
    dopplerIndExpected = 6401;
    cdma_sig_way6 = sign(sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + 1e-8));
    cdma_sig_way7 = (sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + 1e-8));
    cdma_sig_way8 = (cos(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + 1e-8));
    cdma_sig_rep = repmat(cdma_sig_way6, size(frameTot,1), 1);
    correlatedFrame_64{iRx} = sum(frameTot.*cdma_sig_rep,2);
    cdma_sig_rep_sin = repmat(cdma_sig_way7, size(frameTot,1), 1);
    correlatedFrame_sin_64{iRx} = sum(frameTot.*cdma_sig_rep_sin,2);
    cdma_sig_rep_cos = repmat(cdma_sig_way8, size(frameTot,1), 1);
    correlatedFrame_cos_64{iRx} = sum(frameTot.*cdma_sig_rep_cos,2);
    
    
    pause(1);
end

load audio48, sound(signal48kHz, Fs48); clear Fs48 signal48kHz
fileExtension = datestr(now,30);
save(sprintf('D:\\XethruData\\ExpData%s.mat',fileExtension),'-v7.3','frameTotCell','timeTotCell')
% % save limited data
save(sprintf('D:\\LimitedXethruData\\LimitedExpData%s.mat',fileExtension),'-v7.3','limitedFftMat','correlatedFrame','correlatedFrame_sin','correlatedFrame_cos',...
     'correlatedFrame_64','correlatedFrame_sin_64','correlatedFrame_cos_64','freqIndSave');
load audio48, sound(signal48kHz, Fs48)
