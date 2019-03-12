%% 
clear; close all; 

tagFreq = 256;
radarFreq = 400;
rangeRes = 0.05; % in m
c = 3e8; % speed of light
lambda = c/7.5e9; % wavelength in m

% % % experiments in pipe at a single location
% dataDir = './fourRadarsPipe';
% startFile = 'ExpData20180705T234454';
% expFileOffsetTot = 1:12; % 1:6;

% dataDir = './locChangeRange';
% filenamesDataTmp = importdata('./locChangeRangeNames.xlsx');
% filenamesData = filenamesDataTmp.textdata.x76(2:end,2);
% expFileOffsetTot = [1:30 32:size(filenamesData,1)];

% dataDir = './pseudoNlos';
% filenamesDataTmp = importdata('./nlosPseudoNames.csv');
% filenamesData = filenamesDataTmp.textdata(2:end,2);
% expFileOffsetTot = [1:3];

% dataDir = './nlos1';
% filenamesDataTmp = importdata('./nlos1Names.csv');
% filenamesData = filenamesDataTmp.textdata(2:end,2);
% expFileOffsetTot = [1:8];

% dataDir = './nlosTurkeySteakAgain';
% importdata('./nlosTurkeySteakAgainNames.csv')
% filenamesData = filenamesDataTmp.textdata(2:end,2);
% expFileOffsetTot = [1:12];

% dataDir = './agarPos';
% startFile = 'ExpData20180713T002200';
% expFileOffsetTot = 1:4; % 1:6;

% dataDir = './agarPos';
% startFile = 'ExpData20180714T155912';
% expFileOffsetTot = 1:27; % 1:6;

% dataDir = './farAgar';
% startFile = 'ExpData20180718T212221';
% expFileOffsetTot = [1:3]; % 1:6;

% dataDir = './onOffFarAgar';
% startFile = 'ExpData20180719T181428';
% expFileOffsetTot =  [14:2:24]; [1 3 6:2:12];

% dataDir = './onOffNearAgar';
% startFile = 'ExpData20180723T214001';
% expFileOffsetTot = 17:19; [1 3 6 7 9 11]; [2 4 5 8 10 12];

% dataDir = './agarSweep';
% startFile = 'ExpData20180724T201620';
% expFileOffsetTot = 50:110; [38 45:49]; 38:44; [21 23 24 27:2:37]; [2:2:8 11 13 14 17 19];

dataDir = 'D:\\XethruData\\';
startFile = 'ExpData20180729T194014';
expFileOffsetTot = 1:159; 1:10; 
                   %101:159; 62:100; 1:41;



nRx = 8;
%%
dopplerIndExpectedTot = 25600; [25600 6401]; 25600; 5121; 2561; 25600; 255994;  %(tagFreq/radarFreq)*size(fftSpeedDim,2); % estimated shift in doppler

for iExp = 1:length(expFileOffsetTot)
    expFileOffset = expFileOffsetTot(iExp);
        
    filenames = dir( sprintf('%s/Exp*.mat',dataDir));
    ExpStartFileIdx = strmatch(startFile,{filenames(:).name}) -1 + expFileOffset;
    filename = filenames(ExpStartFileIdx).name;
    % filename = filenamesData{expFileOffset};

    radarData = load([dataDir '/' filename]);
    if length(radarData.frameTotCell)<nRx
        frameTotCellTmp = radarData.frameTotCell;
        radarData.frameTotCell(4:8) = frameTotCellTmp(3:7);
    end
        
    correlatedFrame = cell(nRx,1);
    correlatedFrame_64 = cell(nRx,1);
    correlatedFrameMedMax = cell(nRx,1);
    correlatedFrameMedMax_64 = cell(nRx,1);
    correlatedFrameBestLobe = cell(nRx,1);
    correlatedFrameBestLobe_64 = cell(nRx,1);
    absMetricAllRx = cell(nRx,1);
    absMetricMedMaxAllRx = cell(nRx,1);
    absMetricBestLobeAllRx = cell(nRx,1);
    maxIdxTotAllRx = cell(nRx,1);
    medMaxIdxAllRx = cell(nRx,1);
    mainFreqEnergyAllRx = cell(nRx,1);
    maxMainEnergyIdxAllRx = cell(nRx,1);
    radarDataOversampleForMaxIdxAllRx = cell(nRx,1);

    for iRx = 1:nRx        
        frameTot = radarData.frameTotCell{iRx};

        for dopplerIndExpected = dopplerIndExpectedTot
            shiftRadTot = linspace(-0.3*pi, 2.3*pi, 197);
            absMetric = zeros(size(shiftRadTot));
            angMetric = zeros(size(shiftRadTot));
            maxIdxTot = zeros(size(shiftRadTot));
            radarDataOversampleForMaxIdx = zeros(size(frameTot,1), length(shiftRadTot));
            for iAngShift = 1:length(shiftRadTot)
                cdma_sig_way6 = sign(sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + shiftRadTot(iAngShift)));
                cdma_sig = cdma_sig_way6; % cdma_sig_way1; default is cdma_sig_way1
                %cdma_sig_rep = repmat(cdma_sig, size(frameTot,1), 1);
                correlatedFrameTmp = frameTot*cdma_sig.'; %sum(frameTot.*cdma_sig_rep,2);

                % % % % finding the range bin of the tag naively by ignoring the first three radar samples
                % [absMetric(iAngShift), maxIdx] = max(abs(correlatedFrame(4:end)));
                % maxIdx = maxIdx + 3;
                % angMetric(iAngShift) = angle(correlatedFrame(maxIdx));

                % % % % finding the range bin by finding the local maxima that is beyond the first few samples
                overSampleFactor = 1;
                radarDataOverSample = correlatedFrameTmp;
                minRangeThr = 1.0;
                radarDataOverSample(1:round(minRangeThr*overSampleFactor),:) = 1e-6; % small tolerance value
                maxRangeThr = 27;
                radarDataOverSample(maxRangeThr*overSampleFactor+1:end,:) = 1e-6; % small tolerance value
                % % looking at regional maximum
                [pks,locs] = findpeaks(abs(radarDataOverSample));
                minRangeInd = minRangeThr*overSampleFactor+1;
                maxRangeInd = maxRangeThr*overSampleFactor;
                pks(locs==minRangeInd) = []; locs(locs==minRangeInd) = []; % ignore the first peak as it is caused by sudden jump of radar data from tolerance threhold of 1e-6 to actul radar data values.
                pks(locs==maxRangeInd) = []; locs(locs==maxRangeInd) = []; 
                % % finding the good row based on the maximum of the peaks
                maxIdx = locs(pks==max(pks)); 
                absMetric(iAngShift) = max(pks);
                angMetric(iAngShift) = angle(correlatedFrameTmp(maxIdx));
                maxIdxTot(iAngShift) = maxIdx;

                figure(1); plot(abs(correlatedFrameTmp)); 
                %pause(1);
                
                if iAngShift==34
                    1;
                end

                radarDataOversampleForMaxIdx(:,iAngShift) = correlatedFrameTmp;
            end
            absMetricConsider = medfilt1(absMetric,3);
            [~,iAngShiftConsider] = max(absMetricConsider);
            cdma_sig_way6 = sign(sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + shiftRadTot(iAngShiftConsider)));
            cdma_sig = cdma_sig_way6; % cdma_sig_way1; default is cdma_sig_way1
            cdma_sig_rep = repmat(cdma_sig, size(frameTot,1), 1);
            correlatedFrameTmp = sum(frameTot.*cdma_sig_rep,2);

            medMaxIdx = median(maxIdxTot);
            absMetricMedMax = abs(radarDataOversampleForMaxIdx(medMaxIdx,:));
            absMetricConsiderMedMax = medfilt1(absMetricMedMax,3);
            [~,iAngShiftConsiderMedMax] = max(absMetricConsiderMedMax);
            cdma_sig = sign(sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + shiftRadTot(iAngShiftConsiderMedMax)));
            cdma_sig_rep = repmat(cdma_sig, size(frameTot,1), 1);
            correlatedFrameTmpMedMax = sum(frameTot.*cdma_sig_rep,2);
            
            disp('diff between the two ways to estimate the time shift with max amp')
            [iAngShiftConsider iAngShiftConsiderMedMax]
            
            mainFreqEnergy = zeros(1,25);
            for iRange = 1:25
                a = abs(radarDataOversampleForMaxIdx(iRange,:)); 
                fftVal = abs(fft(a - mean(a)));
                fftVal = fftVal(1:floor(length(fftVal)/2));
                [~,maxFftIdx] = max(fftVal);
                mainLobeInd = maxFftIdx + (-2:2);
                mainLobeInd(mainLobeInd<1) = [];
                mainLobeInd(mainLobeInd>floor(length(fftVal)/2)) = [];
                sideLobeInd = setdiff(1:floor(length(fftVal)/2), mainLobeInd);
                mainFreqEnergy(iRange) = norm(fftVal(mainLobeInd))^2/norm(fftVal(sideLobeInd))^2;
            end
            disp('range bin choosing ways')
            [~, maxMainEnergyIdx] = max(mainFreqEnergy)
            absMetricBestLobe = abs(radarDataOversampleForMaxIdx(maxMainEnergyIdx,:));
            absMetricConsiderBestLobe = medfilt1(absMetricBestLobe,3);
            [~,iAngShiftConsiderBestLobe] = max(absMetricConsiderBestLobe);
            cdma_sig = sign(sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + shiftRadTot(iAngShiftConsiderBestLobe)));
            correlatedFrameTmpBestLobe = frameTot*cdma_sig.';
            
            
            
            figure(1);
            plot(absMetric); hold on; plot(absMetricConsider); hold off;
            figure(2);
            plot(abs(correlatedFrameTmp));
            figure(3);
            plot(maxIdxTot);
            figure(4);
            plot(absMetricMedMax);
            figure(5);
            plot(absMetricBestLobe);
            figure(6);
            plot(abs(correlatedFrameTmpBestLobe)); title('best lobe correlated frame')
            
            
            % % saving the variables 
            if dopplerIndExpected == 25600
                correlatedFrame{iRx} = correlatedFrameTmp;
                correlatedFrameMedMax{iRx} = correlatedFrameTmpMedMax;
                correlatedFrameBestLobe{iRx} = correlatedFrameTmpBestLobe;
            elseif dopplerIndExpected == 6401
                correlatedFrame_64{iRx} = correlatedFrameTmp;
                correlatedFrameMedMax_64{iRx} = correlatedFrameTmpMedMax;
                correlatedFrameBestLobe_64{iRx} = correlatedFrameTmpBestLobe;
            end
            absMetricAllRx{iRx} = absMetric;
            absMetricMedMaxAllRx{iRx} = absMetricMedMax;
            absMetricBestLobeAllRx{iRx} = absMetricBestLobe;
            maxIdxTotAllRx{iRx} = maxIdxTot;
            medMaxIdxAllRx{iRx} = medMaxIdx;
            mainFreqEnergyAllRx{iRx} = mainFreqEnergy;
            maxMainEnergyIdxAllRx{iRx} = maxMainEnergyIdx;
            radarDataOversampleForMaxIdxAllRx{iRx} = radarDataOversampleForMaxIdx;
            
            pause(1);
        end
        
    end
    save(sprintf('D:\\PhaseLimitedXethruData_%d\\PhaseLimited%s',dopplerIndExpectedTot,filename),'-v7.3',...
        'correlatedFrame','correlatedFrame_64','absMetricAllRx','maxIdxTotAllRx',...
        'correlatedFrameMedMax','correlatedFrameMedMax_64','medMaxIdxAllRx','absMetricMedMaxAllRx',...
        'correlatedFrameBestLobe','correlatedFrameBestLobe_64','absMetricBestLobeAllRx','mainFreqEnergyAllRx','maxMainEnergyIdxAllRx',...
        'radarDataOversampleForMaxIdxAllRx');
end








