% clear; close all; clc;
% 
% dataDir = 'C:\\Users\\snsgOvr\\Dropbox\\XethruData\\';
% startFile = 'ExpData20180628T224847';
% expFileOffsetTot = 8:17; % 1:6;
% 
% for iExp = 1:length(expFileOffsetTot)
%     expFileOffset = expFileOffsetTot(iExp);
%     filenames = dir( sprintf('%s/Exp*.mat',dataDir));
%     ExpStartFileIdx = strmatch(startFile,{filenames(:).name}) -1 + expFileOffset;
%     filename = filenames(ExpStartFileIdx).name;
%     
%     radarData = load([dataDir '/' filename]);
%     frameTot = radarData.frameTot;
%     timeTot = radarData.timeTot;
%     
%     a = (db(abs(fft(frameTot,[],2))));
%     freqIndices = [255994 143427];
%     figure(5); plot(a(:,freqIndices));
%     
%     fftFrame = fft(frameTot,[],2);
%     fftFrameSmall = fftFrame(:, freqIndices);
%     figure; plot(db(abs(fftFrameSmall)))
%     
%     save([dataDir '/' filename], 'fftFrameSmall', 'timeTot');
%     
% end
