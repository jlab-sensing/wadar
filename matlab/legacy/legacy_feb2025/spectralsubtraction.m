%USAGE: Test script to do spectral subtraction

clear
clc

% Modify name of file you want to read
% Assumes one name is 'on' and the other is 'off'
onname  = 'chipotle8cm60sRAW2082TagOn11'; %Change this for a different file
offname = strrep(onname, 'On', 'Off');

localDataPath = '/Users/Eugenio/Downloads/Accuracy_Test0722'; %Change path

%Get both captures and downconvert them
oncaptureFile = dir(fullfile(localDataPath, onname));
offcaptureFile = dir(fullfile(localDataPath, offname));

[onFrames pgen fs_hz chipSet] = salsaLoad(fullfile(localDataPath,onname));
onframeWindow = onFrames;

[offFrames pgen fs_hz chipSet] = salsaLoad(fullfile(localDataPath,offname));
offframeWindow = offFrames;

onframeCount = size(onframeWindow, 2);
onframeWindow_bb = zeros(size(onframeWindow));
for i = 1:onframeCount
    onframeWindow_bb(:,i) = NoveldaDDC(onframeWindow(:,i), chipSet, pgen, fs_hz);
end

offframeCount = size(offframeWindow, 2);
offframeWindow_bb = zeros(size(offframeWindow));
for i = 1:offframeCount
    offframeWindow_bb(:,i) = NoveldaDDC(offframeWindow(:,i), chipSet, pgen, fs_hz);
end

onframesFFT = db(abs(fft(onframeWindow_bb,onframeCount,2)));
offframesFFT = db(abs(fft(offframeWindow_bb,offframeCount,2)));

%Plots the noise and the signal + noise
figure()
salsaPlot(onframeWindow_bb, onframesFFT, 'Signal+Noise');

figure()
salsaPlot(offframeWindow_bb, offframesFFT, 'Noise');

% Spectral subtraction math (PSD version)
% Subtract the noise from the signal with noise, get only signal
PSDframes = onframesFFT.*onframesFFT - 0.8*(offframesFFT.*offframesFFT);
PSDframes(PSDframes<0) = 0; %Set to zero if below zero

firstBin = 1;
lastBin = 512;
figure()
plot(PSDframes(firstBin:lastBin, 4800)') %Gets PSD only at 80 Hz.