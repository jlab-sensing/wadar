% to test stuff

close all; clear all; clc

localDataPath = '/data/chi-silt-dual-tag-dataset/run-2';

c=299792458;                    % Speed of light (m/s)
resolution = 0.003790984152165; % Range resolution (m)

labelFile = strcat(localDataPath(2:end), '/label.json');

label = readstruct(labelFile);

fprintf("Processing %s ...\n", localDataPath)

tag1Hz = label.tagFrequencies(1);
tag2Hz = label.tagFrequencies(2);
distance = label.tagDistance;
scansPerDatum = label.scansPerDatum;

d = strcat(pwd, localDataPath);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

% To get the size right
% [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, string(listOfCaptures(1)));

peakDifference = [];
SNRdB1 = [];
SNRdB2 = [];

for j = 1:scansPerDatum:length(listOfCaptures)
    for i = j:j+scansPerDatum-1
        if (i - j) == 0
            captureName = string(listOfCaptures(i));
            [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
            widthFrames = width(frameTot);
        else
            captureName = string(listOfCaptures(i));
            [frameTot(:, end+1:end+widthFrames), framesBB(:, end+1:end+widthFrames), ~] = ProcessFrames(localDataPath, captureName);
        end
    end

    [captureFT, tag1FT] = ProcessFFT(framesBB, frameRate, tag1Hz);
    [~, tag2FT] = ProcessFFT(framesBB, frameRate, tag2Hz);
    
    figure
    subplot(2,1,1)
    plot(tag1FT)
    subplot(2,1,2)
    plot(tag2FT)
    
    peakBin1 = TagLocateCorrelation(tag1FT, tag1FT);
    peakBin2 = TagLocateCorrelation(tag1FT, tag2FT);

    [~, freq1Index] = TagIndex(captureFT, frameRate, tag1Hz);
    [~, freq2Index] = TagIndex(captureFT, frameRate, tag2Hz);

    SNRdB1(end+1) = TagSNR(captureFT, freq1Index, peakBin1);
    SNRdB2(end+1) = TagSNR(captureFT, freq2Index, peakBin2);

    peakDifference(end+1) = abs(peakBin1 - peakBin2);

end