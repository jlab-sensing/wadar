function [peakDifference, SNRdB1, SNRdB2] = DatasetFramesProc(databaseName, tag1Hz, tag2Hz, scansPerDatum)

d = strcat(pwd, databaseName);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

% To get the size right
% [frameTot, framesBB, frameRate] = ProcessFrames(databaseName, string(listOfCaptures(1)));

peakDifference = [];
SNRdB1 = [];
SNRdB2 = [];

for j = 1:scansPerDatum:length(listOfCaptures)
    for i = j:j+scansPerDatum-1
        if (i - j) == 0
            captureName = string(listOfCaptures(i));
            [frameTot, framesBB, frameRate] = ProcessFrames(databaseName, captureName);
            widthFrames = width(frameTot);
        else
            captureName = string(listOfCaptures(i));
            [frameTot(:, end+1:end+widthFrames), framesBB(:, end+1:end+widthFrames), ~] = ProcessFrames(databaseName, captureName);
        end
    end

    [captureFT, tag1FT] = ProcessFFT(framesBB, frameRate, tag1Hz);
    [~, tag2FT] = ProcessFFT(framesBB, frameRate, tag2Hz);

    peakBin1 = TagLocateCWT(tag1FT, false);
    peakBin2 = TagLocateCWT(tag2FT, false);

    [~, freq1Index] = TagIndex(captureFT, frameRate, tag1Hz);
    [~, freq2Index] = TagIndex(captureFT, frameRate, tag2Hz);

    SNRdB1(end+1) = TagSNR(captureFT, freq1Index, peakBin1);
    SNRdB2(end+1) = TagSNR(captureFT, freq2Index, peakBin2);

    peakDifference(end+1) = abs(peakBin1 - peakBin2);

end

end