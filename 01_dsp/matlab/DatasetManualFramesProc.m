function [peakDifference, SNRdB1, SNRdB2] = DatasetManualFramesProc(databaseName, tag1Hz, tag2Hz, scansPerDatum)

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

    [~, peaks_idx_1] = findpeaks(tag1FT, 'MinPeakDistance', 6, 'MinPeakHeight', 0.5 * max(tag1FT));
    [~, peaks_idx_2] = findpeaks(tag2FT, 'MinPeakDistance', 6, 'MinPeakHeight', 0.5 * max(tag2FT));
    
    figure(1)
    plot(tag1FT ./ max(tag1FT));
    hold on
    plot(tag2FT ./ max(tag2FT))
    xticks(sort([peaks_idx_1; peaks_idx_2]))
    legend(["Tag 1", "Tag 2"])

    peakBin1 = input("Peak Bin 1: ");
    peakBin2 = input("Peak Bin 2: ");

    close(1)

    [~, freq1Index] = TagIndex(captureFT, frameRate, tag1Hz);
    [~, freq2Index] = TagIndex(captureFT, frameRate, tag2Hz);

    SNRdB1(end+1) = TagSNR(captureFT, freq1Index, peakBin1);
    SNRdB2(end+1) = TagSNR(captureFT, freq2Index, peakBin2);

    peakDifference(end+1) = abs(peakBin1 - peakBin2);

end

end