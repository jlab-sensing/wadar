function peakDifference = RunDualTagDataset2(databaseName ,tag1Hz, tag2Hz, viz)

d = strcat(pwd, databaseName);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

% To get the size right
[frameTot, framesBB, frameRate] = ProcessFrames(databaseName, string(listOfCaptures(1)));

widthFrames = width(frameTot);

for i = 2:length(listOfCaptures)
    captureName = string(listOfCaptures(i));
    [frameTot(:, end+1:end+widthFrames), framesBB(:, end+1:end+widthFrames), ~] = ProcessFrames(databaseName, captureName);
end

[captureFT, tag1FT] = ProcessFFT(framesBB, frameRate, tag1Hz);
[~, tag2FT] = ProcessFFT(framesBB, frameRate, tag2Hz);

peakBin1 = TagLocateCWT(tag1FT, false);
peakBin2 = TagLocateCWT(tag2FT, false);

[~, freq1Index] = TagIndex(captureFT, frameRate, tag1Hz);
[~, freq2Index] = TagIndex(captureFT, frameRate, tag2Hz);

SNRdB1 = TagSNR(captureFT, freq1Index, peakBin1);
SNRdB2 = TagSNR(captureFT, freq2Index, peakBin2);

peakDifference = abs(peakBin1 - peakBin2);

tag1Name = strcat(num2str(tag1Hz), ' Hz');
tag2Name = strcat(num2str(tag2Hz), ' Hz');

if (viz)
    PlotFrames(frameTot, framesBB)
    PlotDualTag(captureFT, tag1FT, tag2FT, tag1Name, tag2Name, frameRate)
end

end