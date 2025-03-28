function results = RunDualTagData(captureName, localDataPath, tag1Hz, tag2Hz)
% results = RunDualTagData(captureName, localDataPath, tag1Hz, tag2Hz)
%
% Function to run the dual tag test on Novelda radar data captures and
% visualize the results.
%
% Inputs:
%   localDataPath: Path to the data capture file.
%   tag1Hz: First tag frequency in Hz.
%   tag2Hz: Second tag frequency in Hz.
%
% Outputs:
%   results: Table containing the capture name, peak bins, and SNR (dB).

[frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);

[captureFT, tag1FT] = ProcessFFT(framesBB, frameRate, tag1Hz);
[~, tag2FT] = ProcessFFT(framesBB, frameRate, tag2Hz);

[peakBin1] = TagLocateCWT(tag1FT, false);
[peakBin2] = TagLocateCWT(tag2FT, false);

[~, freq1Index] = TagIndex(captureFT, frameRate, tag1Hz);
[~, freq2Index] = TagIndex(captureFT, frameRate, tag2Hz);

SNRdB1 = TagSNR(captureFT, freq1Index, peakBin1);
SNRdB2 = TagSNR(captureFT, freq2Index, peakBin2);

peakDifference = abs(peakBin1 - peakBin2);

tag1Name = strcat(num2str(tag1Hz), ' Hz');
tag2Name = strcat(num2str(tag2Hz), ' Hz');

tableHeader = {'Capture Name', strcat(tag1Name, ' Peak Bin'), strcat(tag1Name, ' SNR (dB)'), ...
    strcat(tag2Name, ' Peak Bin'), strcat(tag2Name, ' SNR (dB)'), 'Peak Difference'};
results = table({captureName}, peakBin1, SNRdB1, peakBin2, SNRdB2, peakDifference, ...
    'VariableNames', tableHeader);

PlotFrames(frameTot, framesBB)
PlotDualTag(captureFT, tag1FT, tag2FT, tag1Name, tag2Name, frameRate)

end