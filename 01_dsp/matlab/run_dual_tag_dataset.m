function results = run_dual_tag_dataset(localDataPath, tag1Hz, tag2Hz)
% results = run_dual_tag_dataset(localDataPath, tag1Hz, tag2Hz)
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

d = strcat(pwd, localDataPath);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

peakBin1 = zeros(1, length(listOfCaptures));
SNRdB1 = zeros(1, length(listOfCaptures));
peakBin2 = zeros(1, length(listOfCaptures));
SNRdB2 = zeros(1, length(listOfCaptures));
peakDifference = zeros(1, length(listOfCaptures));
for i = 1:length(listOfCaptures)
    captureName = string(listOfCaptures(i));

    [frameTot, framesBB, frameRate] = proc_frames(localDataPath, captureName);
    
    [captureFT, tag1FT] = proc_fft(framesBB, frameRate, tag1Hz);
    [~, tag2FT] = proc_fft(framesBB, frameRate, tag2Hz);
    
    peakBin1(i) = tag_cwt(tag1FT, false);
    peakBin2(i) = tag_cwt(tag2FT, false);
    
    [~, freq1Index] = tag_index(captureFT, frameRate, tag1Hz);
    [~, freq2Index] = tag_index(captureFT, frameRate, tag2Hz);
    
    SNRdB1(i) = tag_snr(captureFT, freq1Index, peakBin1(i));
    SNRdB2(i) = tag_snr(captureFT, freq2Index, peakBin2(i));

    peakDifference(i) = abs(peakBin1(i) - peakBin2(i));
end

tag1Name = strcat(num2str(tag1Hz), ' Hz');
tag2Name = strcat(num2str(tag2Hz), ' Hz');

tableHeader = {'Capture Name', strcat(tag1Name, ' Peak Bin'), strcat(tag1Name, ' SNR (dB)'), ...
    strcat(tag2Name, ' Peak Bin'), strcat(tag2Name, ' SNR (dB)'), 'Peak Difference'};
results = table(transpose(listOfCaptures), transpose(peakBin1), transpose(SNRdB1), transpose(peakBin2), transpose(SNRdB2), transpose(peakDifference), ...
    'VariableNames', tableHeader);

viz_frames(frameTot, framesBB)
viz_dual_tag(captureFT, tag1FT, tag2FT, tag1Name, tag2Name, frameRate)

end