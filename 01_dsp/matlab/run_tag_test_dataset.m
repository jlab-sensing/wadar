function results = run_tag_test_dataset(localDataPath, tagHz)
% results = run_tag_test_dataset(localDataPath, tagHz)
%
% Function to run the tag test on Novelda radar data captures and visualize
% the results.
%
% Inputs:
%   localDataPath: Path to the data capture file.
%   tagHz: Tag frequency in Hz.
%
% Outputs:
%   results: Table containing the capture name, peak bin, and SNR (dB).

d = strcat(pwd, localDataPath);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

peakBin = zeros(1, length(listOfCaptures));
SNRdB = zeros(1, length(listOfCaptures));
for i = 1:length(listOfCaptures)
    captureName = string(listOfCaptures(i));
    [frameTot, framesBB, frameRate] = proc_frames(localDataPath, captureName);
    [captureFT, tagFT] = proc_fft(framesBB, frameRate, tagHz);
    peakBin(i) = tag_cwt(tagFT, false);
        
    [~, freqIndex] = tag_index(captureFT, frameRate, tagHz);
    
    SNRdB(i) = tag_snr(captureFT, freqIndex, peakBin(i));
end

tableHeader = {'Capture Name', 'Peak Bin', 'SNR (dB)'};
results = table(transpose(listOfCaptures), transpose(peakBin), transpose(SNRdB), ...
    'VariableNames', tableHeader);

viz_frames(frameTot, framesBB)
viz_fft(captureFT, tagFT, frameRate)

end