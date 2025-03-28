function results = RunTagDataset(localDataPath, tagHz)
% results = RunTagDataset(localDataPath, tagHz)
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
    [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
    [captureFT, tagFT] = ProcessFFT(framesBB, frameRate, tagHz);
    peakBin(i) = TagLocateCWT(tagFT, false);
        
    [~, freqIndex] = TagIndex(captureFT, frameRate, tagHz);
    
    SNRdB(i) = TagSNR(captureFT, freqIndex, peakBin(i));
end

tableHeader = {'Capture Name', 'Peak Bin', 'SNR (dB)'};
results = table(transpose(listOfCaptures), transpose(peakBin), transpose(SNRdB), ...
    'VariableNames', tableHeader);

PlotFrames(frameTot, framesBB)
PlotTag(captureFT, tagFT, frameRate)

end