function [peakBin, SNRdB] = RunTagDataEmbedded(captureName, localDataPath, tagHz)

[~, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
[captureFT, tagFT] = ProcessFFT(framesBB, frameRate, tagHz);
[peakBin] = TagLocateCorrelation(tagFT, tagFT);
[~, freqIndex] = TagIndex(captureFT, frameRate, tagHz);
[SNRdB] = TagSNR(captureFT, freqIndex, peakBin);

fprintf("Peak Bin: %f\n", peakBin)
fprintf("SNRdB: %f\n", SNRdB)

end