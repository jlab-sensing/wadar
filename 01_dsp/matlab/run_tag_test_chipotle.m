function [peakBin, SNRdB] = run_tag_test_chipotle(captureName, localDataPath, tagHz)
% results = run_tag_test(captureName, localDataPath, tagHz)
%
% Function to run the tag test on Novelda radar data. This function will
% process the data capture file, calculate the FFT, find the peak bin, and
% calculate the SNR.
%
% Inputs:
%   captureName: Name of the data capture file.
%   localDataPath: Path to the data capture file.
%   tagHz: Tag frequency in Hz.
%
% Outputs:
%   peakBin: Peak bin of the tag frequency.
%   SNRdB: Signal-to-noise ratio in dB.

[frameTot, framesBB, frameRate] = proc_frames(localDataPath, captureName);
[captureFT, tagFT] = proc_fft(framesBB, frameRate, tagHz);
[peakBin] = tag_correlation(tagFT, tagFT);

frameCount = width(framesBB);

% Find tag frequency index
freqTag = tagHz / frameRate * frameCount;
tagFT = abs(captureFT(:, freqTag));
freqIndex = 0;
for i = (freqTag-2:1:freqTag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(tagFT)
            tagFT = temp;
            freqIndex = i;
    end
end

SNRdB = tag_snr(captureFT, freqIndex, peakBin);

end