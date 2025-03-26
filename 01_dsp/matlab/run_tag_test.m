function results = run_tag_test(captureName, localDataPath, tagHz)
% results = run_tag_test(captureName, localDataPath, tagHz)
%
% Function to run the tag test on Novelda radar data captures and visualize
% the results. 
%
% Inputs:
%   captureName: Name of the data capture file.
%   localDataPath: Path to the data capture file.
%   tagHz: Tag frequency in Hz.
%
% Outputs:
%   results: Table containing the capture name, peak bin, and SNR (dB).

[frameTot, framesBB, frameRate] = proc_frames(localDataPath, captureName);
[captureFT, tagFT] = proc_fft(framesBB, frameRate, tagHz);
[peakBin] = tag_cwt(tagFT, false);

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

tableHeader = {'Capture Name', 'Peak Bin', 'SNR (dB)'};
results = table({captureName}, peakBin, SNRdB, ...
    'VariableNames', tableHeader);

viz_frames(frameTot, framesBB)
viz_fft(captureFT, tagFT, frameRate)

end