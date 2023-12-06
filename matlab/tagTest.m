function SNRdB = tagTest(captureName)
% vwc = SNRdB = (captureName)
%
% Displays the tag FT and isolates peak bin.
% Calculates signal to noise ratio (ratio of peak bin amplitude at desired 
% frequency vs an average of a few irrelevant frequencies).
%
% Inputs:
%           captureName: Radar capture
%
% Outputs:
%           SNRdB: Calculated volumetric water content

close all

localDataPath = "C:/jlab/wadar/matlab/data/";
tagHz = 80;
frameRate = 200;

%% Load soil capture %%
[frames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath,captureName));

% Baseband Conversion
frameCount = size(frames, 2);
framesBB = zeros(size(frames));
for i = 1:frameCount
    framesBB(:,i) = NoveldaDDC(frames(:,i), chipSet, pgen, fs_hz);
end

% Find Tag FT
captureFT = fft(framesBB(:,1:frameCount),frameCount,2);  
freqTag = tagHz / frameRate * frameCount;
freqTagHar = (frameRate - tagHz) / frameRate * frameCount; 

TagFT = abs(captureFT(:, freqTag));
for i = (freqTag-2:1:freqTag+2)
    temp = abs(captureFT(:, i));
    if max(temp) > max(TagFT)
        TagFT = temp;
    end
end

% TagFT = abs(captureFT(:, freqTag));

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(TagFT, 'MinPeakHeight', max(TagFT) * 0.90);

figure(1)
plot(TagFT)
xline(peaks(1))
xlabel('Range Bins')
ylabel('Magnitude')
title("80 Hz Isolated");

figure(2)
hold on
for i = 2:1:2000
    plot(abs(captureFT(:, i)))
end
xlabel('Range Bins')
ylabel('Magnitude')
title("FT of all peak bins");

% SNR Calculation
signalMag = abs(captureFT(peaks(1), freqTag));
noiseFreq = round(freqTag*0.945 : freqTag*0.955);
noiseMag = mean(abs(captureFT(peaks(1), noiseFreq))); 
SNR = signalMag / noiseMag; 
SNRdB = 10 * log10(SNR); 

fprintf("SNR at peak bin %d is %.2fdB\n", peaks(1), SNRdB);

figure(1)

end