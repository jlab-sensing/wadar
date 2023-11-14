function vwc = wadar(airFileName, captureName)
% vwc = wadar(airFileName, captureName)
%
% Script calculates the volumetric water content (vwc) of the soil from 
% the capture
%
% Inputs:
%        airFileName: Radar capture with tag uncovered with soil
%        captureName: Radar capture with tag covered with soil
%
% Outputs:
%   vwc: Calculated volumetric water content
%


close all

d = 0.13; % in meters
localDataPath = "C:/jlab/wadar/matlab/data/";
tagHz = 80;
airCaptureFile = airFileName;
frameRate = 200;

%% Load Air Capture %%
[airRawFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath, airCaptureFile));
airFrameCount = size(airRawFrames, 2);
airFramesBB = zeros(size(airRawFrames));

[freqTag, freqTagHar] = calculateTagFrequencies(tagHz, 200, airFrameCount); 

for j = 1:airFrameCount
    airFramesBB(:,j) = NoveldaDDC(airRawFrames(:,j), chipSet, pgen, fs_hz);
end

% Find the bin corresponding to the largest peak 
airFT = fft(airFramesBB, airFrameCount , 2); 
airTagFT = abs(airFT(:, freqTag)); 
[vals, peaks] = findpeaks(airTagFT, 'MinPeakHeight', max(airTagFT) * 0.9);
airPeakBin = peaks(1);

%% Load soil capture %%
[frames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath,captureName));
% Baseband Conversion
frameCount = size(frames, 2);
framesBB = zeros(size(frames));
for i = 1:frameCount
    framesBB(:,i) = NoveldaDDC(frames(:,i), chipSet, pgen, fs_hz);
end

% FFT of signal for each bin
ft = fft(framesBB(:,1:frameCount),frameCount,2);    

% Find Tag FT
freqTag = tagHz / frameRate * frameCount;
freqTagHar = (frameRate - tagHz) / frameRate * frameCount; 

TagFT = abs(ft(:, freqTag));
for i = (freqTag-2:1:freqTag+2)
    temp = abs(ft(:, i));
    if max(temp(1:airPeakBin, :)) > max(temp(airPeakBin:end, :)) % if greatest peak is before air peak, skip
        continue
    end
    if max(temp(airPeakBin:end)) > max(TagFT(airPeakBin:end))
        TagFT = temp;
    end
end

TagHarFT = abs(ft(:, freqTagHar));
for i = (freqTagHar-2:1:freqTagHar+2)
    temp = abs(ft(:, i));
    if max(temp(1:airPeakBin, :)) > max(temp(airPeakBin:end, :)) % if greatest peak is before air peak, skip
        continue
    end
    if max(temp(airPeakBin:end)) > max(TagFT(airPeakBin:end))
        TagHarFT = temp;
    end
end

TagFT = TagFT; % + TagHarFT;

[~, peaks] = findpeaks(TagFT(airPeakBin:end), 'MinPeakHeight', max(TagFT(airPeakBin:end)) * 0.90);
peaks = peaks+airPeakBin;

% Select peak bin greater than air peak
peak = peaks(1);
i = 0;
while peak < airPeakBin
    if i > length(peaks) - 1
        break;
    end
    i = i + 1;
    peak = peaks(i);
end
% peak = 250;

corrArray = zeros(512,1);
for i = (1:1:512)
    corrArray(i) = corr(circshift(airTagFT, i), TagFT);
end
figure(2)
plot(circshift(corrArray, airPeakBin))
xline(peak)
[~, peakIndex] = max(circshift(corrArray, airPeakBin))

vwc = calculateSoilMoisture(airPeakBin, peak, "farm", d);

figure(1)
plot(normalize(TagFT, "range"))
hold on;
plot(normalize(airTagFT, "range"))
xline(peakIndex)
legend('Capture (80 Hz + 120 Hz)', 'Air (80 Hz)')
xlabel('Range Bins')
ylabel('Magnitude')
title("80 Hz Isolated");

fprintf("Air peak located at %d\n", airPeakBin);
fprintf("Capture peak located at %d\n", peak);
fprintf("VWC = %f\n", vwc);

end