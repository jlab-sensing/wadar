function vwc = wadar(airFileName, templateName, captureName)
% vwc = wadar(airFileName, captureName)
%
% Script calculates the volumetric water content (vwc) of the soil from 
% the capture
%
% Inputs:
%        airFileName: Radar capture with tag uncovered with soil
%        captureName: Radar capture with tag covered with soil
%        templateName: Radar capture with obvious tag peak
%
% Outputs:
%   vwc: Calculated volumetric water content
%


close all

d = 0.12; % in meters
localDataPath = "C:/jlab/wadar/matlab/data/";
tagHz = 80;
frameRate = 200;

%% Load Template Capture
[templateRawFrames, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, templateName));

% Baseband Conversion
templateFrameCount = size(templateRawFrames, 2);
templateFramesBB = zeros(size(templateRawFrames));
for j = 1:templateFrameCount
    templateFramesBB(:,j) = NoveldaDDC(templateRawFrames(:,j), chipSet, pgen, fs_hz);
end

% Find Tag FT
templateFreqTag = tagHz / frameRate * templateFrameCount;
templateFT = fft(templateFramesBB, templateFrameCount , 2); 
templateTagFT = abs(templateFT(:, templateFreqTag));
for i = (templateFreqTag-2:1:templateFreqTag+2)
    temp = abs(templateFT(:, i));
    if max(temp) > max(templateTagFT)
        templateTagFT = temp;
    end
end
templateTagFT = smoothdata(templateTagFT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(templateTagFT, 'MinPeakHeight', max(templateTagFT) * 0.9);
if (size(peaks, 1) > 1)
    templatePeakBin = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((templateTagFT(peaks(2)) - templateTagFT(peaks(1))) / max(templateTagFT) * (peaks(2) - peaks(1)));
else
    templatePeakBin = peaks(1);
end


%% Load Air Capture %%
[airRawFrames, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, airFileName));

% Baseband Conversion
airFrameCount = size(airRawFrames, 2);
airFramesBB = zeros(size(airRawFrames));
for j = 1:airFrameCount
    airFramesBB(:,j) = NoveldaDDC(airRawFrames(:,j), chipSet, pgen, fs_hz);
end

% Find Tag FT
airFreqTag = tagHz / frameRate * airFrameCount;
airFT = fft(airFramesBB, airFrameCount , 2); 
airTagFT = abs(airFT(:, airFreqTag));
for i = (airFreqTag-2:1:airFreqTag+2)
    temp = abs(airFT(:, i));
    if max(temp) > max(airTagFT)
        airTagFT = temp;
    end
end
airTagFT = smoothdata(airTagFT, 'movmean', 10);

% Find the bin corresponding to the largest peak 
[~, peaks] = findpeaks(airTagFT, 'MinPeakHeight', max(airTagFT) * 0.9);
if (size(peaks, 1) > 1)
        airPeakBin = peaks(1) +  round((peaks(2) - peaks(1)) / 2) + round((airTagFT(peaks(2)) - airTagFT(peaks(1))) / max(airTagFT) * (peaks(2) - peaks(1)));
else
    airPeakBin = peaks(1);
end



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
    if max(temp(1:airPeakBin, :)) > max(temp(airPeakBin:end, :)) % if greatest peak is before air peak, skip
        continue
    end
    if max(temp(airPeakBin:end)) > max(TagFT(airPeakBin:end))
        TagFT = temp;
    end
end
TagFT = smoothdata(TagFT, 'movmean', 10);

TagHarFT = abs(captureFT(:, freqTagHar));
for i = (freqTagHar-2:1:freqTagHar+2)
    temp = abs(captureFT(:, i));
    if max(temp(1:airPeakBin, :)) > max(temp(airPeakBin:end, :)) % if greatest peak is before air peak, skip
        continue
    end
    if max(temp(airPeakBin:end)) > max(TagFT(airPeakBin:end))
        TagHarFT = temp;
    end
end

% Find the bin corresponding to the largest peak 
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

% TagFT = smoothdata(TagFT, 'lowess', 20);
% templateTagFT = smoothdata(templateTagFT, 'lowess', 20);

% Select peak bin correlated to template capture
corrArray = zeros(length(peaks),2);
for i = (1:1:512)
    corrArray(i) = corr(normalize(circshift(templateTagFT, i)), normalize(TagFT), 'Type', 'Pearson');
end
[~, peakIndex] = max(circshift(corrArray, templatePeakBin));
closestPeak = peaks(1);
for i = 1:size(peaks, 1)
    if abs(peaks(i) - peakIndex) < closestPeak
        closestPeak = peaks(i);
    end
end

% figure(2)
% plot(circshift(corrArray, templatePeakBin))
% for i = 1:size(peaks, 1)
%     xline(peaks)
% end

vwc = calculateSoilMoisture(airPeakBin, closestPeak, "farm", d);

figure(1)
plot(normalize(TagFT, "range"))
hold on;
plot(normalize(templateTagFT, "range"))
plot(normalize(airTagFT, "range"))
xline(airPeakBin)
xline(closestPeak)
legend('Capture (80 Hz)', 'Template (80 Hz)', 'Air (80 Hz)')
xlabel('Range Bins')
ylabel('Magnitude')
title("80 Hz Isolated");

fprintf("Air peak located at %d\n", airPeakBin);
fprintf("Capture peak located at %d\n", closestPeak);
fprintf("VWC = %f\n", vwc);

SNR = calculateSNR(captureFT, freqTag, closestPeak);
SNRdB = 10 * log10(SNR)
% ratio of peak bin amplitude at desired 
% frequency vs an average of a few irrelevant frequencies 

end