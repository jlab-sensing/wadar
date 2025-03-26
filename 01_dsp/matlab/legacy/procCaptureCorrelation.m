function [procSuccess, wetPeakBin, airPeakBin, SNRdB, wetTagFT] = procCaptureCorrelation(templateFramesName, airFramesName, wetFramesName, localDataPath)
% volumetricWaterContent = procWetSoilFrames(airFileName, templateName, localDataPath)
%
% Function returns volumetric moisture content based on radar data
%
% Inputs:
%       templateFramesName: Name of radar capture with obvious peak
%       airFramesName: Name of radar capture with backscatter tag on soil but
%           left uncovered
%       wetFramesName: Name of radar capture with backscatter tag buried
%       undeneath wet soil
%       localDataPath: Path of radar frames storage
%
% Outputs:
%       wetCapturePeak: Determined peak bin of backscatter tag

%% Process Template Capture
[procSuccess, ~, templateTagFT, templatePeakBin, ~] = procRadarFrames(localDataPath, templateFramesName);
if (procSuccess == false)
    error("ERROR: Template Frames Invalid")
end

%% Process Air Capture
[procSuccess, ~, ~, airPeakBin, ~] = procRadarFrames(localDataPath, airFramesName);
if (procSuccess == false)
    error("ERROR: Air Frames Invalid")
end

%% Process Wet Soil Capture
[procSuccess, ~, wetTagFT, ~, SNRdB] = procRadarFrames(localDataPath, wetFramesName);

% Find the bin corresponding to the largest peak 
[~, wetPeaks] = findpeaks(wetTagFT(airPeakBin:end), 'MinPeakHeight', max(wetTagFT(airPeakBin:end)) * 0.90);
wetPeaks = wetPeaks+airPeakBin;
if (size(wetPeaks, 1) > 1)
    if (wetPeaks(2) - wetPeaks(1) < 50)  % if double peak, invalid radar capture
        procSuccess = false;
        fprintf("Double peak detected. Please reorient the radar.\n")
    end
end

% Select peak bin greater than air peak
wetPeak = wetPeaks(1);
j = 0;
while wetPeak < airPeakBin
    if j > length(wetPeaks) - 1
        break;
    end
    j = j + 1;
    wetPeak = wetPeaks(j);
end

% Select peak bin correlated to template capture
corrArray = zeros(1, 512);
for j = (1:1:512)
    shiftedTemplateTagFT = circshift(norm(templateTagFT), j);

    % Pearson Correlation
    meanTemplate = mean(norm(shiftedTemplateTagFT));
    meanWetTag = mean(norm(wetTagFT));

    corrArray(j) = sum((shiftedTemplateTagFT - meanTemplate) .* (norm(wetTagFT) - meanWetTag)) / sqrt(sum((shiftedTemplateTagFT - meanTemplate).^2) * sum((norm(wetTagFT) - meanWetTag).^2));
end
[~, peakIndex] = max(circshift(corrArray, templatePeakBin));
closestPeak = wetPeaks(1);
for j = 1:size(wetPeaks, 1)
    if abs(wetPeaks(j) - peakIndex) < closestPeak
        closestPeak = wetPeaks(j);
    end
end
wetPeakBin = closestPeak;

% TODO: Update SNR. It's currently using the absolute peak bin rather than
% the correlated peak bin.

end