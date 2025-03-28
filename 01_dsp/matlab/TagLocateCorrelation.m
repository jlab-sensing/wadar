function [tagPeakBin] = TagLocateCorrelation(correlatedTagFT, tagFT)
% [tagPeakBin] = TagLocateCorrelation(correlatedTagFT, tagFT)
%
% Function to get the peak bin of the tag signal in the radar data capture. 
% This function uses the correlatedTagFT and tagFT to find the peak bin of 
% the tag signal using Pearson correlation. This is a Stage 3 processing
% function.
%
% Inputs:
%   correlatedTagFT: FFT of the correlated tag signal. This should be a capture
%                    of the tag where the tag's signal is very strong. Optimal
%                    capture requires free space permittivity (aka air) and
%                    a clear line of sight between the radar and the tag.
%   tagFT: FFT of the tag signal. 
%
% Outputs:
%   tagPeakBin: Peak bin of the tag signal in the radar data capture.

% Find the bin corresponding to the largest peak in the correlated FT
[~, correlatedTagFT] = findpeaks(correlatedTagFT, 'MinPeakHeight', max(tagFT) * 0.90);

% Find the bin corresponding to the largest peak in the tag FT
[~, tagFT] = findpeaks(tagFT, 'MinPeakHeight', max(tagFT) * 0.90);

% Commented out for now
% if (size(tagFT, 1) > 1)
%     if (tagFT(2) - tagFT(1) < 50)  % if double peak, invalid radar capture
%         procSuccess = false;
%         fprintf("Double peak detected. Please reorient the radar.\n")
%     end
% end

% Select peak bin correlated to template capture
corrArray = zeros(1, 512);
for j = (1:1:512)
    shiftedTemplateTagFT = circshift(norm(correlatedTagFT), j);

    % Pearson Correlation
    meanTemplate = mean(norm(shiftedTemplateTagFT));
    meanTag = mean(norm(tagFT));

    corrArray(j) = sum((shiftedTemplateTagFT - meanTemplate) .* (norm(tagFT) - meanTag)) / sqrt(sum( ...
        (shiftedTemplateTagFT - meanTemplate).^2) * sum((norm(tagFT) - meanTag).^2));
end
[~, peakIndex] = max(circshift(corrArray, correlatedTagFT));
closestPeak = tagFT(1);
for j = 1:size(tagFT, 1)
    if abs(tagFT(j) - peakIndex) < closestPeak
        closestPeak = tagFT(j);
    end
end
tagPeakBin = closestPeak;

end