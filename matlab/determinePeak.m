function [peak] = determinePeak(templateFT, templatePeakBin, ft, varargin) 

numPeaks = 6; % number of peaks in the ft to consider  

plotting = 0; 
if length(varargin) > 0
    plotTitle = varargin{1}; 
    plotting = 1;
end

if plotting
    h = figure;
    plot(ft, '-o','displayname','ft'); hold on; 
end

% find top numPeaks peaks of ft 
% TODO: change to not consider peaks of too little amplitude? 
[peaks,peakBins] = findpeaks(ft); 
[maxPeaks,I] = maxk(peaks, numPeaks); 
peaks = peaks(I); 
peakBins = peakBins(I); 

% calculate correlation score for all peaks 
corr = []; 
shiftedTemplateFTs = []; 
for binIndex = 1:length(peakBins)
    peakBin = peakBins(binIndex);

    if peakBin < templatePeakBin 
        corr = [corr -1]; % we don't want this to be a valid choice for the max 
        continue % ignore peaks before bin peak
    end

    binDifference = peakBin - templatePeakBin; 

    % wrap signal 
    shiftedTemplateFTs(:,binIndex) = circshift(templateFT, binDifference); 

    % calculate correlation 
    if length(shiftedTemplateFTs(:,binIndex)) ~= length(ft)
        error('length of the signals do not match - cannot compute correlation'); 
    end
   
    corr = [corr sum(shiftedTemplateFTs(:,binIndex) .* ft)]; 
   
    if plotting 
        text(peakBins(binIndex), peaks(binIndex), num2str(corr(binIndex))); 
    end
end

% choose peak corresponding to the max correlation score
[val, argMaxCorr] = max(corr);
peak = peakBins(argMaxCorr);

[val, argMaxPeak] = max(peaks); 

if plotting
    plot(shiftedTemplateFTs(:,argMaxCorr), 'DisplayName', 'Template shifted to best corr'); hold on; 
    plot(shiftedTemplateFTs(:,argMaxPeak), 'DisplayName', 'Template shifted to best height'); 
    
    plotTitle = strcat(plotTitle, ' , ', 'auto peak = ', num2str(peak));
    title(plotTitle, 'Interpreter', 'none') 
    xlabel('bin')
    ylabel('corr')
    legend()
    fprintf('waiting until figure is closed...\n')
    waitfor(h)
end

end