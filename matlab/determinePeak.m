% function to determine the peak of an ft via a selected method 

% Args
% temlateFT: ft of the template signal 
% templatePeakBin: bin of the template that is used for correlation alignment 
% ft: ft of the signal 
% method: method to use for calculating the peak 
% varargin: if not None, it contains info for plotting (and causes plotting)
    % note: r.e. the version from 12/28, the title info is not being used
    % for the title, but is being displayed and is causing plotting 

% Returns 
% peakBin: calculated peak bin based on the selected method

% TODO: this function should return the confidence as well as the peak 

function [peakBin confidence ftTag shiftedTemplate] = determinePeak(templateFT, templatePeakBin, ft, frameRate, method, varargin) 

% ------------------------------------------ PREPROCESSING------------------------------------------
% find the magnitude of the ft 
ft = abs(ft); 
shiftedTemplate = [];
templateFT = abs([templateFT(1:512,:); zeros(512-512,1)]); 

% Find the desired frequencies for the tag and the harmonic 
hardCodeFrequencies = 1; % hard code or max peak height 

if hardCodeFrequencies
    if length(ft(1,:)) <= frameRate * 30 % <= 30s
        freqTag = 80 / frameRate * length(ft(1,:)) + 1; 
        freqTagHar = (frameRate - 80) / frameRate * length(ft(1,:)) + 1;   
        templateFT = abs([templateFT(1:400,:); zeros(512-400,1)]); 
    elseif length(ft(1,:)) == frameRate * 100 % 100s
        freqTag = 80 / frameRate * length(ft(1,:)) + 0; 
        freqTagHar = (frameRate - 80) / frameRate * length(ft(1,:)) + 2; 
        templateFT = abs([templateFT(1:300,:); zeros(512-300,1)]); 

    elseif length(ft(1,:)) == frameRate * 300 % 300s
        freqTag = 80 / frameRate * length(ft(1,:)) - 1;
        freqTagHar = (frameRate - 80) / frameRate * length(ft(1,:)) + 3;
        templateFT = abs([templateFT(1:300,:); zeros(512-300,1)]); 
    else
        error('unrecognized capture duration') 
    end
    
else
    freqTag = 80 * length(ft) / frameRate; % original guess

    searchRadius = 3; % constrain to +- searchRadius of original frequency 
    [val argMax] = max(max(ft(:, [freqTag-searchRadius : freqTag+searchRadius])));  
    offset = argMax - (searchRadius + 1); 
    freqTag = freqTag + offset; 

    freqTagHar = (200 - 80) * length(ft) / frameRate; % original guess

    searchRadius = 3; % constrain to +- searchRadius of original frequency 
    [val argMax] = max(max(ft(:, [freqTagHar-searchRadius : freqTagHar+searchRadius]))); 
    offset = argMax - (searchRadius + 1);
    freqTagHar = freqTagHar + offset; 
end

% combine information from harmonic frequencies into ftTag 
ftTag = ft(:,freqTag) + ft(:, freqTagHar); 

%TODO: which normalization method is better?
% normalize the fts 
%ft = ft ./ sum(ft); 
%templateFT = templateFT ./ sum(templateFT); 
%ftTag = ftTag ./ sum(ftTag); 
% ft = ft ./ max(ft); 
templateFT = templateFT / max(templateFT); 
ftTag = ftTag / max(ftTag); 

% ------------------------------------------ SETUP PLOTS--------------------------------------------
plotting = 0; 
if length(varargin) > 0
    close all; 
    plotInfo = varargin{1};  
    
    plotting = 1; 
    
    manualPeakBin = str2num(plotInfo); 
    manualPeakBin = round(manualPeakBin); 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% METHOD 1: CORRELATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ----------------------------------------- CALCULATE CORRELATION-----------------------------------
   
numPeaks = 6; % number of peaks in the ft to consider 
filterSize = 10; 

% apply MA heuristic filter 
smoothing = 0; 
if smoothing
    ftTag = smoothdata(ftTag, 'movmean', filterSize);
end

% find top numPeaks peaks of ft 
[peaks, peakBins] = findpeaks(ftTag); 
[maxPeaks,I] = maxk(peaks, numPeaks); 
peaks = peaks(I); 
peakBins = peakBins(I); 

% calculate correlation score for all peaks 
corr = []; 
shiftedTemplateFTs = []; 
for binIndex = 1:length(peakBins)

    peakBin = peakBins(binIndex);

    if peakBin < (templatePeakBin-10) 
        corr = [corr -1]; % we don't want range bins < template bin to be considered
        continue 
    end

    % align template ft with data ft by shifting and wrapping 
    binDifference = peakBin - templatePeakBin; 

    shiftedTemplateFTs(:,binIndex) = circshift(templateFT, binDifference);

    % calculate correlation 
    if length(shiftedTemplateFTs(:,binIndex)) ~= length(ftTag)
        error('length of the signals do not match - cannot compute correlation'); 
    end

    corr = [corr sum(shiftedTemplateFTs(:,binIndex) .* ftTag)]; 
end

% ----------------------------------------- CHOOSE BEST PEAK -----------------------------------
[val, argMax] = max(corr);
peakBin = peakBins(argMax);
peakBinCorr = peakBin; 

% --------------------------------------------- PLOT -------------------------------------------
shiftedTemplate = shiftedTemplateFTs(:,argMax);
if plotting
    plot(ftTag, 'displayname','signal fourier transform'); hold on;
    plot(shiftedTemplateFTs(:,argMax), 'DisplayName', 'shifted fingerprint fourier transform'); hold on; 
    plot(manualPeakBin, ftTag(manualPeakBin), 'o', 'DisplayName', 'manual peak'); hold on; 
    plot(peakBin, ftTag(peakBin), 'x', 'DisplayName', 'auto peak');

    disp(sprintf("%s \n autopeak=%i",plotInfo, peakBin));
    title('peak detection using correlation method') 
    xlabel('bin')
    ylabel('fourier transform magnitude') 
    legend();
end
      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% METHOD 2: LEFT PEAK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% constants
thresholdAdjust = 0.9; % factor for adjusting which peaks are considered valid
filterSize = 10; % filter size for smoothing data 

%     % normalize against the other tag frequencies 
%     ftOther = mean(ft(:, [freqTag - 3 : freqTag - 1, freqTag + 1 : freqTag + 3]), 2); 
%     ftOther = movmean(ftOther, 30); 
%     ftOtherHar = mean(ft(:, [freqTagHar - 3 : freqTagHar - 1, freqTagHar + 1 : freqTagHar + 3]), 2); 
%     ftOtherHar = movmean(ftOtherHar, 30); 
%     
%     ftTag = ft(:,freqTag) ./ ftOther + ft(:, freqTagHar) ./ ftOtherHar;
%     ftTag = ftTag ./ sum(ftTag); 

%     ftTag = ft(:, freqTag) ./ mean(ft(:, [freqTag-5:freqTag-1, freqTag+1:freqTag+5]),2) + ...
%         ft(:, freqTagHar) ./ mean(ft(:, [freqTagHar-5:freqTagHar-1, freqTagHar+1:freqTagHar+5]),2); 

% apply MA heuristic filter 
smoothing = 0; 
if smoothing
    ftTag = smoothdata(ftTag, 'movmean', filterSize);
end

% find left-most peak matching criteria 
h1 = mean(findpeaks(ftTag(1:100)));
h2 = max(ftTag); 
threshold = thresholdAdjust * (h1 + h2) / 2; 

[peaks peakBins] = findpeaks(ftTag, 'MinPeakHeight', threshold); 

peakBins = peakBins(peakBins > 100); % assume no peak in first 100 
peakBin = peakBins(1); 
peakBinLeftMost = peakBin; 

if plotting
    figure
    plot(ftTag, 'displayname','signal fourier transform'); hold on;
    plot(manualPeakBin, ftTag(manualPeakBin), 'o', 'DisplayName', 'manual peak'); hold on; 
    plot(peakBin, ftTag(peakBin), 'x', 'DisplayName', 'auto peak'); hold on; 

    title('peak detection using "left most" peak method') 
    xlabel('bin')
    ylabel('fourier transform magnitude') 
    line([1, 512], [h1,h1], 'LineStyle', '--', 'displayname', 'cutoff1'); 
    line([1, 512], [h2,h2], 'LineStyle', '--', 'displayname', 'cutoff2'); 
    line([1, 512], [threshold, threshold], 'Color', 'red', 'LineStyle', '-', 'displayname', 'threshold'); 
    legend(); 
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ------------------------------------ DETERMINE CONFIDENCE ----------------------------------------
tau = 20; % normalizing constant 

confidence = exp(-abs(peakBinCorr - peakBinLeftMost) / tau); 

% ------------------------------------ CHOOSE PEAK BY METHOD ---------------------------------------

peakBin = 0; 

if method == "corr"
    peakBin = peakBinCorr; 
elseif method == "leftMost"
    peakBin = peakBinLeftMost; 
else
    error(strcat("No algorithm called ", method)); 
end

SNR = calculateSNR(ft, freqTag, peakBin); 
    
end