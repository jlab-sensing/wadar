% function to determine the peak of an ft via a selected method 

% Args
% temlateFT: ft of the template signal 
% templatePeakBin: bin of the template that is used for correlation alignment 
% ft: ft of the signal 
% peakMethod: method to use for calculating the peak 
% confidenceMethod: method to use for calculating the confidence 
% varargin: if not None, it contains info for plotting (and causes plotting)
    % note: r.e. the version from 12/28, the title info is not being used
    % for the title, but is being displayed and is causing plotting 

% Returns 
% TODO: fill out returns 

function [peakBin confidence ftTag shiftedTemplate SNR] = determinePeak(...
    templateFT, templatePeakBin, ft, frameRate, tagHz, peakMethod, confidenceMethod, varargin) 

% ------------------------------------------ PREPROCESSING------------------------------------------
% find the magnitude of the ft 
ft = abs(ft); 
shiftedTemplate = [];
templateFT = abs([templateFT(1:512,:); zeros(512-512,1)]); 

% Find the desired frequencies for the tag and the harmonic 
hardCodeFrequencies = 1; % hard code or max peak height 

if hardCodeFrequencies
    [freqTag, freqTagHar] = calculateTagFrequencies(tagHz, frameRate, length(ft(1,:))); 
    
    if length(ft(1,:)) <= frameRate * 30 % <= 30s
        templateFT = abs([templateFT(1:400,:); zeros(512-400,1)]); 
        
    elseif length(ft(1,:)) == frameRate * 100 % 100s
        templateFT = abs([templateFT(1:300,:); zeros(512-300,1)]); 
        
    elseif length(ft(1,:)) == frameRate * 300 % 300s
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
searchRadius = 10; % how many bins to consider on left and right side of peaks 

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
corr = zeros(numPeaks, searchRadius*2+1); 

for binIndex = 1:length(peakBins)
    
    for radius = -searchRadius:searchRadius

        peakBin = peakBins(binIndex) + radius;
        
        if (peakBin < 1 || peakBin > length(templateFT))
            continue
        end

        if peakBin < (templatePeakBin-10) 
            continue % we don't want range bins < template bin to be considered
        end

        % align template ft with data ft by shifting and wrapping 
        binDifference = peakBin - templatePeakBin; 

        shiftedTemplateFT = circshift(templateFT, binDifference);

        % calculate the correlation and remember the highest
        corr(binIndex,radius + searchRadius + 1) = sum(shiftedTemplateFT .* ftTag); 
    end
end

% ----------------------------------------- CHOOSE BEST PEAK -----------------------------------
corr = corr / max(max(corr)); % normalize max corr to 1

% find global maximum 
[val argBinIndex] = max(corr); 
[corrBest argRadiusIndex] = max(val); 
argBinIndex = argBinIndex(argRadiusIndex); 

peakBin = peakBins(argBinIndex) + (argRadiusIndex - searchRadius - 1); 
peakBinCorr = peakBin; 

% for plotting 
binDifference = peakBin - templatePeakBin; 
shiftedTemplateFT = circshift(templateFT, binDifference);

% --------------------------------------GET CONFIDENCE -------------------------------------------
% find confidence metric using correlation difference between max
% correlation found in best peak region and second best peak region
% peak region refers to original peak bins +- searchRadius 
% TODO - we only want to consider peak regions that are far (>30 bins) away
corrSecondBest = max(max(corr([1:argBinIndex-1, argBinIndex+1:end],:))); 

% shifted sigmoid such that shifted_sigmoid(0) = 0, shifted_sigmoid(1) = 1
% TODO: should confidence range from 0.5 to 1 here?
tau = 0.08; % trial and error 
corrConfidence = 2 * (1 / (1 + exp(-(corrBest - corrSecondBest)/tau)) - 0.5); 

% --------------------------------------------- PLOT -------------------------------------------
if plotting
    plot(ftTag, 'displayname','signal fourier transform'); hold on;
    plot(shiftedTemplateFT, 'DisplayName', 'shifted fingerprint fourier transform'); hold on; 
    plot(manualPeakBin, ftTag(manualPeakBin), 'o', 'DisplayName', 'manual peak'); hold on; 
    plot(peakBin, ftTag(peakBin), 'x', 'DisplayName', 'auto peak');

    disp(sprintf("%s \n autopeak=%i",plotInfo, peakBin));
    title('peak detection using correlation method') 
    xlabel('bin')
    ylabel('fourier transform magnitude') 
    set(gca,'fontsize',24)
    legend();
end
      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% METHOD 2: LEFT PEAK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% constants
thresholdAdjust = 0.9; % factor for adjusting which peaks are considered valid
filterSize = 10; % filter size for smoothing data 

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

% ----------------------------------------- CHOOSE BEST PEAK -----------------------------------
peakBins = peakBins(peakBins > 100); % assume no peak in first 100 
peakBin = peakBins(1); 
peakBinLeftMost = peakBin; 

% ----------------------------------------- GET CONFIDENCE -----------------------------------
% leftMostConfidence = 0; 

% --------------------------------------------- PLOT -------------------------------------------
if plotting
    figure
    set(gca,'fontsize',24)
    plot(ftTag, 'displayname','signal fourier transform'); hold on;
    plot(manualPeakBin, ftTag(manualPeakBin), 'o', 'DisplayName', 'manual peak'); hold on; 
    plot(peakBin, ftTag(peakBin), 'x', 'DisplayName', 'auto peak'); hold on; 

    title('peak detection using "left most" peak method') 
    xlabel('bin')
    ylabel('fourier transform magnitude') 
    set(gca,'fontsize',24)
    line([1, 512], [h1,h1], 'LineStyle', '--', 'displayname', 'cutoff1'); 
    line([1, 512], [h2,h2], 'LineStyle', '--', 'displayname', 'cutoff2'); 
    line([1, 512], [threshold, threshold], 'Color', 'red', 'LineStyle', '-', 'displayname', 'threshold'); 
    legend(); 
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ------------------------------------CHOOSE CONFIDENCE BY METHOD-----------------------------------
confidence = 0; 

if confidenceMethod == "joint"
    tau = 20; % normalizing constant 
    confidence = exp(-abs(peakBinCorr - peakBinLeftMost) / tau); 
    
elseif confidenceMethod == "same"
    if peakMethod == "corr"
        confidence = corrConfidence; 
    elseif peakMethod == "leftMost"
       error("no confidence metric created for left most peak method"); 
    end
else
    error("invalid confidence method"); 
end

% ------------------------------------ CHOOSE PEAK BY METHOD ---------------------------------------

peakBin = 0; 

if peakMethod == "corr"
    peakBin = peakBinCorr; 
elseif peakMethod == "leftMost"
    peakBin = peakBinLeftMost; 
else
    error(strcat("No algorithm called ", peakMethod)); 
end

% --------------------------------------------- SNR ------------------------------------------------
SNR = calculateSNR(ft, freqTag, peakBin); 
    
end