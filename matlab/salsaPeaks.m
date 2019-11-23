% Reads a csv containing information about experiments 
% Loads captures, including data and templates 
% Finds peaks in the ft of template (0cm) captures 
% Automatically estimates peaks in the data 
% Performs summary statistics 
% Writes results to an xlsx 

% Assumptions: 
% the desired peak for the template signal (airbin) is the tallest peak
% template captures will have 'template' in the name 
% data is organized by localpath -> experiment directory -> captures
%   (captures are templates & data)
% frameRate is consistent among the captures 
% csv follows a certain format (see readme) 

% Args 
% localPath: path to experiment folders and csv 
% writeMode: 0 if looking at individual peak detection plots
%            1 if writing all peak detection results to xlsv 

% Example Usage: salsaPeaks('/home/bradley/Documents/Research/peak_detect/', 0) 
%   Note: be sure to specify captures in the "capturesToPlot" variable 

function salsaPeaks(localPath, writeMode)
frameRate = 200; 
maxTemplates = 2; % max number of templates used in any experiment

% ------------------------------------------ DETERMINE MODE ----------------------------------------
% TODO: allow plotDetectedPeaks to also specify experiment name, since
% there are repeats in the capture names 
if writeMode
    capturesToPlot = [];
else
    % specify as expName:captureName 
    capturesToPlot = {'clay_active:fullDepth_10s_2can8','silt_passive:fullDepth_100s_4can6'}; 
end

% ------------------------------------------ READ THE CSV ------------------------------------------
csvFilename = 'peaks.csv';  
fid1 = fopen(fullfile(localPath, csvFilename));  
in = textscan(fid1,'%s%s%f%f%f%f','delimiter',',');
expNames = in{1};
captureNames = in{2}; 
airPeaks = in{3}; 
peaksManual = in{4}; 
vwcManual = in{5}; 
vwcTrue = in{6}; 

% ----------------------------------------- ESTIMATE PEAKS AND VWC ---------------------------------  
% get names of the folders that contain experiments 
expDirs = dir(localPath); 

peakPreds = zeros(length(captureNames), maxTemplates); 
vwcPreds = zeros(length(captureNames), maxTemplates); 

% estimate peaks and vwc for each experiment 
for k = 1:length(expDirs)
    expFileName = expDirs(k).name; 
 
    if strcmp(expFileName, '.') | strcmp(expFileName, '..') | contains(expFileName, 'csv') | ...
            contains(expFileName, 'xlsx') | contains(expFileName, 'zip') | contains(expFileName, 'txt')
        continue 
    end
    
    % get names of the experiment's captures (templates & data)   
    dataDirs = dir(fullfile(localPath, expFileName)); 
    
    % determine the soil type 
    if (contains(expFileName, 'farm'))
        soilType = 'farm'; 
    elseif (contains(expFileName, 'silt'))
        soilType = 'silt'; 
    elseif (contains(expFileName, 'clay'))
        soilType = 'clay'; 
    else
        error('experiment folder name does not contain a valid soil type'); 
    end
     
    % get the fts and the maximum peak bins for the templates only   
    templatePaths = []; 
    templateFTs = []; 
    templatePeakBins = []; 
    for i = 1:length(dataDirs)
        dataFileName = dataDirs(i).name; 
        
        if ~(strcmp(dataFileName, '.') | strcmp(dataFileName, '..') | contains(dataFileName, 'md5'))
            if contains(dataFileName, 'template')
                
                % get path 
                templatePaths = [templatePaths string(dataFileName)];
                
                % get ft of data 
                % TODO: make this a function 
                [newFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(dataDirs(i).folder, dataFileName));
                frameTot = [newFrames]; 
                frameWindow = frameTot;
                frameCount = size(frameWindow, 2);
                bg = zeros(size(frameWindow(:,1)));
                frameWindow_bb = zeros(size(frameWindow));
                
                for j = 1:frameCount
                    frameWindow_bb(:,j) = NoveldaDDC(frameWindow(:,j)-bg, chipSet, pgen, fs_hz);
                end

                ft = fft(frameWindow_bb(:,(0)*(frameCount) + 1: 1*(frameCount)),(frameCount),2);
              
                % choose the frequency that gives the largest area  
                freq = 80 * length(ft) / frameRate; 
                ft = abs(ft(:, freq-3 : freq+3));
                [argVal, argMax] = max(sum(ft,1));
                ft = ft(:,argMax); 
                
                ft = ft ./ sum(ft); % normalize 
                
                templateFTs(:,length(templatePaths)) = ft(:,1);
                
                % find the bin corresponding to the largest peak 
                [argVal, argMax] = max(ft);
                templatePeakBins = [templatePeakBins argMax]; 
            end
        end
    end
    
    % make sure there is at least one template 
    if length(templateFTs) == 0
        error('no templates for the current experiment'); 
    end
    
    % get estimated peak and vwc for each data capture 
    for i = 1:length(dataDirs)
        dataFileName = dataDirs(i).name; 
        
        if ~(strcmp(dataFileName, '.') | strcmp(dataFileName, '..') | contains(dataFileName, 'md5'))
            if ~contains(dataFileName, 'template')
                
                % if "read" mode, continue if capture not specified for plotting
                if ~writeMode
                    captureMatch = false; 
                    for capture = capturesToPlot
                        str = strsplit(capture{1}, ':'); 
                        if strcmp(str{2}, dataFileName) && strcmp(str{1}, expFileName)
                            captureMatch = true;
                        end
                    end
                    
                    if ~captureMatch
                        continue 
                    end
                end
                
                % find unique index in csv file matching exp and capture names 
                ind1 = find(strcmp(expNames, expFileName)); 
                ind2 = find(strcmp(captureNames, dataFileName));
                csvIndex = intersect(ind1,ind2); 
                if length(csvIndex) ~= 1
                    disp(dataFileName)
                    error('capture name cannot be linked to a single, unique entry in the csv') 
                end
                
                % get ft of data 
                [newFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(dataDirs(i).folder, dataFileName));
                frameTot = [newFrames]; 
                frameWindow = frameTot;
                frameCount = size(frameWindow, 2);
                bg = zeros(size(frameWindow(:,1)));
                frameWindow_bb = zeros(size(frameWindow));
                
                for j = 1:frameCount
                    frameWindow_bb(:,j) = NoveldaDDC(frameWindow(:,j)-bg, chipSet, pgen, fs_hz);
                end

                ft = fft(frameWindow_bb(:,(0)*(frameCount) + 1: 1*(frameCount)),(frameCount),2);
              
                % choose the frequency that gives the largest area 
                freq = 80 * length(ft) / frameRate; 
                ft = abs(ft(:, freq-3 : freq+3));
                [argVal, argMax] = max(sum(ft,1));
                ft = ft(:,argMax); 
                
                ft = ft ./ sum(ft); % normalize 
                
                % determine best peak for each template by template matching 
                for j = 1:length(templateFTs(1,:))
                   
                    % If "read" mode, view the peak detection plot
                    % Otherwise, calculate the peak without plotting  
                    if ~writeMode 
                        plotTitle = strcat(expFileName, " , ", dataFileName, " , template: ", num2str(j), ...
                            " ,manual peak = ", num2str(peaksManual(csvIndex))); 
                        peak = determinePeak(templateFTs(:,j),templatePeakBins(j), ft, plotTitle); 
                    else
                        peak = determinePeak(templateFTs(:,j),templatePeakBins(j), ft); 
                    end
                    
                    % store results according to order in csv
                    peakPreds(csvIndex, j) = peak; 
                    vwcPreds(csvIndex, j) = calculateSoilMoisture(templatePeakBins(j), peak, soilType); 
                end        
            end
        end
    end  
end
%-------------------------------------------- DO SOME ANALYSIS ------------------------------------- 
% calculate errors in peak calculations  
if writeMode 
    peakErrors = [];  
    vwcErrors = []; 
    for i = 1: length(peakPreds(1,:))
        peakErrors(:,i) = peakPreds(:,i) - peaksManual; 
        vwcErrors(:,i) = vwcPreds(:,i) - vwcManual; 
    end
end

% ------------------------------------------ TABLE TO XLSV -----------------------------------------  
if writeMode
    T = table(expNames, captureNames, peaksManual, peakPreds, peakErrors, vwcTrue, vwcManual, vwcPreds, vwcErrors);
    writetable(T, fullfile(localPath,'peakResults.xlsx')); 
    fclose(fid1);   
end

end 
                
                
        
     
            
         
    
    
    
    