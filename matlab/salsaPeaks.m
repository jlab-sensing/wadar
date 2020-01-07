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
% writeMode: 0 if looking at individual peak detection plots (specify captureExpression variable below) 
%            1 if writing all peak detection results to xlsv 
% peakMethod: algorithm used to detect peak; used in determinePeak 

% Example Usage: salsaPeaks('/home/bradley/Documents/Research/peak_detect/', 1, "leftMost") 

function salsaPeaks(localPath, writeMode, peakMethod)
frameRate = 200; 
maxTemplates = 2; % max number of templates used in any experiment
captureExpression = 'fullDepth_10s_0can2'; % expression in the capture name to match for plotting...
                                            % program will continue if expression is not in capture
                                            % name 

% ------------------------------------------ DETERMINE MODE ----------------------------------------
if writeMode
    xlsxFilename = input('Please enter a name for the xlsx file (including .xlsx extension): \n', 's');
    if ~strcmp(xlsxFilename(end-4:end), '.xlsx')
        xlsxFilename = strcat(xlsxFilename, '.xlsx'); 
    end  
end
% ------------------------------------------ READ THE CSV------------------------------------------
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
            contains(expFileName, 'xlsx') | contains(expFileName, 'zip') | contains(expFileName, 'txt') ...
            | contains(expFileName, 'odt')
        continue 
    end
    
    % get names of the experiment's captures (templates + data)   
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

                ft = fft(frameWindow_bb(:,1:frameCount),frameCount,2);
              
                % choose the frequency that gives the largest peak 
                freq = 80 * length(ft) / frameRate; 
                ft = abs(ft(:, freq-1 : freq+1)); % constrain search to tag frequency +/-1
                [val, argMax] = max(max(ft)); 
                ft = ft(:,argMax); 
                
                templateFTs(:,length(templatePaths)) = ft(:,1);
                
                % find the bin corresponding to the largest peak 
                [val, binMax] = max(ft);
                templatePeakBins = [templatePeakBins binMax]; 
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
                
                % if "read" mode, continue if capture not selected for plotting
                if ~writeMode
                    
                    % set the condition for showing plot 
                    showPlot = false;               
                    
                    % condition based on showing certain capture types 
                    if contains(dataFileName, captureExpression)
                        showPlot = true;
                    end
                    
                    if ~showPlot
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

                ft = fft(frameWindow_bb(:,1:frameCount),frameCount,2);
                
                % determine best peak for each template 
                for j = 1:length(templateFTs(1,:))
                   
                    if ~writeMode 
%                         plotInfo = strcat(expFileName, " , ", dataFileName, " , template: ", num2str(j), ...
%                             " ,manual peak = ", num2str(peaksManual(csvIndex))); 
                        plotInfo = strcat(num2str(peaksManual(csvIndex))); 
                        peak = determinePeak(templateFTs(:,j),templatePeakBins(j), ft, frameRate, peakMethod, plotInfo); 
                    else
                        peak = determinePeak(templateFTs(:,j),templatePeakBins(j), ft, frameRate, peakMethod); 
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
    writetable(T, fullfile(localPath,xlsxFilename)); 
    fclose(fid1);   
end

end 
                
                
        
     
            
         
    
    
    
    