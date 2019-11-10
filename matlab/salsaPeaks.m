% Assumptions: 
% the desired peak for the template signal is the tallest peak
% template captures will have 'template' in the name 
% data is organized by localpath -> experiment directory -> capture data
% framerate is consistent among the captures 

function salsaPeaks(localPath)
frameRate = 200; 

% open the csv file for writing 
csvFilename = 'peaks.csv';  
csvOutputName = 'output.csv'; 
fid1 = fopen(fullfile(localPath, csvFilename));  
in = textscan(fid1,'%s%s%f','delimiter',',');
expNames = in{1};
captureNames = in{2}; 
manualPeaks = in{3}; 

% get names of the folders that contain experiments 
expDirs = dir(localPath); 

for k = 1:length(expDirs)
    expPath = expDirs(k).name; 
 
    if strcmp(expPath, '.') | strcmp(expPath, '..') | contains(expPath, 'csv') | contains(expPath, 'xlsx')
        continue 
    end
    
    % get names of the templates (data) and capture folders 
    dataDirs = dir(fullfile(localPath, expPath)); % these are data and template files  
     
    % get the file name, ft, and max peak bin for each template   
    templatePaths = []; 
    templateFTs = []; 
    templatePeakBins = []; 
    for i = 1:length(dataDirs)
        
        if ~(strcmp(dataDirs(i).name, '.') | strcmp(dataDirs(i).name, '..') | contains(dataDirs(i).name, 'md5'))
            if contains(dataDirs(i).name, 'template')
                % get path 
                templatePaths = [templatePaths string(dataDirs(i).name)];
                
                % get ft of data 
                [newFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(dataDirs(i).folder, dataDirs(i).name));
                frameTot = [newFrames]; 
                frameWindow = frameTot;
                frameCount = size(frameWindow, 2);
                bg = zeros(size(frameWindow(:,1)));
                frameWindow_bb = zeros(size(frameWindow));
                
                for j = 1:frameCount
                    frameWindow_bb(:,j) = NoveldaDDC(frameWindow(:,j)-bg, chipSet, pgen, fs_hz);
                end

                ft = fft(frameWindow_bb(:,(0)*(frameCount) + 1: 1*(frameCount)),(frameCount),2);
              
                % choose the frequency that gives the largest area under
                % the ft graph 
                factor = length(ft) / (frameRate * 10); 
                ft = abs(ft(:, 800*factor-3 : 800*factor+3)); 
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
    
    peakPreds = zeros(length(captureNames), length(templatePaths)); 
    
    % get filename, ft, and estimated peak of the data 
    dataPaths = []; 
    dataPeakBins = [];
    dataIndex = 1; 
    for i = 1:length(dataDirs)
        
        if ~(strcmp(dataDirs(i).name, '.') | strcmp(dataDirs(i).name, '..') | contains(dataDirs(i).name, 'md5'))
            if ~contains(dataDirs(i).name, 'template')
                % get path 
                dataPaths = [dataPaths string(dataDirs(i).name)]; 
                
                % get ft of data 
                [newFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(dataDirs(i).folder, dataDirs(i).name));
                frameTot = [newFrames]; 
                frameWindow = frameTot;
                frameCount = size(frameWindow, 2);
                bg = zeros(size(frameWindow(:,1)));
                frameWindow_bb = zeros(size(frameWindow));
                
                for j = 1:frameCount
                    frameWindow_bb(:,j) = NoveldaDDC(frameWindow(:,j)-bg, chipSet, pgen, fs_hz);
                end

                ft = fft(frameWindow_bb(:,(0)*(frameCount) + 1: 1*(frameCount)),(frameCount),2);
              
                % choose the frequency that gives the largest area under
                % the graph 
                factor = length(ft) / (frameRate * 10); 
                ft = abs(ft(:, 800*factor-3 : 800*factor+3)); 
                [argVal, argMax] = max(sum(ft,1));
                ft = ft(:,argMax); 
                ft = ft ./ sum(ft); % normalize 
                
                % compare to each template to determine best peak
                for j = 1:length(templateFTs(1,:))
                    % grab top 6 peaks of data ft 
                    [peaks,peakBins] = findpeaks(ft); 
                    [maxPeaks,I] = maxk(peaks, 6); 
                    peaks = peaks(I); 
                    peakBins = peakBins(I); 

                    % TODO: add function to not consider peaks of a certain amplitude 

                    % calculate correlation score for all peaks 
                    corr = []; 
                    for binIndex = 1:length(peakBins)
                        peakBin = peakBins(binIndex);

                        if peakBin < templatePeakBins(j) 
                            corr = [corr -1]; % we don't want this to be a valid choice for the max 
                            continue % ignore peaks before bin peak
                        end

                        binDifference = peakBin - templatePeakBins(j); 

                       % wrap signal 
                       newTemplateFT = circshift(templateFTs(:,j), binDifference); 

                       corr = [corr sum(newTemplateFT .* ft)]; 
                    end

                    % choose peak corresponding to the max correlation score
                    [val, argMax] = max(corr);
                    peak = peakBins(argMax);
                    dataPeakBins(j, dataIndex) = peak; 

                    plotting = 1; 
                    if plotting
                        close all; 
                        figure; 
                        plot(templateFTs(:,j)); hold on; 
                        plot(ft); hold on; 
                        plot(peakBins, peaks, 'o'); 
                        grid on; 
                        title(sprintf('%s', dataDirs(i).name)); 
                        text(peak, ft(peak), string(val)); 
                    end
                end
                
                % write peaks to csv 
                in1 = find(strcmp(expNames, expPath)); 
                in2 = find(strcmp(captureNames, dataDirs(i).name));
                csvIndex = intersect(in1,in2); 
                if length(csvIndex) ~= 1
                    disp(dataDirs(i).name)
                    error('capture name cannot be linked to a single, unique entry in the csv') 
                end
                
                peakPreds(csvIndex, :) = dataPeakBins(:, dataIndex); 
                
                dataIndex = dataIndex + 1; 
            end
        end
    end  
end

% calculate errors in peak calculations  
peakErrors = [];  
for i = 1: length(peakPreds(1,:))
    peakErrors(:,i) = peakPreds(:,i) - manualPeaks;  
end

% write to csv 
writetable(cell2table([in{1} in{2} num2cell(in{3}) num2cell(peakPreds) num2cell(peakErrors)]), fullfile(localPath,'peakResults.xlsx')) 
                
fclose(fid1);
end
    
    