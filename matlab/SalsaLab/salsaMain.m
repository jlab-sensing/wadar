% USAGE: Captures or loads radar data from C or MATLAB and displays results  

% REQUIRED ARGS: 
% isDataCaptured: Boolean specifying whether data should be loaded from file or newly acquired
% fullDataPath: string to specify path to data 
% radar type: Ancho or Cayenne or Chipotle 

% EXAMPLE 1: Capture and display data from Ancho, with save
% salsaMain(1, 'bradley@192.168.7.1:/home/bradley/Downloads/data/', 'Ancho')
    
% EXAMPLE 2: Load data and display
% salsaMain(0, 'bradley@192.168.7.1:/home/bradley/Downloads/data/')

function salsaMain(isDataCaptured, varargin)
%% Process arguments 
radarType = 'default'; 
fullDataPath = 'default'; 
localDataPath = 'default'; 

for i = 1:length(varargin)
    arg = varargin{i}; 
    if strcmp(lower(arg),'ancho')
        radarType = 'ancho'; 
        radarSpecifier = "X2";
    elseif strcmp(lower(arg), 'cayenne')
        radarType = 'cayenne'; 
        radarSpecifier = "X1-IPG0"; 
    elseif strcmp(lower(arg), 'chipotle')
        radarSpecifier = "X1-IPG1"; 
        radarType = 'chipotle';
    else
        fullDataPath = arg; 
    end
end

if (strcmp(fullDataPath, 'default'))
    error('No file name provided to load data from')
end

if (strcmp(radarType, 'default'))
    error('Please specify a valid radar type')
end

k = strfind(fullDataPath, '/'); %get directory name from full data path by searching for first /
k = k(1);
localDataPath = fullDataPath(k:end); %does not include IP address

%name of saved data
captureName = input('Enter a name for the data file: ', 's');

%% Specify parameters 
numTrials = 2000; 
runs = 3; 
frameRate = 200; 

%% Capture Data

%TODO- make function to copy JSON file over to BBB

if isDataCaptured == 1
    
    %Make sure files do not already exist with current name
    existingFiles = dir(localDataPath);
    pattern = strcat(captureName, '[0-9]+'); %name of file followed by some run number
    for i = 1:length(existingFiles)
        if regexp(existingFiles(i).name, pattern); %file already exists
            error('File name in use. Change file name or remove existing files'); 
        end
    end
    
    %tell BBB to run C program to capture data
    options = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
        captureName, numTrials, runs, frameRate, radarType, fullDataPath); 
    command = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && ./frameLogger %s" &', options); 
    %TODO : understand what the '&' is doing 
    
    [status,cmdout] = system(command); 
end

%% Load and display Data
frameTot = []; 
runCount = 1; 

while (runCount <= runs)
    %Check for new data and plot
    captureFile = dir(fullfile(localDataPath, strcat(captureName, string(runCount))));
    if length(captureFile) == 1
        
        pause(0.5); %give time for copy to fully happen
        
        % Get the frames 
        fileName = captureFile(1).name; 
        [newFrames pgen fs_hz] = salsaLoad(fullfile(localDataPath,fileName), radarSpecifier); 
        frameTot = [frameTot newFrames]; 
        runCount = runCount + 1; 
        
        % Baseband Conversion
        frameCount = size(frameTot, 2); 
        frameTot_bb = zeros(size(frameTot)); 
        for i = 1:frameCount
            frameTot_bb(:,i) = NoveldaDDC(frameTot(:,i), radarSpecifier, pgen, fs_hz); 
        end

        % FFT of signal for each bin
        framesFFT = db(abs(fft(frameTot_bb,frameCount,2)));
        
        % TODO make this continuously update instead of close and reopen 
        close all; 
        salsaPlot(frameTot_bb, framesFFT); 
    end
    
    pause(1); %wait 1 second 
end


        

 