% USAGE: Captures or loads radar data from C or MATLAB and displays results  

% REQUIRED ARGS: 
% isDataCaptured: Boolean specifying whether data should be loaded from file or newly acquired
% fullDataPath: string to specify path to data 
% radar type: Ancho or Cayenne or Chipotle 
    % required only if isDataCaptured == 1

% EXAMPLE 1: Capture and display data from Ancho, with save
% salsaMain(1, 'bradley@192.168.7.1:/home/bradley/Downloads/data/', 'Ancho')
    
% EXAMPLE 2: Load data and display
% salsaMain(0, 'bradley@192.168.7.1:/home/bradley/Downloads/data/')

function salsaMain(isDataCaptured, varargin)
%% Process arguments 
radarType = -1; 
fullDataPath = 'default'; 
localDataPath = 'default'; 

if isDataCaptured == 1
    for i = 1:length(varargin)
        arg = varargin{i}; 
        if (strcmp(lower(arg),'ancho') || strcmp(lower(arg),'cayenne') || strcmp(lower(arg),'chipotle'))
            radarType = lower(arg);
        else
            fullDataPath = arg; 
        end
    end
        
    if ~(strcmp(radarType,'ancho') || strcmp(radarType,'cayenne') || strcmp(radarType,'chipotle'))
            error('Invalid radar type')
    end
    
else 
    for i = 1:length(varargin)
        arg = varargin{i}; 
        if (strcmp(lower(arg),'ancho') || strcmp(lower(arg),'cayenne') || strcmp(lower(arg),'chipotle'))
            % Do not update radar type to load data
        else 
            fullDataPath = arg; 
        end
    end
        
end

if (strcmp(fullDataPath, 'default'))
    error('No file name provided to load data from')
end

k = strfind(fullDataPath, '/'); %get directory name from full data path by searching for first /
k = k(1);
localDataPath = fullDataPath(k:end); %does not include IP address

%name of saved data
captureName = input('Enter a name for the data file: ', 's');

%Make sure files do not already exist with current name
existingFiles = dir(localDataPath);
pattern = strcat(captureName, '[0-9]+'); %name of file followed by some run number
for i = 1:length(existingFiles)
    if regexp(existingFiles(i).name, pattern); %file already exists
        error('File name in use. Change file name or remove existing files'); 
    end
end

%% Specify parameters 
numTrials = 2000; 
runs = 3; 
frameRate = 200; 

%% Capture Data

%TODO- make function to copy JSON file over to BBB

if isDataCaptured == 1
    
    %tell BBB to run C program to capture data
    options = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
        captureName, numTrials, runs, frameRate, radarType, fullDataPath); 
    command = sprintf('ssh root@192.168.7.2 "cd FlatEarth/Demos/Common/FrameLogger && ./frameLogger %s" &', options); 
    %TODO : understand what the '&' is doing 
    
    [status,cmdout] = system(command); 
end

%% Load and display Data
frameTot = []; 
runCount = 1; 

while (runCount <= runs)
    %Check for new data
    captureFile = dir(fullfile(localDataPath, strcat(captureName, string(runCount))));
    if length(captureFile) == 1
        fileName = captureFile(1).name; 
        newFrames = salsaLoad(fullfile(localDataPath,fileName)); 
        frameTot = [frameTot newFrames]; 
        runCount = runCount + 1; 
    end
    
    pause(1); %wait 1 second 
end
        

 