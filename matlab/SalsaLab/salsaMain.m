% USAGE: Captures or loads previous radar captures and displays useful results  

% REQUIRED ARGS: 
% isDataCaptured: Boolean specifying whether data should be loaded from file or newly acquired
% fullDataPath: string to specify path to data 
% radar type: Ancho or Cayenne or Chipotle 

% EXAMPLE 1: Capture and display data from Ancho, with save
% salsaMain(1, 'bradley@192.168.7.1:/home/bradley/Downloads/data/', 'Ancho')

function salsaMain(isDataCaptured, varargin)
%% Process arguments 
radarType = 'default'; 
fullDataPath = 'default'; %path from BBB's persepctive (see Example 1)
localDataPath = 'default'; %path from computer's persepctive (no IP address included)

% TODO - (optional) - since all varargin arguments are required, the function could take
% in 3 arguments instead of using 1 required arg and varargin. But this
% format allows for optional inputs to be added later 
for i = 1:length(varargin)
    arg = varargin{i}; 
    % TODO : (optional) - having both radarType AND chipSet is a bit redundant.
    % If consolidated to just chipSet, change C code to expect args matching form of chipSet 
    if strcmp(lower(arg),'ancho')
        radarType = 'ancho'; 
        chipSet = "X2"; 
    elseif strcmp(lower(arg), 'cayenne')
        radarType = 'cayenne'; 
        chipSet = "X1-IPG0"; 
    elseif strcmp(lower(arg), 'chipotle')
        radarType = 'chipotle';
        chipSet = "X1-IPG1"; 
    else
        fullDataPath = arg; 
        
        k = strfind(fullDataPath, '/'); %separate full from local by first "/"
        k = k(1);
        localDataPath = fullDataPath(k:end); 
    end
end

% Check for good inputs 
% TODO : (optional) check to make sure the specified path is valid
if (strcmp(fullDataPath, 'default'))
    error('No file name provided to load data from')
end

if (strcmp(radarType, 'default'))
    error('Please specify a valid radar type')
end

% user specifies a name for the captures 
captureName = input('Enter a name for the data file: ', 's');

%% Specify parameters 
numTrials = 2000; 
runs = 3; 
frameRate = 200; 

%% Capture Data

%TODO - make a function to generate and copy JSON file over to BBB

if isDataCaptured == 1
    
    % Make sure no files already exist in directory with current name
    existingFiles = dir(localDataPath);
    pattern = strcat(captureName, '[0-9]+'); %name of file followed by a run number
    for i = 1:length(existingFiles)
        if regexp(existingFiles(i).name, pattern); %file already exists
            error('File name in use. Change file name or remove existing files'); 
        end
    end
    
    % Start the capture 
    options = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
        captureName, numTrials, runs, frameRate, radarType, fullDataPath); 
    command = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && ./frameLogger %s" &', options); 
    % TODO : understand what the '&' is doing. It is necessary to include this to allow
    % MATLAB to continue execution before the C program ends 
    
    [status,~] = system(command); 
end

%% Load and display Data
frameTot = []; 
runCount = 1; 

while (runCount <= runs)
    %this will detect when a capture named captureName followed by runCount appears
    captureFile = dir(fullfile(localDataPath, strcat(captureName, string(runCount))));
    if length(captureFile) == 1
        
        % make sure file is completely copied onto computer
        pause(1); %TODO : change this to a checksum
        
        % Add new frames to existing frames 
        fileName = captureFile(1).name; 
        [newFrames pgen fs_hz] = salsaLoad(fullfile(localDataPath,fileName), chipSet); 
        frameTot = [frameTot newFrames]; 
        runCount = runCount + 1; 
        
        % Baseband Conversion
        frameCount = size(frameTot, 2); 
        frameTot_bb = zeros(size(frameTot)); 
        for i = 1:frameCount
            frameTot_bb(:,i) = NoveldaDDC(frameTot(:,i), chipSet, pgen, fs_hz); 
        end

        % FFT of signal for each bin
        framesFFT = db(abs(fft(frameTot_bb,frameCount,2)));
        
        % TODO make the plots continuously update instead of close and reopen 
        close all; 
        salsaPlot(frameTot_bb, framesFFT); 
    end
    
    pause(1); %wait 1 second 
    
    % TODO : add early termination by user; be sure to kill BBB process if
    % desired 
end


        

 