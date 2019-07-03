% USAGE: Captures or loads previous radar captures and displays useful results

% REQUIRED ARGS:
% captureData: Boolean specifying whether data should be captured (true) or
% loaded from file (false), no quotations needed
% fullDataPath: string to specify full path to data
% radar type: Ancho or Cayenne or Chipotle

% EXAMPLE 1: Capture and display data from Ancho, with save
% salsaMain(1, 'bradley@192.168.7.1:/home/bradley/Downloads/data/', 'Ancho')

function salsaMain(captureData, varargin)
%% Process arguments
radarType = 'default';
fullDataPath = 'default'; %path from BBB's perspective (see Example 1)
localDataPath = 'default'; %path from computer's perspective (no IP address included)

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
%runs = 3;  Removed to specify user input
frameRate = 200; % frames per sec

runs = input('Enter number of desired runs (-1 for indefinite): ');
runs = floor(runs);
while runs ~= -1 && runs < 1
    runs = input('Please enter -1 or an integer number greater than zero: ');
end

%% Capture Data

%TODO - make a function to generate and copy JSON file over to BBB

if captureData == 1
    
    % Make sure no files already exist in directory with current name
    existingFiles = dir(localDataPath);
    pattern = strcat(captureName, '[0-9]+'); %name of file followed by a run number
    for i = 1:length(existingFiles)
        if regexp(existingFiles(i).name, pattern) %file already exists
            error('File name in use. Change file name or remove existing files');
        end
    end
    
    % Start the capture
    if runs == -1
        %Hardcoding 1000 runs (~160 minutes)
        options = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r 1000 -f %d -t %s -c %s', ...
            captureName, numTrials, frameRate, radarType, fullDataPath);
    else
        options = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
            captureName, numTrials, runs, frameRate, radarType, fullDataPath);
    end
    %textoptions = ' 2>&1 ~/log.txt';
    command = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && ./frameLogger %s " &', options);
    % TODO : understand what the '&' is doing. It is necessary to include this to allow
    % MATLAB to continue execution before the C program ends
    
    [status,~] = system(command);
    
    
end

% TODO: add ssh timeout, check for status
% TODO: check ssh validity: ssh into radar, scp file into matlab directory,
% check it exists
% TODO: check validity of user directory
% TODO: ssh into radar, run framelogger for short time, redirect output to
% file (check for cayenne, ancho, chipotle)
% TODO: make dft of 3 10-sec capture similar to 30-sec capture, compare

%% Load and display Data
frameTot = [];
runCount = 1;

% if status ~= 0
%     fprintf('There is a problem with the linux command.\n')
%     exit
% end

myFig = figure;
button = uicontrol('Parent',myFig,...
    'Style','togglebutton',...
    'String','STOP');
button.HandleVisibility = 'off';
%button = uicontrol('Style', 'togglebutton', 'String', 'STOP');

addlistener(button,'Value','PostSet',...
    @stopbutton);
%el = addlistener(uicontrol, 'Value', 'PostSet', @stopbutton);

% button.Style = 'togglebutton';
% button.String = 'STOP';

while (runCount <= runs) || (runs == -1)
    if button.Value == 1
        break
    end
    %this will detect when a capture named captureName followed by runCount appears
    captureFile = dir(fullfile(localDataPath, strcat(captureName, string(runCount))));
    if length(captureFile) == 1
        
        % make sure file is completely copied onto computer
        pause(1); %TODO : change this to a checksum
        
        % Add new frames to existing frames
        fileName = captureFile(1).name;
        [newFrames pgen fs_hz] = salsaLoad(fullfile(localDataPath,fileName), chipSet);
        frameTot = [frameTot newFrames];
        frameWindow = frameTot;
        %Getting only last 10 captures to avoid DDC and FFT slowdown,
        %this is a temporary workaround
        if runCount > 10
            frameWindow = frameTot(:,(runCount-10)*(numTrials)+1:end); %Get last 10 captures only
        end
        
        %TODO: Change code below to better process several captures
        
        % Baseband Conversion
        frameCount = size(frameWindow, 2);
        frameWindow_bb = zeros(size(frameWindow));
        for i = 1:frameCount
            frameWindow_bb(:,i) = NoveldaDDC(frameWindow(:,i), chipSet, pgen, fs_hz);
        end
        
        % FFT of signal for each bin
        framesFFT = db(abs(fft(frameWindow_bb,frameCount,2)));
        
        if button.Value ~= 1
            if runCount ~= 1
                clf
            end
            salsaPlot(frameWindow_bb, framesFFT, runCount);
        else
            break;
        end
        
        runCount = runCount + 1;
    end
    
    pause(1); %wait 1 second
end

fprintf('Done!\n')
end


function stopbutton(hObject, eventdata)
command = sprintf('ssh root@192.168.7.2 "pkill frame"'); %Kills processes with "frame" in name
[status,~] = system(command);
fprintf('Stopping...\n')

eventdata.AffectedObject.Value = 1;
eventdata.AffectedObject.Interruptible = false;
eventdata.AffectedObject.ForegroundColor = 'red';
eventdata.AffectedObject.String = 'STOPPED';

fprintf('Program paused. Please use "Ctrl+C" to resume and finish.\n')
uiwait()
end

