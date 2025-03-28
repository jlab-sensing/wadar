function fileName = CaptureDataset(captureName, localDataPath, numCaptures, frameCount, frameRate, radarType)
% fileName = CaptureDataset(captureName, localDataPath, numCaptures, frameCount, frameRate, radarType)
%
% Function to acquire a radar capture and save it to a file. This function
% sends a command to the radar to start a capture and then waits for the
% capture to be saved to the specified directory. The function will check
% for the existence of the file and then return the name of the file.
%
% Inputs:
%   captureName: Name of the data capture file.
%   localDataPath: Path to the data capture file.
%   numCaptures: Number of captures to take.
%   frameCount: Number of frames to capture.
%   frameRate: Frame rate in Hz.
%   radarType: Type of radar to capture data from.
%
% Outputs:
%   fileName: Name of the saved data capture file.

% Setting default parameters
arguments 
    captureName = 'untitled'
    localDataPath = strcat(pwd, '/data')
    numCaptures = 10
    frameCount = 2000
    frameRate = 200
    radarType = 'Chipotle'
end