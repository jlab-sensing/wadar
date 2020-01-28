d=0.17; %meters
% = "/Users/cjoseph/Documents/research/radar/matlab/data/demo";
localDataPath = "/Users/cjoseph/radar/matlab/data/demo";
fullDataPath = sprintf("cjoseph@192.168.7.1:%s",localDataPath);
radarType="Chipotle";
numTrials = 2000;
frameRate = 200; 
corrTemplateFile = "template";
airCaptureFile = "air";
captureName = "demoDump";

%% loading template %%
[tempRawFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath, corrTemplateFile));
tempFrameCount = size(tempRawFrames, 2);
tempFramesBB = zeros(size(tempRawFrames));

for j = 1:tempFrameCount
    tempFramesBB(:,j) = NoveldaDDC(tempRawFrames(:,j), chipSet, pgen, fs_hz);
end

tempFT = fft(tempFramesBB,tempFrameCount,2); 

% choose the frequency based on the capture duration 
if length(tempFT(1,:)) <= frameRate * 30 % <= 30s
    freqTag = 80 / frameRate * length(tempFT(1,:)) + 1;      
elseif length(tempFT(1,:)) == frameRate * 100 % 100s
    freqTag = 80 / frameRate * length(tempFT(1,:)) + 0;  
elseif length(tempFT(1,:)) == frameRate * 300 % 300s
    freqTag = 80 / frameRate * length(tempFT(1,:)) - 1;
else
    error('unrecognized capture duration') 
end

%TODO: improve this?
tempTagFT = abs(tempFT(:, freqTag)); 
% find the bin corresponding to the largest peak 
[val, binMax] = max(tempTagFT);
%templatePeakBin = [templatePeakBins binMax]; 
% find left-most peak matching criteria 
h1 = mean(findpeaks(tempTagFT(1:100)));
h2 = max(tempTagFT); 
thresholdAdjust = 0.9; % factor for adjusting which peaks are considered valid
threshold = thresholdAdjust * (h1 + h2) / 2; 
%threshold = 0.85 * max(tempTagFT);  
[peaks peakBins] = findpeaks(tempTagFT, 'MinPeakHeight', threshold); 
peakBins = peakBins(peakBins > 22); % assume no peak in first 22
templatePeakBin = peakBins(1); 

%% loading air capture %%
[airRawFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath, airCaptureFile));
airFrameCount = size(airRawFrames, 2);
airFramesBB = zeros(size(airRawFrames));

for j = 1:airFrameCount
    airFramesBB(:,j) = NoveldaDDC(airRawFrames(:,j), chipSet, pgen, fs_hz);
end

airFT = fft(airFramesBB,airFrameCount,2); 

%TODO: improve this?
airTagFT = abs(airFT(:, freqTag)); 
% find the bin corresponding to the largest peak 
[val, binMax] = max(airTagFT);
%templatePeakBins = [templatePeakBins binMax]; 
% find left-most peak matching criteria 
h1 = mean(findpeaks(airTagFT(1:100)));
h2 = max(airTagFT); 
thresholdAdjust = 0.9; % factor for adjusting which peaks are considered valid
threshold = thresholdAdjust * (h1 + h2) / 2; 
%threshold = 0.85 * max(tempTagFT);  
[peaks peakBins] = findpeaks(airTagFT, 'MinPeakHeight', threshold); 
peakBins = peakBins(peakBins > 22); % assume no peak in first 22
airPeakBin = peakBins(1);

% Make sure no files already exist in directory with current name
existingFiles = dir(localDataPath);
pattern = strcat(captureName, '[0-9]+'); %name of file followed by a run number
for i = 1:length(existingFiles)
    if regexp(existingFiles(i).name, pattern) %file already exists
        overwrite = lower(input('Would you like to overwrite the existing files (Y/N)?: ', 's'));
        while (~strcmp(overwrite, 'y') && ~strcmp(overwrite, 'n'))
            overwrite = lower(input('Please input ''Y'' or ''N'': ', 's'));
        end

        if strcmp(overwrite, 'y')
            for j = 1:length(existingFiles)
                %Delete all matching files, now we can procede to
                %writing files.
                if regexp(existingFiles(j).name, pattern)
                    delete(fullfile(localDataPath, existingFiles(j).name));
                end
            end
        else
            error('Change file name or remove existing files.');
        end
        break %Only want to go through this on first occurence of file match!!!
    end
end

%% Connection check %%
fprintf('\nPlease wait. Verifying radar connection...\n')
checkcmd = sprintf('ping -c 3 192.168.7.2');
[checkstatus, checkcmdout] = system(checkcmd);
if checkstatus ~= 0 
    fprintf('\n')
    msg = 'Connection failed. Please check the connection to the radar.';
    error(msg)
else
    fprintf('Connection successful!\n')
end

%Framelogger check: 1 second capture
%Name is captureName.check1
checkoptions = sprintf('-s ../data/captureSettings -l ../data/%s.check -n %d -r 1 -f %d -t %s -c %s', ...
    captureName, frameRate, frameRate, radarType, fullDataPath);
checkcommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', checkoptions);
[status,~] = system(checkcommand);
fprintf('\nPlease wait. Verifying framelogger captures...\n');
pause(5); %5 seconds to transfer files
checkFile = dir(fullfile(localDataPath, strcat(captureName, '.check1')));
checkmd5File = dir(fullfile(localDataPath, strcat(captureName, '.check1', '.md5')));
if (length(checkFile) ~= 1) || (length(checkmd5File) ~= 1)
    error('There is a data transfer issue. Please verify your capture settings and scp directory.')
end
fileName = checkFile(1).name;
md5Name = checkmd5File(1).name;

md5command = sprintf('md5 %s', fullfile(localDataPath, fileName));
[status, cmdout] = system(md5command);
localchecksum = char(strsplit(cmdout));
localchecksum = lower(strtrim(localchecksum(4,:)));
localchecksum = deblank(localchecksum);

md5checksum = fileread(fullfile(localDataPath, md5Name));
md5checksum = char(strsplit(md5checksum));
md5checksum = lower(md5checksum(1,:));
md5checksum = deblank(localchecksum);

%Avoid overwriting data file
delete(fullfile(localDataPath, strcat(captureName, '.check1')));
delete(fullfile(localDataPath, strcat(captureName, '.check1', '.md5')));

if (~strcmp(localchecksum, md5checksum))
    fprintf('Failure on framelogger check.\n', runCount);
    fprintf('Local checksum is %s.\n', localchecksum);
    fprintf('BBB checksum is %s.\n', md5checksum);
    error('Uh oh. There has been an error in the file transfer. The md5 hashes do not match.\n')
else
    fprintf('Framelogger successful!\n\n')
end

%% start capture %%
%TODO: lol fix this REU crap of hardcoding 160 mins
%Hardcoding 1000 runs (~160 minutes)
options = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r 1000 -f %d -t %s -c %s', ...
                    captureName, numTrials, frameRate, radarType, fullDataPath);
command = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && ./frameLogger %s " &', options);
% The '&' is necessary to include this to allow MATLAB to continue execution before the C program ends
[status,~] = system(command);

%% Open demo graphics figure
frameTot = [];
runCount = 1;

genFig = figure('Position', [10 10 900 600]);
pos1 = get(gcf,'Position');
set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]);
button = uicontrol('Parent',genFig,...
    'Style','togglebutton',...
    'String','STOP');
button.HandleVisibility = 'off';
addlistener(button,'Value','PostSet',...
    @stopbutton);

%% Demo capture loop
vwcList = [];
timer = 0;
while true
    if button.Value == 1
        break
    end
    %this will detect when a capture named captureName followed by runCount appears
    captureFile = dir(fullfile(localDataPath, strcat(captureName, string(runCount))));
    md5File = dir(fullfile(localDataPath, strcat(captureName, string(runCount), '.md5')));
    
    if length(md5File) == 1 && length(captureFile) == 1
        % Add new frames to existing frames
        fileName = captureFile(1).name;
        md5Name = md5File(1).name;
        
        %Get correct string in cmdout
        %Format is: MD5 (filename) = checksum
        md5command = sprintf('md5 %s', fullfile(localDataPath, fileName));
        [status, cmdout] = system(md5command);
        %for linux;
        if status ~= 0
            md5command = sprintf('md5sum %s', fullfile(localDataPath, fileName));
            [status, cmdout] = system(md5command);
        end
        %Split into strings (cell array)
        for s = strsplit(cmdout) 
            %find longest string in cell array and convert to char
            localchecksum = deblank(lower(strtrim(s{1})));
            if (isempty(strfind(localchecksum,'/')) && length(localchecksum) == 32)
                break
            end         
        end
        %Get correct string in file 
        %Format is: checksum filename (cell array type)
        md5checksum = fileread(fullfile(localDataPath, md5Name));
        %Separates into strings and converts to appropriate type
        md5checksum = char(strsplit(md5checksum));
        %Gets only checksum (not filename) and puts in lowercase and
        %removes trailing blank space
        md5checksum = deblank(lower(md5checksum(1,:)));
        
        if (~strcmp(localchecksum, md5checksum))
            killcommand = sprintf('ssh root@192.168.7.2 "pkill frame"'); %Kills processes with "frame" in name
            [status,~] = system(killcommand);

            fprintf('\nRun number is #%d.\n', runCount);
            fprintf('Local checksum is %s.\n', localchecksum);
            fprintf('BBB checksum is %s.\n', md5checksum);
            error('Uh oh. There has been an error in the file transfer. The md5 hashes do not match.\n')
        end
        
        [frames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath,fileName));
        % Baseband Conversion
        frameCount = size(frames, 2);
        framesBB = zeros(size(frames));
        for i = 1:frameCount
            framesBB(:,i) = NoveldaDDC(frames(:,i), chipSet, pgen, fs_hz);
        end
        
        % FFT of signal for each bin
        ft = fft(framesBB(:,1:frameCount),frameCount,2);
        [peak confidence ftProcessed shiftedTemp] = determinePeak(tempTagFT,templatePeakBin, ft, frameRate, "corr"); 
        vwc = calculateSoilMoisture(airPeakBin, peak, "farm", d); 
        vwcList = [vwcList vwc];

        if button.Value ~= 1
            if runCount ~= 1
                clf
            end
            %% display data %%
            % correlation plot
            subplot(2,2,1);
            plot(ftProcessed, 'displayname','80Hz FFT bin of signal'); hold on;
            plot(shiftedTemp, 'DisplayName', '80Hz FFT bin of fingerprint'); hold on; 
            plot(peak, ftProcessed(peak), 'o', 'DisplayName', sprintf('detected peak = %i', peak));
            title('detected peaks') 
            xlabel('bin')
            ylabel('normalized magnitude') 
            legend();
            grid on
            
            % VWC plot
            subplot(2,2,3);
            plot(vwcList, '-o')
            ylim([0 0.6])
            ylabel("water content")
            xlabel("time")
            xlim([0 length(vwcList)+5])
            grid on
            title("VWC history")
            
            % stats?
            subplot(2,2,2);
            
            text(0.1,0.9, sprintf('TIMER: %i', timer),'fontsize',18); axis off
            text(0.1,0.7, sprintf('Capture duration: %i', numTrials/frameRate), 'fontsize',18); 
            text(0.1,0.5, sprintf('SNR: 21dB'), 'fontsize',18); 
            text(0.1,0.3, sprintf('Confidence: %f', confidence), 'fontsize',18);   
            
            subplot(2,2,4);
            bar(vwc)
            ylim([0 0.6])
            grid on
            title(sprintf("Current VWC = %.3f", vwc))

        else
            break;
        end
        timer = 0;
        runCount = runCount + 1;
    end
    timer = timer + 1;
    pause(1)
end    