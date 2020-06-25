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
%close all
numTrials = 20000;
frameRate = 200; % frames per sec
% TODO - (optional) - since all varargin arguments are required, the function could take
% in 3 arguments instead of using 1 required arg and varargin. But this
% format allows for optional inputs to be added later
if(captureData == true)
    radarType = 'default';
end
fullDataPath = 'default'; %path from BBB's perspective (see Example 1)
localDataPath = 'default'; %path from computer's perspective (no IP address included)

% TODO - (optional) - since all varargin arguments are required, the function could take
% in 3 arguments instead of using 1 required arg and varargin. But this
% format allows for optional inputs to be added later
for i = 1:length(varargin)
    arg = varargin{i};
    % TODO : (optional) - having both radarType AND chipSet is a bit redundant.
    % If consolidated to just chipSet, change C code to expect args matching form of chipSet
    startRange = 300;
    endRange = 512;
    if strcmp(lower(arg),'ancho')
        radarType = 'ancho';
        chipSet = "X2";
        startRange = 110;
        endRange = 240;
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

if (captureData == true && strcmp(radarType, 'default'))
    error('Please specify a valid radar type')
end

% user specifies a name for the captures
captureName = input('Enter a name for the data file: ', 's');
%Avoids going over in buffers
while length(captureName) > 32
    captureName = input('Please enter a shorter name (max 32 characters): ', 's');
end



runs = input('Enter number of desired runs (-1 for indefinite): ');
runs = floor(runs);
while runs ~= -1 && runs < 1
    runs = input('Please enter -1 or an integer number greater than zero: ');
end

noiseRemoval = 0;
if captureData == 0
    subtract = lower(input('Do you have a noise file for spectral removal (Y/N)?: ', 's'));
    while (~strcmp(subtract, 'y') && ~strcmp(subtract, 'n'))
        subtract = lower(input('Please input ''Y'' or ''N'': ', 's'));
    end
    
    %Put noise files in same place as where data is
    if strcmp(subtract, 'y')
        noiseRemoval = 1;
        noiseName = input('Enter the name of the noise file(s): ', 's');
        existingFiles = dir(localDataPath);
        subtractpattern = strcat(noiseName, '[0-9]+');
        noiseCount = 0;
        noiseFFT  = [];
        for index = 1:length(existingFiles)
            condition1 = regexp(existingFiles(index).name, subtractpattern);
            condition2 = isempty(regexp(existingFiles(index).name, 'md5')); %Avoids getting md5 files
            condition = condition1 & condition2;
            if condition
                noiseCount = noiseCount + 1; %Number of noise files
                noisefile = existingFiles(index).name;
                
                [noiseFrames, pgen, fs_hz, chipSet] = salsaLoad(fullfile(localDataPath,noisefile));
                noiseFrameCount = size(noiseFrames, 2);
                noiseFrame_bb = zeros(size(noiseFrames));
                for i = 1:noiseFrameCount
                    noiseFrame_bb(:,i) = NoveldaDDC(noiseFrames(:,i), chipSet, pgen, fs_hz);
                end
                noiseframesFFT = fft(noiseFrame_bb,noiseFrameCount,2);
                noiseFFT(:,:,noiseCount) = noiseframesFFT; %Creates 3D matrix of noise captures
            end
        end
        if noiseCount == 0
            fprintf('No noise files found. Continuing...\n\n')
            noiseRemoval = 0;
        else
            noiseFFT = mean(noiseFFT,3); %Gets mean of noise
        end
    end
end
%% Capture Data

%TODO - make a function to generate and copy JSON file over to BBB

if captureData == 1
    
    % Make sure no files already exist in directory with current name
    existingFiles = dir(localDataPath);
    pattern = strcat(captureName, '[0-9]+'); %name of file followed by a run number
    for i = 1:length(existingFiles)
        if regexp(existingFiles(i).name, pattern) %file already exists
            fprintf('File name in use.\n');
            
            overwrite = lower(input('Would you like to overwrite the file (Y/N)?: ', 's'));
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
    
    
    %Connection check
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
    % figure out which token of the command output is the actual checksum
    for i = 1:size(localchecksum)
        ascii = double(strtrim(lower(localchecksum(i,:))));
        if length(ascii((ascii <= 103 & ascii >= 97) | (ascii <= 75 & ascii >= 48)))
            localchecksum = strtrim(lower(localchecksum(i,:)));
            break
        end
    end
    
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
    
    
    
    
    % Start the capture
    if runs == -1
        %Hardcoding 1000 runs (~160 minutes)
        options = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r 1000 -f %d -t %s -c %s', ...
            captureName, numTrials, frameRate, radarType, fullDataPath);
    else
        options = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
            captureName, numTrials, runs, frameRate, radarType, fullDataPath);
    end

    command = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && ./frameLogger %s " &', options);
    % The '&' is necessary to include this to allow MATLAB to continue execution before the C program ends
    
    [status,~] = system(command);
    
    
end

% TODO: make dft of 3 10-sec capture similar to 30-sec capture, compare

%% Load and display Data
frameTot = [];
runCount = 1;

genFig = figure;
pos1 = get(gcf,'Position');
set(gcf,'Position', pos1 - [pos1(3)/2,0,0,0]);
button = uicontrol('Parent',genFig,...
    'Style','togglebutton',...
    'String','STOP');
button.HandleVisibility = 'off';
addlistener(button,'Value','PostSet',...
    @stopbutton);

while (runCount <= runs) || (runs == -1)
    if button.Value == 1
        break
    end
    %this will detect when a capture named captureName followed by runCount appears
    captureFile = dir(fullfile(localDataPath, strcat(captureName, string(runCount))));
    if ~captureData && isempty(captureFile)
        error(sprintf('File %s not found\n', strcat(captureName, string(runCount))));
    end
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
        
        [newFrames pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(localDataPath,fileName));
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
        filtered = size(frameWindow, 2);
        bg = zeros(size(frameWindow(:,1)));
        frameWindow_bb = zeros(size(frameWindow));
        for i = 1:frameCount
            frameWindow_bb(:,i) = NoveldaDDC(frameWindow(:,i)-bg, chipSet, pgen, fs_hz);
            alpha = 0.5;
            %bg = alpha*frameWindow(:,i) + (1-alpha)*bg;
        end
        
        % FFT of signal for each bin
        %framesFFT = fft(frameWindow_bb,frameCount,2);
        %Would like to use framesFFT for noiseRemoval, but matrix gets
        %bigger, how do we fix this???
        %framesFFT = db(abs(framesFFT));
               
        if button.Value ~= 1
            if runCount ~= 1
                clf
            end
            %salsaPlot(frameWindow_bb, framesFFT, runCount, startRange, endRange);
        else
            break;
        end
        
        %TODO: Put this after first figure plot
        %TODO: Figure out how to open figs side by side
        %TODO: Move plotting into Salsa Plot
        if noiseRemoval
            
            %TODO: Get only last run, this needs to be fixed
            newFrameCount = size(newFrames, 2);
            newFrames_bb = zeros(size(newFrames));
            for i = 1:newFrameCount
                newFrames_bb(:,i) = NoveldaDDC(newFrames(:,i), chipSet, pgen, fs_hz);
            end
            onFramesFFT = fft(newFrames_bb, newFrameCount,2);
            
            
            PSDon = onFramesFFT.*conj(onFramesFFT); %PSD of Signal + Noise
            PSDoff = noiseFFT.*conj(noiseFFT);
            PSD = PSDon - 1.*PSDoff;
            PSD(PSD<0) = 0; %No negative values
            
            firstBin = 1;
            lastBin = 512;
            
            if runCount == 1
                PSDFig = figure;
                pos2 = get(gcf,'Position');
                set(gcf,'Position', pos2 + [pos1(3)/2,0,0,0]);
            end
            figure(PSDFig) %Switch to PSD figure
            clf
            hold on
            fbin = 4800;
            plot(PSDon(firstBin:lastBin,fbin)', '-b')
            plot(PSDoff(firstBin:lastBin, fbin)', ':k');
            plot(PSD(firstBin:lastBin, fbin)', '-r') %Gets PSD only at 80 Hz.
            legend('Signal + Noise', 'Noise', 'Signal (Spectral Subtraction)')
            xlim([firstBin lastBin])
            str = strcat('PSD at 80 Hz Bin, Run #', num2str(runCount));
            title(str)
            hold off
            figure(genFig) %Switch back to general figure
        end
        
        runCount = runCount + 1;
    end
    pause(1)
end
deltas = [];
for i=1:(length(timeDeltas)-1)
    deltas = [deltas timeDeltas(i+1)-timeDeltas(i)];
end
fprintf("Inter-frame spacing goal is %f, achieved %f with stdev %f, min %f, max %f.", 1/200, mean(deltas), std(deltas), max(deltas), min(deltas));
% square = repmat([1 1 -1 -1], 1, numTrials/4);
% pn = square;%[0 1 1 0 1 0 1 0 1 1 1 0 0 0 0 1 0 1 1 1 1 0 0 1 1 0 1 0 1 0 1 0];
% corr = [];
% %for i=pn
% %    if ~i
% %        corr = [corr -1 -1];
% %    else
% %        corr = [corr i i];
% %    end
% %end
% corr = repmat(corr, 1, 32);
% corr = corr(1:length(corr)-1.5*length(pn));
% hold on
%for s=1:length(pn)*2
%    shifted=circshift(corr,s); square_wave_rep = repmat(shifted, size(frameWindow_bb,1), 1); correlatedFrame = sum(frameWindow_bb.*square_wave_rep,2);plot(abs(correlatedFrame), 'DisplayName',string(s)); 
%end
legend
% hold on
% s=0;shifted=circshift(square,s); square_wave_rep = repmat(shifted, size(frameWindow_bb,1), 1); correlatedFrame = sum(frameWindow_bb.*square_wave_rep,2);plot(abs(correlatedFrame))
% s=1;shifted=circshift(square,s); square_wave_rep = repmat(shifted, size(frameWindow_bb,1), 1); correlatedFrame = sum(frameWindow_bb.*square_wave_rep,2);plot(abs(correlatedFrame))
% s=0;shifted=circshift(square,s); square_wave_rep = repmat(shifted, size(frameWindow_bb,1), 1); correlatedFrame = sum(frameWindow_bb.*square_wave_rep,2);plot(abs(correlatedFrame))
% legend('square s0','square s1','ft')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%LOLOLOLO
fprintf('\nDone loading! Plotting...\n')

%%% IMPORTANT DEFS %%%%%%%%
f = 0;%79.96;%0;%%;
d = 4543-90;
radarOffset=3000; %offset in radar settings (mm?)
adjustment = -40; %-33;-13;
smoothingFactor = 13;
phasing = 0; %true=1
offset=round((0.0557*radarOffset+145)/4); %offset induced by tag
divisor = 10;%0;
segSize = size(frameWindow_bb,2)/divisor;
%%%%
load('notag4500outside','notagframeWindow_bb');
bins = zeros(divisor,1);
autoBins = zeros(divisor,1);
vals = zeros(divisor,1);
noise1 = zeros(divisor,1);
noise2 = zeros(divisor,1);
phases = zeros(divisor,1);

for i=1:divisor
    seg = frameWindow_bb(:,(i-1)*segSize+1:i*segSize);
    noiseSeg = notagframeWindow_bb(:,(i-1)*segSize+1:i*segSize);
    %FFT of signal for each bin
    ft = fft(seg,segSize,2);
    %%%%%%% FTIDX and IDX %%%%%%%%%%%%
    ftidx = round(round(f)*(segSize/frameRate))+1;
    if f == 0.1
       ftidx = 6;
    end
    close all;
    figure; 
    idx = round(((d-radarOffset)/4))+adjustment;
    maximum = 0; argmax = 0; 
    if  f > 0   
        idx = idx+offset;
        noiseFT = fft(noiseSeg, segSize, 2);
        noiseFT = noiseFT(:,ftidx);
        %smooth the noise?
        %corrft = fft(noiseFT(:,ftidx)); corrft(smoothingFactor:512-smoothingFactor) = 0; noiseFT = ifft(corrft);
        % Searching phase for correlation argmax
        if phasing
            for phase = linspace(0, 2*pi)
                square_wave = sign(sin(2*pi*f*(0:segSize-1)/frameRate + phase + 1e-8));
                square_wave_rep = repmat(square_wave, size(seg,1), 1);
                correlatedFrame = sum(seg.*square_wave_rep,2);
                m = max(abs(correlatedFrame)); 
                if m >= maximum
                    maximum = m; argmax = phase; 
                end 
                %plot(abs(correlatedFrame)); 
            end
        end
%         % smoothing peaks
%         phase = 0;
%         square_wave = sign(sin(2*pi*f*(0:segSize-1)/frameRate + phase + 1e-8));
%         square_wave_rep = repmat(square_wave, size(seg,1), 1);
%         correlatedFrame = sum(seg.*square_wave_rep,2);
%         plot(abs(correlatedFrame))
%         fprintf('\nPhase argmax is %f rad\n', argmax)
        %noise
%        phase = argmax;
%        square_wave = sign(sin(2*pi*f*(0:segSize-1)/frameRate + phase + 1e-8));
%        square_wave_rep = repmat(square_wave, size(noiseSeg,1), 1);
%        noiseCorr = sum(noiseSeg.*square_wave_rep,2);

         corrft = fft(ft(:,ftidx)); corrft(smoothingFactor:512-smoothingFactor) = 0; lpfd = ifft(corrft);
         lpfd(1:50) = 0;
         bins(i)=idx;
         vals(i) = ft(idx,ftidx);

         [pks,locs,w, p] = findpeaks(abs(lpfd));
         [~,biggest]=max(abs(pks));
         maxidx = locs(biggest);
         autoBins(i) = maxidx;
         fak = lpfd;
         fak(maxidx-45:maxidx + 45) = 0; 
         [pks,locs2,~, ~] = findpeaks(abs(fak));
         [~,second]=max(abs(pks));
         penidx = locs2(second);
%         p(biggest:end) = 0; 
         
         noise1(i) = noiseFT(idx-offset);
         noise2idx = max([locs(biggest) locs2(second)]);
         if maxidx > penidx
             autoBins(i) = penidx;
             noise2idx = min([locs(biggest) locs2(second)]);
         end
         noise2(i) = noiseFT(noise2idx);
        if phasing
            phase = argmax; square_wave = sign(sin(2*pi*f*(0:segSize-1)/frameRate + phase + 1e-8)); square_wave_rep = repmat(square_wave, size(seg,1), 1); correlatedFrame = sum(seg.*square_wave_rep,2); plot(abs(correlatedFrame))
            phases(i) = rad2deg(angle(correlatedFrame(bins(i))));
        end
    else
        load('hitag4500outside','hitagframeWindow_bb');  
        meanHi = mean(hitagframeWindow_bb,2);
        meanFrame = mean(seg,2);
        meanFrame = meanFrame-meanHi; idx=idx+offset;
        meanNoise = mean(notagframeWindow_bb,2);
        vals(i) = meanFrame(idx);
        noise1(i) = meanNoise(idx);
        noise2(i) = max(meanFrame);
    end
    if f > 0
        plot(abs(ft(:,ftidx))); 
        hold on, plot(abs(lpfd)); 
    else
        plot(abs(meanFrame));hold on;
        plot(abs(meanNoise));
    end
    
    if phasing
        plot(abs(correlatedFrame));
        %bins, phases
    end
end
format short g
idx, autoBins, abs(vals), abs(noise1), abs(noise2)
snr1 = 20*log10(abs(vals-noise1)./abs(noise1));
snr2 = 20*log10(abs((vals-noise2)./noise2));
%%%%%%% min       bottom quart              med         top quart      max
stats1=[min(snr1) quantile(snr1,0.25) median(snr1) quantile(snr1,0.75) max(snr1)]
stats2=[min(snr2) quantile(snr2,0.25) median(snr2) quantile(snr2,0.75) max(snr2)]
if phasing
    for k=1:length(phases)
        if k > 1 && abs(corrected(k-1)-phases(k)) > 100
            if corrected(k-1)-phases(k) > 0
                corrected(k) = phases(k)+180;
            else
                corrected(k) = phases(k)-180;
            end
        else
            corrected(k) = phases(k);
        end
    end
    figure; plot(corrected'); ylim([-22 180]); grid on
    bins, corrected'
end
close all;
if f > 0
    plot(abs(ft(:,ftidx))); 
    hold on, plot(abs(lpfd)); 
    plot(abs(noiseFT));
    plot(idx,abs(ft(idx,ftidx)), 'g*')
    legend('fft','lpf','noise','tag idx')
else
    plot(abs(meanFrame));hold on;
    plot(abs(meanNoise));
    plot(idx,abs(meanFrame(idx)), 'g*')
    legend('mean signal','mean noise','tag idx')
end
end


function stopbutton(hObject, eventdata)
killcommand = sprintf('ssh root@192.168.7.2 "pkill frame"'); %Kills processes with "frame" in name
[status,~] = system(killcommand);
fprintf('Stopping...\n')

eventdata.AffectedObject.Value = 1;
eventdata.AffectedObject.Interruptible = false;
eventdata.AffectedObject.ForegroundColor = 'red';
eventdata.AffectedObject.String = 'STOPPED';

fprintf('Program paused. Please use "Ctrl+C" to resume and finish.\n')
uiwait()
end

