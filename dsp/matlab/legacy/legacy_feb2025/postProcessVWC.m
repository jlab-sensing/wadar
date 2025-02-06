% This processing script assumes strongly structured filenames in the form of
%       'DDDDmm_ofsX-X_YYY_100s1'
% DDDDmm = distance to tag (in mm) measured w/ laser ranger
% ofsX-X = radar offset setting (in m)
% YYY    = tag oscillation frequency, e.g. 80hz, or NONE
function postProcessVWC(varargin)
    %% IMPORTANT DEFS %%%%%%%%
    ftindices = containers.Map({10,30,100,300},[801,2401,8000,23999]);
    %binLocs = containers.Map({0,1,2,3,4},[134,175,200,222,288]); %clay
    %binLocs = containers.Map({0,1,2,3,4},[141,180,200,222,277]); %farm
    %binLocs = containers.Map({0,1,2,3,4},[144,195,217,286,315]); %silt
    %dirs={'/home/cjoseph/Documents/radar/matlab/data/peak_detect/silt_passive'};
    dirs={'/home/cjoseph/Documents/radar/matlab/data/autumn19/FarmPassive/0can/'};
    frameRate = 200; % frames per sec
    divisor = 1; % how many chunks to divide the capture into
    radarType = 'chipotle';
    chipSet = "X1-IPG1";
    adjustment = 0; %-33;-13; % how many bins to adjust peak location
    smoothingFactor = 13; %how much to smooth during peakfinding, lower is smoother
    phasing = 0; %true=1
    f=80;
    snr1 = [];
    close all;
    %% Process arguments
    for i = 1:length(varargin) %build files list
        fh = char(varargin(i));
        files{i} = fh;
        %extract distance from filename
        butt  = strfind(fh, 'can') - 1;
        wets(i) = str2double(fh(butt));
    end
    
    %extract duration from filename
    head = 11;
    butt  = strfind(fh, 's') - 1;
    dur = str2double(fh(head:butt)); 
    
    %% load files
    md5file='';
    for fh=files
        fidx = find(ismember(files,fh));
        fh = char(fh);
        type=fh(end-5);
        % look in all the dirs for the file, then load it
        for d=dirs
            d = char(d);
            captureFile = dir(fullfile(d, fh));
            if isempty(captureFile)
                continue
            end
            md5file = dir(fullfile(d, strcat(fh, '.md5')));
            break
        end
        if isempty(md5file)
            error('could not find file (of md5) %s',fh)
        end
        
        % do the md5 checks
        if ~md5check(captureFile, md5file) % signal file
            error('signal file %s failed md5 check', fullfile(d,fh))
        end

        % Load the files
        [frameWindow pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(captureFile.folder,captureFile.name));
        
        % Baseband Conversion
        frameWindow_bb = toBaseband(frameWindow, chipSet, pgen, fs_hz);
        
        deltas = [];
        for i=1:(length(timeDeltas)-1)
            deltas = [deltas timeDeltas(i+1)-timeDeltas(i)];
        end
        fprintf("Inter-frame spacing goal is %f, achieved %f with stdev %f, min %f, max %f.\n", 1/frameRate, mean(deltas), std(deltas), max(deltas), min(deltas));

        %% Process the data
        [bins, autoBins, noiseBins, sigVal, noiseVal] = deal(zeros(divisor,1));
        segSize = size(frameWindow_bb,2)/divisor;

        for i=1:divisor
            seg = frameWindow_bb;
            
            %FFT of signal for each bin
            ft = fft(seg,length(seg),2);
            %%%%%%% FTIDX and IDX %%%%%%%%%%%%
            ftidx = ftindices(dur);
            %idx = binLocs(wets(fidx))+adjustment;

            corrft = fft(ft(:,ftidx)); corrft(smoothingFactor:512-smoothingFactor) = 0; lpfd = ifft(corrft);
            lpfd(1:50) = 0;
            

            [sigIdx, noiseIdx] = findSCR(ft,lpfd, ftidx);
            sigIdx=211;
            autoBins(i) = sigIdx; 
            bins(i)= sigIdx;%idx;
            %sigVal(i) = ft(idx,ftidx);
            sigVal(i) = ft(sigIdx,ftidx);
            tmp=lpfd;
            tmp(sigIdx-100:end) = 0; %remove peak area from array
            [pks,locs,~, ~] = findpeaks(abs(tmp));
            [~,third]=max(abs(pks));
            noiseIdx = 100;%locs(third);
            noiseVal(i) = ft(noiseIdx,ftidx); noiseBins(i) = noiseIdx;
            
            %% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%
            figure; 
            plot(abs(ft(:,ftidx)),'g'); 
            hold on, plot(abs(lpfd),'g:'); 
            plot(bins(i),abs(ft(bins(i),ftidx)), 'gx')
            plot(autoBins(i),abs(ft(autoBins(i),ftidx)), 'g*')
            plot(noiseBins(i),abs(ft(noiseBins(i),ftidx)), 'go')

            legend('ft','lpf','idx','autoIdx','noiseIdx')
            title(fh,'Interpreter', 'none')
            pause(1)
        end

%         figure; 
%         plot(abs(ft(:,ftidx)),'g'); 
%         hold on, plot(abs(lpfd),'g:'); 
%         plot(autoBins(i),abs(ft(autoBins(i),ftidx)), 'g*')
%         plot(noiseBins(i),abs(ft(noiseBins(i),ftidx)), 'go')
% 
%         legend('ft','lpf','idx','noiseIdx')
%         title(fh,'Interpreter', 'none')
%         pause(1)
%         %idx, autoBins, abs(vals), abs(noise1), abs(noise2)
         snr1 = [snr1; 20*log10(abs(sigVal./noiseVal))];
    end
    
    format short g
    %% output SNR stats
%     snr1=snr1(1:end-10);
%     snr2=snr2(1:end-10);
    %%      approxDist min       bottom quart              med         top quart      max
    %SNRstats  =[approxDist min(snr1) quantile(snr1,0.25) median(snr1) quantile(snr1,0.75) max(snr1)]
    %peakStats =[approxDistst min(snr2) quantile(snr2,0.25) median(snr2) quantile(snr2,0.75) max(snr2)]
    snr1
    snrStats =[min(snr1) quantile(snr1,0.25) median(snr1) quantile(snr1,0.75) max(snr1)]
end


function check = md5check(fh, md5fh)
    %Get correct string in cmdout
    %Format is: MD5 (filename) = checksum
    md5command = sprintf('md5 %s', fullfile(fh.folder, fh.name));
    [status, cmdout] = system(md5command);
    %use different md5 command on linux;
    if status ~= 0
        md5command = sprintf('md5sum %s', fullfile(fh.folder, fh.name));
        [status, cmdout] = system(md5command);
    end
    %Split into strings (cell array)
    for s = strsplit(cmdout) 
        % find longest string in cell array and convert to char
        localchecksum = deblank(lower(strtrim(s{1})));
        if (isempty(strfind(localchecksum,'/')) && length(localchecksum) == 32)
            break
        end         
    end
    %Get correct string in file 
    %Format is: checksum filename (cell array type)
    md5checksum = fileread(fullfile(md5fh.folder, md5fh.name));
    %Separates into strings and converts to appropriate type
    md5checksum = char(strsplit(md5checksum));
    %Gets only checksum (not filename) and puts in lowercase and
    %removes trailing blank space
    md5checksum = deblank(lower(md5checksum(1,:)));
        
    check = strcmp(localchecksum, md5checksum);
end


function framesBB = toBaseband(frames, chipSet, pgen, fs_hz)    
    frameCount = size(frames, 2);
    framesBB = zeros(size(frames));
    for i = 1:frameCount
        framesBB(:,i) = NoveldaDDC(frames(:,i), chipSet, pgen, fs_hz);
    end
end

function [sigIdx, noiseIdx] = findSCR(ft,lpfd, ftidx)
        [pks,locs,~, ~] = findpeaks(abs(lpfd));
        [~,biggest]=max(abs(pks));
        biggestIdx = locs(biggest);
        sigIdx = biggestIdx;
        
        tmp = lpfd;
        tmp(max(1,biggestIdx-70):biggestIdx + 45) = 0; %remove peak area from array
        [pks2,locs2,~, ~] = findpeaks(abs(tmp));
        [~,second]=max(abs(pks2));
        secondIdx = locs2(second);
        
        sigIdx = biggestIdx;
        if biggestIdx > secondIdx && abs(ft(biggestIdx,ftidx)/ft(secondIdx,ftidx)) < 2.2
            sigIdx = secondIdx;
        end
        
        tmp(sigIdx-50:end) = 0; %remove 2nd peak area from array
        [pks3,locs3,~, ~] = findpeaks(abs(tmp));
        [~,third]=max(abs(pks3));
        noiseIdx = locs3(third);
end