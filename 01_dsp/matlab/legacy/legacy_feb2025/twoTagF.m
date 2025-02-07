% This processing script assumes strongly structured filenames in the form of
%       'DDDDmm_ofsX-X_YYY_100s1'
% DDDDmm = distance to tag (in mm) measured w/ laser ranger
% ofsX-X = radar offset setting (in m)
% YYY    = tag oscillation frequency, e.g. 80hz, or NONE

function twoTagF(varargin)
    %% IMPORTANT DEFS %%%%%%%%
    dirs={'/home/cjoseph/Documents/radar/matlab/data/twoTag/'};
    dirs=reshape({'/home/ericdvet/jlab/wadar/matlab/data/LOSsnrBedroom/'
                 '/home/ericdvet/jlab/wadar/matlab/data/LOSsnrKitchen/'
                 '/home/ericdvet/jlab/wadar/matlab/data/LOSsnrOutside/'}, [1 3]);
    frameRate = 200; % frames per sec
    divisor = 10; % how many chunks to divide the capture into
    z=1;
    radarType = 'chipotle';
    chipSet = "X1-IPG1";
    adjustment = -40;%-7;%-40,-22; %-33;-13; % how many bins to adjust peak location
    smoothingFactor = 13; %how much to smooth during peakfinding, lower is smoother
    phasing = 0; %true=1
    ftidx1 = 801;
    ftidx2 = 101;
    f1=80;
    f2=10;
    [snr1, snr2] = deal([]);
    close all;
    %% Process arguments
    %if length(varargin) < 3
    %    error('Please provide the names of at least 3 capture files names')
    %end
    for i = 1:length(varargin) %build files list
        fh = char(varargin(i));
        files{i} = fh;
    end
    radarOffset = 0;
    
    %% load files
    for fh=files
        fidx = find(ismember(files,fh));
        fh = char(fh);
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
            error('could not find filename (or md5)')
        end
        
        % do the md5 checks
        if ~md5check(captureFile, md5file) % signal file
            error('signal file failed md5 check')
        end

        % Load the files
        [frameWindow pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(captureFile.folder,captureFile.name));
        
        % Baseband Conversion
        frameWindow_bb = toBaseband(frameWindow, chipSet, pgen, fs_hz);
        
%         frameWindow_bb=frameWindow_bb(:,1:20000/z);
%         noTagFrameWindow_bb=noTagFrameWindow_bb(:,1:20000/z);
%         hiTagFrameWindow_bb=hiTagFrameWindow_bb(:,1:20000/z);
        
        deltas = [];
        for i=1:(length(timeDeltas)-1)
            deltas = [deltas timeDeltas(i+1)-timeDeltas(i)];
        end
        fprintf("Inter-frame spacing goal is %f, achieved %f with stdev %f, min %f, max %f.", 1/frameRate, mean(deltas), std(deltas), max(deltas), min(deltas));

        %% Process the data
        [bins, autoBins1, autoBins2, noiseBins1, noiseBins2, noise1, noise2, sig1, sig2] = deal(zeros(divisor,1));
        offset=round((0.0557*radarOffset+145)/4); %offset induced by tag
        segSize = floor(size(frameWindow_bb,2)/divisor);
        
        for i=1:divisor
            %i
            seg = frameWindow_bb(:,(i-1)*segSize+1:i*segSize);
            
            %FFT of signal for each bin
            ft = fft(seg,segSize,2);
            
            %%%%%%% FTIDX and IDX %%%%%%%%%%%
            %idx = round(((dists(fidx)-radarOffset)/4))+adjustment;
            %idx = idx+offset;
            corrft = fft(ft(:,ftidx1)); corrft(smoothingFactor:512-smoothingFactor) = 0; lpfd1 = ifft(corrft);
            corrft = fft(ft(:,ftidx2)); corrft(smoothingFactor:512-smoothingFactor) = 0; lpfd2 = ifft(corrft);
            lpfd1(1:22) = 0; lpfd2(1:22) = 0; 
            %bins(i)=idx;
            
            [sigIdx, noiseIdx] = findSCR(ft,lpfd1, ftidx1);
            sig1(i) = ft(sigIdx, ftidx1); autoBins1(i) = sigIdx;
            noise1(i) = ft(noiseIdx,ftidx1); noiseBins1(i) = noiseIdx;
            
            [sigIdx, noiseIdx] = findSCR(ft,lpfd2, ftidx2);
            sig2(i) = ft(sigIdx, ftidx2); autoBins2(i) = sigIdx;
            noise2(i) = ft(noiseIdx,ftidx2); noiseBins2(i) = noiseIdx;
            
            %% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             figure; 
%             plot(abs(ft(:,ftidx1)),'g'); 
%             hold on, plot(abs(lpfd1),'g:'); 
%             plot(autoBins1(i),abs(ft(autoBins1(i),ftidx1)), 'g*')
%             plot(noiseBins1(i),abs(ft(noiseBins1(i),ftidx1)), 'go')
%             
%             plot(abs(ft(:,ftidx2)),'m'); 
%             hold on, plot(abs(lpfd2),'m:'); 
%             plot(autoBins2(i),abs(ft(autoBins2(i),ftidx2)), 'm*')
%             plot(noiseBins2(i),abs(ft(noiseBins2(i),ftidx2)), 'mo')
% 
%             legend('ft1','lpf1','idx1','noise1','ft2','lpf2','idx2','noise2')
%             title(fh,'Interpreter', 'none')
%             pause(1)
        end
        figure; 
        plot(abs(ft(:,ftidx1)),'g'); 
        hold on, plot(abs(lpfd1),'g:'); 
        plot(autoBins1(i),abs(ft(autoBins1(i),ftidx1)), 'g*')
        plot(noiseBins1(i),abs(ft(noiseBins1(i),ftidx1)), 'go')

        plot(abs(ft(:,ftidx2)),'m'); 
        hold on, plot(abs(lpfd2),'m:'); 
        plot(autoBins2(i),abs(ft(autoBins2(i),ftidx2)), 'm*')
        plot(noiseBins2(i),abs(ft(noiseBins2(i),ftidx2)), 'mo')

        legend('ft1','lpf1','idx1','noise1','ft2','lpf2','idx2','noise2')
        title(fh,'Interpreter', 'none')
        pause(1)
        %idx, autoBins, abs(vals), abs(noise1), abs(noise2)
        %trueRange = [trueRange; idx*ones(divisor,1)];
        %radarRange = [radarRange; autoBins];
        snr1 = [snr1; 20*log10(abs(sig1./noise1))];
        snr2 = [snr2; 20*log10(abs(sig2./noise2))];
        
    end
    
    format short g
    %% output SNR stats
%     snr1=snr1(1:end-10);
%     snr2=snr2(1:end-10);
    %rErr = (abs(trueRange-radarRange)*4)/10.0; %range error in cm
    %%      approxDist min       bottom quart              med         top quart      max
    %SNRstats  =[approxDist min(snr1) quantile(snr1,0.25) median(snr1) quantile(snr1,0.75) max(snr1)]
    snr1stats =[min(snr1) quantile(snr1,0.25) median(snr1) quantile(snr1,0.75) max(snr1)]
    snr2stats =[min(snr2) quantile(snr2,0.25) median(snr2) quantile(snr2,0.75) max(snr2)]
    %both79stats =[min(snrBoth79) quantile(snrBoth79,0.25) median(snrBoth79) quantile(snrBoth79,0.75) max(snrBoth79)]
    %solo80stats =[min(snrSolo80) quantile(snrSolo80,0.25) median(snrSolo80) quantile(snrSolo80,0.75) max(snrSolo80)]
    %solo79stats =[min(snrSolo79) quantile(snrSolo79,0.25) median(snrSolo79) quantile(snrSolo79,0.75) max(snrSolo79)]
    %rangeStats=[min(rErr) quantile(rErr,0.25) median(rErr) quantile(rErr,0.75) max(rErr)]
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
        tmp(max(1,biggestIdx-50):biggestIdx + 45) = 0; %remove peak area from array
        [pks2,locs2,~, ~] = findpeaks(abs(tmp));
        [~,second]=max(abs(pks2));
        secondIdx = locs2(second);
        
        sigIdx = biggestIdx;
        if biggestIdx > secondIdx && abs(ft(biggestIdx,ftidx)/ft(secondIdx,ftidx)) < 2.2
            sigIdx = secondIdx;
        end
        
        tmp(sigIdx-70:end) = 0; %remove 2nd peak area from array
        [pks3,locs3,~, ~] = findpeaks(abs(tmp));
        [~,third]=max(abs(pks3));
        noiseIdx = locs3(third);
end
