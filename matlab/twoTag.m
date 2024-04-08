% This processing script assumes strongly structured filenames in the form of
%       'DDDDmm_ofsX-X_YYY_100s1'
% DDDDmm = distance to tag (in mm) measured w/ laser ranger
% ofsX-X = radar offset setting (in m)
% YYY    = tag oscillation frequency, e.g. 80hz, or NONE

function twoTag(varargin)
    %% IMPORTANT DEFS %%%%%%%%
    dirs={'/home/ericdvet/jlab/wadar/matlab/data/twoTagKitchen'};
    % dirs=reshape({'/home/cjoseph/Documents/radar/matlab/data/LOSsnrBedroom/'
    %              '/home/cjoseph/Documents/radar/matlab/data/LOSsnrKitchen/'
    %              '/home/cjoseph/Documents/radar/matlab/data/LOSsnrOutside/'}, [1 3]);
    frameRate = 200; % frames per sec
    divisor = 10; % how many chunks to divide the capture into
    z=1;
    radarType = 'chipotle';
    chipSet = "X1-IPG1";
    adjustment = -40;%-7;%-40,-22; %-33;-13; % how many bins to adjust peak location
    smoothingFactor = 13; %how much to smooth during peakfinding, lower is smoother
    phasing = 0; %true=1
    ft80idx = 801;
    ft79idx = 791;
    [snrBoth80, snrBoth79, snrSolo80, snrSolo79, trueRange, radarRange] = deal([]);
    close all;
    %% Process arguments
    %if length(varargin) < 3
    %    error('Please provide the names of at least 3 capture files names')
    %end
    for i = 1:length(varargin) %build files list
        fh = char(varargin(i));
        files{i} = fh;
        %extract distance from filename
        butt  = strfind(fh, 'mm') - 1;
        dists(i) = str2double(fh(1:butt))-90; %subtract 90 to account for 9cm offset caused by how the laser ranger was mounted
    end
    radarOffset = 1500;
    f=80;
    % add noise files &c
    [solo79,solo80] = deal({});
    for fh=files
        fh = char(fh);
        solo79{end+1} = strcat('79soloO', fh(end-16:end));
        solo80{end+1} = strcat('80soloO',  fh(end-16:end));
    end
    
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
        solo79file = dir(fullfile(d, char(solo79(fidx))));
        solo80file =  dir(fullfile(d, char(solo80(fidx))));
        
        % do the md5 checks
        if ~md5check(captureFile, md5file) % signal file
            error('signal file failed md5 check')
        elseif ~md5check(solo79file, dir(fullfile(d, strcat(char(solo79(fidx)),'.md5')))) % noise file
            error('noise file failed md5 check')
        elseif ~md5check(solo80file, dir(fullfile(d, strcat(char(solo80(fidx)),'.md5')))) % tag held HI file
            error('tag hi file failed md5 check')
        end

        % Load the files
        [frameWindowSolo80 pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(solo80file.folder,solo80file.name));
        [frameWindowSolo79 pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(solo79file.folder,solo79file.name));
        [frameWindow pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(captureFile.folder,captureFile.name));
        
        % Baseband Conversion
        frameWindow_bb = toBaseband(frameWindow, chipSet, pgen, fs_hz);
        solo79FrameWindow_bb = toBaseband(frameWindowSolo79, chipSet, pgen, fs_hz);
        solo80FrameWindow_bb = toBaseband(frameWindowSolo80, chipSet, pgen, fs_hz);
        
%         frameWindow_bb=frameWindow_bb(:,1:20000/z);
%         noTagFrameWindow_bb=noTagFrameWindow_bb(:,1:20000/z);
%         hiTagFrameWindow_bb=hiTagFrameWindow_bb(:,1:20000/z);
        
        deltas = [];
        for i=1:(length(timeDeltas)-1)
            deltas = [deltas timeDeltas(i+1)-timeDeltas(i)];
        end
        fprintf("Inter-frame spacing goal is %f, achieved %f with stdev %f, min %f, max %f.", 1/frameRate, mean(deltas), std(deltas), max(deltas), min(deltas));

        %% Process the data
        [bins, autoBinsBoth79, autoBinsBoth80, autoBins80, autoBins79,...
         nosieBinsBoth79, noiseBinsBoth80, noiseBins80, noiseBins79,...
         noiseBoth80, noise80solo, noiseBoth79, noise79solo, ...
         sigBoth79, sigBoth80, sig79solo, sig80solo] = deal(zeros(divisor,1));
        offset=round((0.0557*radarOffset+145)/4); %offset induced by tag
        segSize = floor(size(frameWindow_bb,2)/divisor);
       
        
        for i=1:divisor
            %i
            seg = frameWindow_bb(:,(i-1)*segSize+1:i*segSize);
            
            %FFT of signal for each bin
            ft = fft(seg,segSize,2);
            solo79FT = fft(solo79FrameWindow_bb(:,(i-1)*segSize+1:i*segSize), segSize, 2);
            solo80FT = fft(solo80FrameWindow_bb(:,(i-1)*segSize+1:i*segSize), segSize, 2);

            %%%%%%% FTIDX and IDX %%%%%%%%%%%
            idx = round(((dists(fidx)-radarOffset)/4))+adjustment;
            idx = idx+offset;
            corrft = fft(ft(:,ft80idx)); corrft(smoothingFactor:512-smoothingFactor) = 0; lpfdBoth80 = ifft(corrft);
            corrft = fft(ft(:,ft79idx)); corrft(smoothingFactor:512-smoothingFactor) = 0; lpfdBoth79 = ifft(corrft);
            corrft = fft(solo80FT(:,ft80idx)); corrft(smoothingFactor:512-smoothingFactor) = 0; lpfd80 = ifft(corrft);
            corrft = fft(solo79FT(:,ft79idx)); corrft(smoothingFactor:512-smoothingFactor) = 0; lpfd79 = ifft(corrft);
            lpfdBoth80(1:22) = 0; lpfd80(1:22) = 0; lpfd79(1:22) = 0;
            bins(i)=idx;
            
            [sigIdx, noiseIdx] = findSCR(ft,lpfdBoth80, ft80idx);
            sigBoth80(i) = ft(sigIdx, ft80idx); autoBinsBoth80(i) = sigIdx;
            noiseBoth80(i) = ft(noiseIdx,ft80idx); noiseBinsBoth80(i) = noiseIdx;
            
            [sigIdx, noiseIdx] = findSCR(ft,lpfdBoth79, ft79idx);
            sigBoth79(i) = ft(sigIdx, ft79idx); autoBinsBoth79(i) = sigIdx;
            noiseBoth79(i) = ft(noiseIdx,ft79idx); noiseBinsBoth79(i) = noiseIdx;
            
            [sigIdx, noiseIdx] = findSCR(solo80FT,lpfd80, ft80idx);
            sig80solo(i) = solo80FT(sigIdx, ft80idx); autoBins80(i) = sigIdx;
            noise80solo(i) = solo80FT(noiseIdx, ft80idx); noiseBins80(i) = noiseIdx;
            
            [sigIdx, noiseIdx] = findSCR(solo79FT,lpfd79, ft79idx);
            sig79solo(i) = solo79FT(sigIdx, ft79idx); autoBins79(i) = sigIdx;
            noise79solo(i) = solo79FT(noiseIdx, ft79idx); noiseBins79(i) = noiseIdx;
            
            %% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % figure; 
            % plot(abs(ft(:,ft80idx)),'g'); 
            % hold on, plot(abs(lpfdBoth80),'g:'); 
            % plot(autoBinsBoth80(i),abs(ft(autoBinsBoth80(i),ft80idx)), 'g*')
            % plot(noiseBinsBoth80(i),abs(ft(noiseBinsBoth80(i),ft80idx)), 'go')
            % 
            % plot(abs(ft(:,ft79idx)),'m'); 
            % hold on, plot(abs(lpfdBoth79),'m:'); 
            % plot(autoBinsBoth79(i),abs(ft(autoBinsBoth79(i),ft79idx)), 'm*')
            % plot(noiseBinsBoth79(i),abs(ft(noiseBinsBoth79(i),ft79idx)), 'mo')
            % 
            % plot(abs(solo80FT(:,ft80idx)),'r');
            % hold on, plot(abs(lpfd80),'r:'); 
            % plot(autoBins80(i),abs(solo80FT(autoBins80(i),ft80idx)), 'r*')
            % plot(noiseBins80(i),abs(solo80FT(noiseBins80(i),ft80idx)), 'ro')
            % 
            % plot(abs(solo79FT(:,ft79idx)),'b');
            % hold on, plot(abs(lpfd79),'b:'); 
            % plot(autoBins79(i),abs(solo79FT(autoBins79(i),ft79idx)), 'b*')
            % plot(noiseBins79(i),abs(solo79FT(noiseBins79(i),ft79idx)), 'bo')
            % legend('ftBoth80','lpfBoth80','idxBoth80','noiseIdxBoth80',...
            %        'ftBoth79','lpfBoth79','idxBoth79','noiseIdxBoth79',...
            %        'ff80solo','lpf80solo','idx80solo','noiseIdx80solo',...
            %        'ff79solo','lpf79solo','idx79solo','noiseIdx79solo')
            % title(fh,'Interpreter', 'none')
            % pause(1)
        end
        figure; 
        plot(abs(ft(:,ft80idx)),'g'); 
        hold on, plot(abs(lpfdBoth80),'g:'); 
        plot(autoBinsBoth80(i),abs(ft(autoBinsBoth80(i),ft80idx)), 'g*')
        plot(noiseBinsBoth80(i),abs(ft(noiseBinsBoth80(i),ft80idx)), 'go')

        plot(abs(ft(:,ft79idx)),'m'); 
        hold on, plot(abs(lpfdBoth79),'m:'); 
        plot(autoBinsBoth79(i),abs(ft(autoBinsBoth79(i),ft79idx)), 'm*')
        plot(noiseBinsBoth79(i),abs(ft(noiseBinsBoth79(i),ft79idx)), 'mo')

        plot(abs(solo80FT(:,ft80idx)),'r');
        hold on, plot(abs(lpfd80),'r:'); 
        plot(autoBins80(i),abs(solo80FT(autoBins80(i),ft80idx)), 'r*')
        plot(noiseBins80(i),abs(solo80FT(noiseBins80(i),ft80idx)), 'ro')

        plot(abs(solo79FT(:,ft79idx)),'b');
        hold on, plot(abs(lpfd79),'b:'); 
        plot(autoBins79(i),abs(solo79FT(autoBins79(i),ft79idx)), 'b*')
        plot(noiseBins79(i),abs(solo79FT(noiseBins79(i),ft79idx)), 'bo')
        legend('ftBoth80','lpfBoth80','idxBoth80','noiseIdxBoth80',...
               'ftBoth79','lpfBoth79','idxBoth79','noiseIdxBoth79',...
               'ff80solo','lpf80solo','idx80solo','noiseIdx80solo',...
               'ff79solo','lpf79solo','idx79solo','noiseIdx79solo')
        title(fh,'Interpreter', 'none')
        pause(1)
        
        %idx, autoBins, abs(vals), abs(noise1), abs(noise2)
        %trueRange = [trueRange; idx*ones(divisor,1)];
        %radarRange = [radarRange; autoBins];
        snrBoth80 = [snrBoth80; 20*log10(abs(sigBoth80./noiseBoth80))];
        snrBoth79 = [snrBoth79; 20*log10(abs(sigBoth79./noiseBoth79))];
        snrSolo80 = [snrSolo80; 20*log10(abs(sig80solo./noise80solo))];
        snrSolo79 = [snrSolo79; 20*log10(abs(sig79solo./noise79solo))];
    end
    
    format short g
    %% output SNR stats
%     snr1=snr1(1:end-10);
%     snr2=snr2(1:end-10);
    %rErr = (abs(trueRange-radarRange)*4)/10.0; %range error in cm
    %%      approxDist min       bottom quart              med         top quart      max
    %SNRstats  =[approxDist min(snr1) quantile(snr1,0.25) median(snr1) quantile(snr1,0.75) max(snr1)]
    statsSolo79 =[min((snrSolo79)) quantile((snrSolo79),0.25) median((snrSolo79)) quantile((snrSolo79),0.75) max((snrSolo79))]
    statsBoth79 =[min((snrBoth79)) quantile((snrBoth79),0.25) median((snrBoth79)) quantile((snrBoth79),0.75) max((snrBoth79))]
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
        tmp(max(1,biggestIdx-50):biggestIdx + 100) = 0; %remove peak area from array
        [pks2,locs2,~, ~] = findpeaks(abs(tmp));
        [~,second]=max(abs(pks2));
        secondIdx = locs2(second);
        
        tmp(max(secondIdx-50,1):secondIdx + 100) = 0; %remove 2nd peak area from array
        [pks3,locs3,~, ~] = findpeaks(abs(tmp));
        [~,third]=max(abs(pks3));
        thirdIdx = locs3(third);
        
        sigIdx = biggestIdx;
        noiseIdx = secondIdx;
        if biggestIdx > secondIdx && abs(ft(biggestIdx,ftidx)/ft(secondIdx,ftidx)) < 2.2
            sigIdx = secondIdx;
            noiseIdx = thirdIdx;
        end
end
