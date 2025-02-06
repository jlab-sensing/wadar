% This processing script assumes strongly structured filenames in the form of
%       'DDDDmm_ofsX-X_YYY_100s1'
% DDDDmm = distance to tag (in mm) measured w/ laser ranger
% ofsX-X = radar offset setting (in m)
% YYY    = tag oscillation frequency, e.g. 80hz, or NONE
function postProcessUnderground(varargin)
    %% IMPORTANT DEFS %%%%%%%%
    ftindices = containers.Map({0,0.5,1,10,80},[1,11,21,201,8000]);
    binLocs = containers.Map({25,38,51,64,76},[169,206,250,305,312]);
    dirs={'/home/cjoseph/Documents/radar/matlab/data/FarmMaxDepth'};
    frameRate = 200; % frames per sec
    divisor = 1; % how many chunks to divide the capture into
    radarType = 'chipotle';
    chipSet = "X1-IPG1";
    adjustment = 0; %-33;-13; % how many bins to adjust peak location
    smoothingFactor = 13; %how much to smooth during peakfinding, lower is smoother
    phasing = 0; %true=1
    subtracting=0;
    [snrOff, snrOn, trueRange, radarRange] = deal([]);
    close all;
    %% Process arguments
    for i = 1:length(varargin) %build files list
        fh = char(varargin(i));
        files{i} = fh;
        %extract distance from filename
        butt  = strfind(fh, 'in') - 1;
        dists(i) = round(str2double(fh(butt-1:butt))*2.54);
    end
    
    %extract frequency from filename
    f=80;
    % add noise files &c
    [noise,hi] = deal({});
    for fh=files
        fh = char(fh);
        noise{end+1} = strcat(fh(1:15),'Off1');
    end
    
    %% load files
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
            error('could not find filename (or md5)')
        end
        noiseFile = dir(fullfile(d, char(noise(fidx))));
        
        % do the md5 checks
        if ~md5check(captureFile, md5file) % signal file
            error('signal file %s failed md5 check', fullfile(d,fh))
        elseif ~md5check(noiseFile, dir(fullfile(d, strcat(char(noise(fidx)),'.md5')))) % noise file
            error('noise file failed md5 check')
        end

        % Load the files
        [frameWindowNoise pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(noiseFile.folder,noiseFile.name));
        [frameWindow pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(captureFile.folder,captureFile.name));
        
        % Baseband Conversion
        frameWindow_bb = toBaseband(frameWindow, chipSet, pgen, fs_hz);
        noTagFrameWindow_bb = toBaseband(frameWindowNoise, chipSet, pgen, fs_hz);
        
        deltas = [];
        for i=1:(length(timeDeltas)-1)
            deltas = [deltas timeDeltas(i+1)-timeDeltas(i)];
        end
        fprintf("Inter-frame spacing goal is %f, achieved %f with stdev %f, min %f, max %f.", 1/frameRate, mean(deltas), std(deltas), max(deltas), min(deltas));

        %% Process the data
        [bins, autoBins, vals, sig1, noise1, noise2, phases] = deal(zeros(divisor,1));
        segSize = size(frameWindow_bb,2)/divisor;
        noiseFT= fft(noTagFrameWindow_bb(:,1:20000), 20000, 2);

        for i=1:divisor
            seg = frameWindow_bb(:,1:20000);
            
            %FFT of signal for each bin
            ft = fft(seg,20000,2);
            %%%%%%% FTIDX and IDX %%%%%%%%%%%%
            ftidx = ftindices(f);
            idx = binLocs(dists(fidx))+adjustment;

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
            % p(biggest:end) = 0;
            
            sig1(i) = noiseFT(idx, 1);
            noise2idx = max([locs(biggest) locs2(second)]);
            if maxidx > penidx
                autoBins(i) = penidx;
                noise2idx = min([locs(biggest) locs2(second)]);
            end
            noise2(i) = noiseFT(noise2idx, ftidx);
            corrft = fft(ft(:,1)); corrft(smoothingFactor:512-smoothingFactor) = 0; lpfdOff = ifft(corrft);
            lpfdOff(idx-45:idx + 45) = 0; 
            [pks,locs3,~, ~] = findpeaks(abs(lpfdOff));
            [~,offmax]=max(abs(pks));
            noise1(i) = noiseFT(locs3(offmax), 1);
        end
        figure;
        plot(abs(ft(:,ftidx))); 
        hold on, plot(abs(lpfd)); 
        plot(abs(noiseFT(:,ftidx)));
        plot(idx,abs(ft(idx,ftidx)), 'g*')
        autoIdx = round(mean(autoBins));
        plot(autoIdx,abs(ft(autoIdx,ftidx)), 'b*')
        legend('fft','lpf','noise','true tag idx','auto idx')
        title(fh,'Interpreter', 'none')
        pause(1)
        %idx, autoBins, abs(vals), abs(noise1), abs(noise2)
        snrOff = [snrOff; 20*log10(abs(sig1./noise1))];
        snrOn = [snrOn; 20*log10(abs(vals./noise2))];
    end
    
    format short g
    %% output SNR stats
%     snr1=snr1(1:end-10);
%     snr2=snr2(1:end-10);
    %%      approxDist min       bottom quart              med         top quart      max
    %SNRstats  =[approxDist min(snr1) quantile(snr1,0.25) median(snr1) quantile(snr1,0.75) max(snr1)]
    %peakStats =[approxDistst min(snr2) quantile(snr2,0.25) median(snr2) quantile(snr2,0.75) max(snr2)]
    snrOn
    snrOff
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