% This processing script assumes strongly structured filenames in the form of
%       'DDDDmm_ofsX-X_YYY_100s1'
% DDDDmm = distance to tag (in mm) measured w/ laser ranger
% ofsX-X = radar offset setting (in m)
% YYY    = tag oscillation frequency, e.g. 80hz, or NONE
function postProcessNLOS(varargin)
    %% IMPORTANT DEFS %%%%%%%%
    ftindices = containers.Map({0,0.5,1,10,80},[1,11,21,201,1601]);
    binLocs = struct('o',containers.Map({1,1.5,2.5,3.5,4.5},[276,419,403,424,387]),...
                     'k',containers.Map({1,1.5,2.5,3.5,4.5},[303,206,410,377,450]),...
                     'c',containers.Map({1,1.5,2.5,3.5,4.5},[287,377,385,441,386]));
    dirs=reshape({'/home/cjoseph/Documents/radar/matlab/data/NLOSoutside/'
                  '/home/cjoseph/Documents/radar/matlab/data/NLOSkitchenWall/'
                  '/home/cjoseph/Documents/radar/matlab/data/NLOScloset/'}, [1 3]);
    frameRate = 200; % frames per sec
    divisor = 5; % how many chunks to divide the capture into
    radarType = 'chipotle';
    chipSet = "X1-IPG1";
    adjustment = 0; %-33;-13; % how many bins to adjust peak location
    smoothingFactor = 13; %how much to smooth during peakfinding, lower is smoother
    phasing = 0; %true=1
    subtracting=0;
    [snr1, snr2, trueRange, radarRange] = deal([]);
    close all;
    %% Process arguments
    if length(varargin) < 3
        error('Please provide the names of at least 3 capture files names')
    end
    if varargin{end} == 1
        subtracting=1;
        varargin(end)=[];
    end
    for i = 1:length(varargin) %build files list
        fh = char(varargin(i));
        files{i} = fh;
        %extract distance from filename
        butt  = strfind(fh, 'mm') - 1;
        approxDist = str2double(fh(1:butt))/1000;
        dists(i) = approxDist*1000-90; %subtract 90 to account for 9cm offset caused by how the laser ranger was mounted
    end
    %infer the radar offset setting from the distance
    if dists(1) < 2000
        radarOffset = 0;
    elseif (dists(1) < 3000 && dists(1) > 2000)
        radarOffset = 1000;
    elseif (dists(1) < 4000 && dists(1) > 3000)
        radarOffset = 2000;
    elseif dists(1) > 4000
        radarOffset = 3000;
    else
        error('something went wrong parsing the radar offset')
    end
    %extract frequency from filename
    fh = char(files{1});
    butt = strfind(lower(fh),'hz');
    if isempty(butt)
        f = 0;
    else
        if strcmp(fh(butt-3:butt-1),'_10')
            f = 10;
        elseif strcmp(fh(butt-3:butt-1),'0_1')
            f = 1;
        elseif strcmp(fh(butt-3:butt-1),'_80')
            f = 80;
        elseif strcmp(fh(butt-3:butt-1),'0-5')
            f = 0.5;
        else
            error('something went wrong parsing the frequency')
        end
    end
    % add noise files &c
    [noise,hi] = deal({});
    for fh=files
        fh = char(fh);
        noise{end+1} = strcat(fh(1:strfind(fh, 'mm')+9),'NONE',fh(end-6:end));
        hi{end+1} = strcat(fh(1:strfind(fh, 'mm')+9),'HI', fh(end-6:end));
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
        hiFile =  dir(fullfile(d, char(hi(fidx))));
        
        % do the md5 checks
        if ~md5check(captureFile, md5file) % signal file
            error('signal file %s failed md5 check', fullfile(d,fh))
        elseif ~md5check(noiseFile, dir(fullfile(d, strcat(char(noise(fidx)),'.md5')))) % noise file
            error('noise file failed md5 check')
        elseif ~md5check(hiFile, dir(fullfile(d, strcat(char(hi(fidx)),'.md5')))) % tag held HI file
            error('tag hi file failed md5 check')
        end

        % Load the files
        [frameWindowHi pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(hiFile.folder,hiFile.name));
        [frameWindowNoise pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(noiseFile.folder,noiseFile.name));
        [frameWindow pgen fs_hz chipSet timeDeltas] = salsaLoad(fullfile(captureFile.folder,captureFile.name));
        
        % Baseband Conversion
        frameWindow_bb = toBaseband(frameWindow, chipSet, pgen, fs_hz);
        noTagFrameWindow_bb = toBaseband(frameWindowNoise, chipSet, pgen, fs_hz);
        hiTagFrameWindow_bb = toBaseband(frameWindowHi, chipSet, pgen, fs_hz);
        
        deltas = [];
        for i=1:(length(timeDeltas)-1)
            deltas = [deltas timeDeltas(i+1)-timeDeltas(i)];
        end
        fprintf("Inter-frame spacing goal is %f, achieved %f with stdev %f, min %f, max %f.", 1/frameRate, mean(deltas), std(deltas), max(deltas), min(deltas));

        %% Process the data
        [bins, autoBins, vals, noise1, noise2, phases] = deal(zeros(divisor,1));
        offset=round((0.0557*radarOffset+145)/4); %offset induced by tag
        segSize = size(frameWindow_bb,2)/divisor;
        noiseFTs={};
        for i=1:divisor
            noiseFTs{i} = fft(noTagFrameWindow_bb(:,(i-1)*segSize+1:i*segSize), segSize, 2);
        end
        %convert to 3d matrix
        noiseFTs = cell2mat(arrayfun(@(x)permute(x{:},[1 3 2]),noiseFTs,'UniformOutput',false)); noiseFTs = permute(noiseFTs,[2 1 3]);
        noiseFT = squeeze(mean(noiseFTs));

        for i=1:divisor
            seg = frameWindow_bb(:,(i-1)*segSize+1:i*segSize);
            
            %FFT of signal for each bin
            ft = fft(seg,segSize,2);
            %%%%%%% FTIDX and IDX %%%%%%%%%%%%
            ftidx = ftindices(f);
            idx = binLocs.(type)(approxDist)+adjustment;
            maximum = 0; argmax = 0; 
            if  f > 0   
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

                 noise1(i) = noiseFT(idx-offset, ftidx);
                 noise2idx = max([locs(biggest) locs2(second)]);
                 if maxidx > penidx
                     autoBins(i) = penidx;
                     noise2idx = min([locs(biggest) locs2(second)]);
                 end
                 noise2(i) = noiseFT(noise2idx, ftidx);
                if phasing
                    phase = argmax; square_wave = sign(sin(2*pi*f*(0:segSize-1)/frameRate + phase + 1e-8)); square_wave_rep = repmat(square_wave, size(seg,1), 1); correlatedFrame = sum(seg.*square_wave_rep,2); plot(abs(correlatedFrame))
                    phases(i) = rad2deg(angle(correlatedFrame(bins(i))));
                end
            else
                meanHi = mean(hiTagFrameWindow_bb,2);
                meanFrame = mean(seg,2);
                if subtracting
                    meanFrame = meanFrame-meanHi; 
                end
                meanNoise = mean(noTagFrameWindow_bb,2);
                vals(i) = meanFrame(idx);
                noise1(i) = meanNoise(idx);
                zerodMean = meanFrame; zerodMean(idx-30:idx+30) = 0;
                noise2(i) = max(zerodMean);
            end
%             figure; 
%             if f > 0
%                 plot(abs(ft(:,ftidx))); 
%                 hold on, plot(abs(lpfd)); 
%             else
%                 plot(abs(meanFrame));hold on;
%                 plot(abs(meanNoise));
%             end

            if phasing
                plot(abs(correlatedFrame));
                %bins, phases
            end
        end
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
        figure;
        if f > 0
            plot(abs(ft(:,ftidx))); 
            hold on, plot(abs(lpfd)); 
            plot(abs(noiseFT(:,ftidx)));
            plot(idx,abs(ft(idx,ftidx)), 'g*')
            autoIdx = round(mean(autoBins));
            plot(autoIdx,abs(ft(autoIdx,ftidx)), 'b*')
            legend('fft','lpf','noise','true tag idx','auto idx')
            title(fh,'Interpreter', 'none')
            pause(1)
        else
            plot(abs(meanFrame));hold on;
            plot(abs(meanNoise));
            plot(idx,abs(meanFrame(idx)), 'g*')
            [~, midx] = max(zerodMean);
            plot(midx,abs(meanFrame(midx)), 'b*')
            legend('mean signal','mean noise','tag idx','2nd most prominent peak')
            title(fh,'Interpreter', 'none')
            pause(1)
        end
        %idx, autoBins, abs(vals), abs(noise1), abs(noise2)
        snr1 = [snr1; 20*log10(abs(vals-noise1)./abs(noise1))];
        snr2 = [snr2; 20*log10(abs(vals./noise2))];
    end
    
    format short g
    %% output SNR stats
%     snr1=snr1(1:end-10);
%     snr2=snr2(1:end-10);
    %%      approxDist min       bottom quart              med         top quart      max
    %SNRstats  =[approxDist min(snr1) quantile(snr1,0.25) median(snr1) quantile(snr1,0.75) max(snr1)]
    peakStats =[approxDist min(snr2) quantile(snr2,0.25) median(snr2) quantile(snr2,0.75) max(snr2)]
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