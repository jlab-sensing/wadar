% Based off the XeThru XEP_X4_plot_frame.m example
%
% Opens an X4 radar and captures frames for a set duration.
% Can save the data and/or play back old data in .mat format
%
% Required args:
%   profile: string, profile name to load
%       To create a profile, save your path variables to a .mat file, e.g.
%       save('colleen.mat', 'moduleConnectorPath', 'includePath', 'libPath')
%       where 'libPath' and such are previously defined path strings
%
% Optional positional args passed as pairs of'name', 'value'
% 
% Usage examples:
%   saveData_X4('colleen', 'savePath', '/foo/name.m')
%   ls    Loads profile 'colleen' and saves data in the folder '/foo'       
%   saveData_X4('brad', 'readFile', '/bar/run2.mat')
%       Loads profile 'brad' and loads previously saved data from run2.mat.
%       If 'savePath' isn't specified, no output data will be saved.
%

function saveData_X4(profile, varargin)
    maxTime = 10;
    whos; %TODO: delete this?
    close all;
    fprintf('Running profile %s...\n', profile)
    
    load(sprintf('%s.mat',profile), 'moduleConnectorPath', 'includePath', 'libPath', 'COM', 'ALTCOM');

    % Parse varargs
    options = struct('readFile','','savePath','');
    % read the acceptable names
    optionNames = fieldnames(options);

    % count arguments
    nArgs = length(varargin);
    if round(nArgs/2)~=nArgs/2
        error('keyword adrgument missing value')
    end

    for pair = reshape(varargin,2,[]) % pair is {propName;propValue}
        inpName = pair{1};
        if any(strcmp(inpName,optionNames))
            options.(inpName) = pair{2};
        else
            optionNames
            error('%s is not a recognized parameter name, valid options listed above',inpName)
            
        end
    end
    
    % Input parameters
    FPS = 400; %500;
    dataType = 'bb';

    % Chip settings
    PPS = 10;
    DACmin = 949;
    DACmax = 1100;
    Iterations = 16;
    FrameStart = 0.2; % meters.
    OffsetDistance = FrameStart-0.86/8;
    FrameStop = 9.4; % meters.
    % default values for FrameStart and FrameStop are 0.2 m and 9.4 m.

    if options.('readFile')
        % load a saved file
        fprintf('Loading saved data from %s...\n',options.('readFile'))
        load(options.('readFile'));
        i = length(frameTot);
    else
        % add paths defined in profile
        addpath(moduleConnectorPath);
        addpath(includePath);
        addpath(libPath);
    
        clc
        close all
        %clear
    
        % Load the library
        Lib = ModuleConnector.Library;
        %Lib.libfunctions
        
        % % Initialize radar.
        open = 0; 
        while ~open
            try 
                [radar, app] = startRadar(COM,FPS,dataType);
                radar.init();
                radar.close();
            catch ME
                if endsWith(ME.message,'verify COM-port and check the logs')
                    try 
                        [radar, app] = startRadar(ALTCOM,FPS,dataType);
                        radar.init();
                    catch
                    end
                else
                    try
                        [radar, app] = startRadar(COM,FPS,dataType);
                        radar.init();
                    catch
                    end
                end
            end
            try 
               radar.radarInstance.x4driver_set_pulsesperstep(PPS);
               open = 1;
            catch
            end
        end
      

        % Configure X4 chip.
        radar.radarInstance.x4driver_set_dac_min(DACmin);
        radar.radarInstance.x4driver_set_dac_max(DACmax);
        radar.radarInstance.x4driver_set_iterations(Iterations);

        % Configure frame area
        radar.radarInstance.x4driver_set_frame_area(FrameStart,FrameStop);
        % Read back actual set frame area
        [frameStart, frameStop] = radar.radarInstance.x4driver_get_frame_area();

        % Start streaming and subscribe to message_data_float.
        % Start the radar three times so that we get valid data
        radar.start();

        tstart = tic;
        tspent = toc(tstart);

        i = 0;
        % frameTot = zeros(181, FPS*maxTime);
        while (tspent < maxTime)
            % Peek message data float
            numPackets = radar.bufferSize();
            if numPackets > 0
                i = i+1;
                % Get 181x1 frame (uses read_message_data_float)
                [frame, ctr] = radar.GetFrameNormalized();

                % timestamp for each collected frame
                if i == 1
                    timeTot = zeros(1, FPS*maxTime); 
                end
                timeTot(i) = toc(tstart);

                if i == 1
                    numBins = length(frame);
                    numBins = numBins/2;
                    binLength = (frameStop-frameStart)/(numBins-1);
                    rangeVec = (0:numBins-1)*binLength + frameStart;
                end

                frame = frame(1:end/2) + 1i*frame(end/2 + 1:end);

                if i == 1
                   frameTot = zeros(size(frame,1), FPS*maxTime);
                end
                frameTot(:,i) = frame;
            end

            tspent = toc(tstart);
        end

        % Stop streaming.
        radar.stop();

        frameTot = frameTot(:,1:i);
        timeTot = timeTot(:,1:i);
          % Output short summary report.
        framesRead = i;
        totFramesFromChip = ctr;

        FPS_est = framesRead/tspent;

        framesDropped = ctr-i;

        disp(['Read ' num2str(framesRead) ' frames. A total of ' num2str(totFramesFromChip) ' were sent from chip. Frames dropped: ' num2str(framesDropped)]);
        disp(['Estimated FPS: ' num2str(FPS_est) ', should be: ' num2str(FPS)]);

        radar.close();
        clear radar frame
    end
    FT = fft(frameTot,i,2);
    a = (db(abs(FT)));
    Fs =  23.328*10^9; %as per XeThru X4 user manual
    sample_resolution = ((0.5 * 299792458) / Fs)*8;    
    % show image of the fourier transform of IQ time domain samples 
    % For matrices, the fft operation is applied to each column. 
    figure(1); im = imagesc(a);%imagesc(a(:,1:i/2));
    %TODO: label axes better
    xticklabels({'50','100','150','200','250','300','350'})
    title(sprintf('Radar response across all freqs'))
    ylabel('Range bin (5.14cm increments)')
    xlabel('Doppler freq (Hz)');
    [~,maxRangeIndex] = max(a(:,1)); 
    figure(2); plot(a(maxRangeIndex,:));
    title(sprintf('???'))
    figure(3); imagesc(db(abs(fft(diff(frameTot,[],2),i,2))))
    title(sprintf('Radar response across all freqs, w/ highlight(?)'))
    ylabel('Range bin (5.13cm increments)')
    xlabel('Doppler freq (Hz)');    
    xticklabels({'50','100','150','200','250','300','350'})
    
      if options.('savePath')
        % load a saved file
        if FPS_est > 50
            fprintf('Saving output to %s...\n',options.('savePath'))
            save(sprintf(strcat(options.('savePath'),'expData%s.mat'),datestr(now,30)),'maxTime','frameTot','timeTot')
        end
      end

    %% plot
    %plot only bins 11-20 (like figures 1 and 3, but not in db and no imsc call)
    figure(5); plot(([1:size(frameTot,2)]/size(frameTot,2))*(size(frameTot,2)/maxTime),a(11:20,:)') 
    title(sprintf('Radar response FFT'))
    ylabel('Magnitude (dB)')
    xlabel('Frequency (Hz)');
    f = 100; %frequency of interest in Hz
    fig  = figure(6); 
    ax = axes('Parent',fig,'position',[0.13 0.39  0.77 0.54]);
    %tag23 = a(:,f*maxTime + 1)';
    %load('tagOn.mat','tag23');
    plt = plot(sample_resolution*[0:size(a)-1]-0.18,a(:,f*maxTime + 1)');
    %plt = plot(a(:,f*maxTime + 1)');
    hold on
    %plot(resolution*[1:size(a)],tag23)
    title(sprintf('Radar response for f = %f',f))
    ylabel('Magnitude (dB)')
    xlabel('Range (m)');
    ylim([-90 10]);
    b = uicontrol('Parent',fig,'Style','slider','Position',[81,55,419,23],...
              'value',f, 'min',0, 'max',FPS,'SliderStep',[0.00025 0.10]);
    bgcolor = fig.Color;
    bl1 = uicontrol('Parent',fig,'Style','text','Position',[50,55,23,23],...
                'String','0','BackgroundColor',bgcolor);
    bl2 = uicontrol('Parent',fig,'Style','text','Position',[500,55,30,23],...
                'String','400','BackgroundColor',bgcolor);
    bl3 = uicontrol('Parent',fig,'Style','text','Position',[200,22,150,23],...
                'String','Doppler frequency','BackgroundColor',bgcolor);
    function slider(es,ed) 
        plt.YData =  a(:,round(es.Value*maxTime) + 1)';
        title(sprintf('Radar response for f = %f',es.Value))
    end
    b.Callback = @slider;
    while ishandle(fig)
        pause(0.01)
        drawnow
    end 
    
end