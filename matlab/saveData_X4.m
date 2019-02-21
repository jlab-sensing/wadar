% Based off the XeThru XEP_X4_plot_frame.m example
%
% Opens an X4 radar and captures frames for a set duration.
% Can save the data and/or play back old data in .mat format
%
% Required args:
%   profile: string, profile name to load
%       To create a profile, save your path variables to a .mat file, e.g.
%       save('colleen.mat', 'moduleConnectorPath', 'includePath', 'libPath', )
%       where 'libPath' and such are previously defined path strings
%
% Optional positional args passed as pairs of'name', 'value'
% 
% Usage examples:
%   saveData_X4('colleen', 'savePath', '/foo/name.m')
%       Loads profile 'colleen' and saves data in the folder '/foo'       
%   saveData_X4('brad', 'readFile', '/bar/run2.mat', 'os', 'linux')
%       Loads profile 'brad' and loads previously saved data from run2.mat.
%       If 'savePath' isn't specified, no output data will be saved.
%

function saveData_X4(profile, varargin)
    whos;
    fprintf('Running profile %s...', profile)
    
    load(sprintf('%s.mat',profile));
    
    try
        radar.close();
    catch

    end

    % add paths defined in profile
    addpath(moduleConnectorPath);
    addpath(includePath);
    addpath(libPath);
    
    clc
    close all
    %clear
    

    % Load the library
    Lib = ModuleConnector.Library;
    Lib.libfunctions

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
    COM = '/dev/tty/su';
    FPS = 400; %500;
    dataType = 'bb';

    % Chip settings
    PPS = 10;
    DACmin = 949;
    DACmax = 1100;
    Iterations = 16;
    FrameStart = 0.2; % meters.
    FrameStop = 9.4; % meters.
    % default values for FrameStart and FrameStop are 0.2 m and 9.4 m.


    % Create BasicRadarClassX4 object
    radar = BasicRadarClassX4(COM,FPS,dataType); %WTF is this

    if options.('readFile')
        % load a saved file
        fprintf('Loading saved data from %s...\n',options.('readFile'))
        load(options.('readFile'))
        i = length(frameTot);
    else
        % otherwise, get frames from radar
        radar.open();

        % Use X4M300 interface to attempt to set sensor mode XEP (manual).
        app = radar.mc.get_x4m300();

        app.set_sensor_mode('stop');
        try
            app.set_sensor_mode('XEP');
        catch
            % Unable to set sensor mode. Assume only running XEP FW.
        end

        % Initialize radar.
        radar.init();

        % Configure X4 chip.
        radar.radarInstance.x4driver_set_pulsesperstep(PPS);
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
        maxTime = 10;
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
    
    figure(1); imagesc(db(abs(fft(frameTot,i,2))))
    a = (db(abs(fft(frameTot,i,2))));
    [~,maxRangeIndex] = max(a(:,1)); 
    figure(2); plot(a(maxRangeIndex,:));
    figure(3); imagesc(db(abs(fft(diff(frameTot,[],2),i,2))))

  
      if options.('savePath')
        % load a saved file
        fprintf('Saving output to %s...\n',options.('savePath'))
        save(sprintf(options.('savePath')+'expData%s.mat',datestr(now,30)),'frameTot','timeTot')
      end

    %% plot
    figure(5); plot(a(11:20,:)')
    figure(6); plot(a(:,2561)')
    % figure(6); plot(a(:,25600)')
    % figure(6); plot(a(:,255994)')
end