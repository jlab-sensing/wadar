% Chipotle Radar Tester
function ChipotleRadar(radarName, testType, localDataPath)
    % 
    % This MATLAB script is designed to comply with the V&V Plan defined in 
    % https://docs.google.com/document/d/1EOrkq5g4RNGee4DSSisru6al-C2NCm0Ayb33J4UpV50/edit?usp=sharing
    % 
    % Inputs:
    % radarName: The name of the radar to be tested for file naming purposes.
    % testType: The type of test to be run. Options are:
    %   - 'SanityCheck': Perform a basic functionality check of the radar.
    %   - 'Calibration': Calibrate the radar using a known target.
    %   - 'Ranging': Measure distances to various targets.
    % localDataPath: The path to the local data directory where captured frames will be stored.
    % 
    % Outputs:
    % None. The function generates plots and saves data files in the specified local data path.
    %
    % Example usage:
    % ChipotleRadar('0_2_1', 'SanityCheck', '../data/')     % Perform a sanity check on radar 0_2_1

    
    close all; clc
    
    % Capture parameters
    frameRate = 200;
    radarType = 'Chipotle';
    
    % File name generation
    [~, hostname] = system('whoami');
    hostname(end) = '';
    fullDataPath = sprintf("%s@192.168.7.1:%s", hostname, localDataPath);
    
    if strcmp(testType, 'SanityCheck')
        captureName = strcat(testType, '_', radarName, '_C');
    
        % Check for existing files with the same name to prevent overwrite
        existingFiles = dir(localDataPath);
        for i = 1:length(existingFiles)
            if strcmp(existingFiles(i).name, strcat(captureName, '1', '.frames'))
                delete(strcat(localDataPath, existingFiles(i).name))
            end
        end

        frameCount = 2000;
        captureCount = 1;
    
        % Send Frame Logger command with appropriate parameters
        frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
            captureName, frameCount, captureCount, frameRate, radarType, fullDataPath);
        frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
            frameLoggerOptions);
        disp(frameLoggerCommand)
        [status,~] = system(frameLoggerCommand);

        fprintf("Radar go BRR. ")
        pause(frameCount/frameRate);
        fprintf("Data capture complete. Waiting for transfer.\n");

        checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
        tic
        while (length(checkFile) == 0)
            checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
            if (toc > 60)
                error('There is a data transfer issue. Verify your capture settings and scp directory. Ensure that your are already scp into the radar')
            end
        end
        fileName = checkFile(1).name;

        pause(3)
        [sanityCheckFrames, ~, ~, ~, ~] = salsaLoad(strcat(localDataPath, fileName));

        figure(1)
        % plot(sanityCheckFrames(:, 200))
        plot(sanityCheckFrames)
        title('Radar Frames')
        xlabel('Range Bin')
        ylabel('DAC')
        ylim([0 8191])

        fprintf("Sanity check complete..\n");

    elseif strcmp(testType, 'Calibration')
        fprintf("Move the calibration target out of the radar's field of view.\n");
        fprintf("Press any key to continue.\n")
        pause
    
        captureName = strcat(testType, '_', radarName, '_Step1_C');
    
        % Check for existing files with the same name to prevent overwrite
        existingFiles = dir(localDataPath);
        for i = 1:length(existingFiles)
            if strcmp(existingFiles(i).name, strcat(captureName, '1', '.frames'))
                error("File already exists. Rename the file or delete the existing file. Overwriting files is dangerous.")
                return
            end
        end
    
        frameCount = 2000;
        captureCount = 1;
    
        % Send Frame Logger command with appropriate parameters
        frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
            captureName, frameCount, captureCount, frameRate, radarType, fullDataPath);
        frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
            frameLoggerOptions);
        [status,~] = system(frameLoggerCommand);
    
        fprintf("Radar go BRR. ")
        pause(frameCount/frameRate);
        fprintf("Data capture complete. Waiting for transfer.\n");
    
        checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
        tic 
        while (length(checkFile) == 0)
            checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
            if (toc > 60)
                error('There is a data transfer issue. Verify your capture settings and scp directory. Ensure that your are already scp into the radar')
            end
        end
        fileName = checkFile(1).name;
    
        pause(3)
        [calibrationStep1Frames, ~, ~, ~, ~] = salsaLoad(strcat(localDataPath, fileName));
        calibrationStep1Frames = median(calibrationStep1Frames,2);
        
        figure(1)
        plot(calibrationStep1Frames)
        title('Beep Boop Boop the radar is calibrating')
        xlabel('Range Bin')
    
        fprintf("Move the calibration target into the radar's field of view. Take note of the distance between the radar and the target.\n");
        fprintf("Press any key to continue.\n");
        pause
        
        fprintf("Enter the distance between the radar and the calibration target in meters.\n");
        trueDistance = input('Distance: ');
    
        captureName = strcat(testType, '_', radarName, '_Step2_C');
    
        % Check for existing files with the same name to prevent overwrite
        existingFiles = dir(localDataPath);
        for i = 1:length(existingFiles)
            if strcmp(existingFiles(i).name, strcat(captureName, '1', '.frames'))
                error("File already exists. Rename the file or delete the existing file. Overwriting files is dangerous.")
                return
            end
        end
    
        % Send Frame Logger command with appropriate parameters
        frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
            captureName, frameCount, captureCount, frameRate, radarType, fullDataPath);
        frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
            frameLoggerOptions);
        [status,~] = system(frameLoggerCommand);
    
        fprintf("Radar go BRR. ")
        pause(frameCount/frameRate);
        fprintf("Data capture complete. Waiting for transfer.\n");
    
        checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
        tic
        while (length(checkFile) == 0)
            checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
            if (toc > 60)
                error('There is a data transfer issue. Verify your capture settings and scp directory. Ensure that your are already scp into the radar')
            end
        end
        fileName = checkFile(1).name;
        
        pause(3)
        [calibrationStep2Frames, ~, ~, ~, ~] = salsaLoad(strcat(localDataPath, fileName));
        calibrationStep2Frames = median(calibrationStep2Frames, 2);
    
        figure(1)
        plot(calibrationStep2Frames - calibrationStep1Frames)
        title("Radar Frames")
        xlabel('Range Bin')
    
        % Calibration
        fprintf("Indicate the range bin of the calibration target.\n");
        scanRangeBin = input('Range Bin: ');
    
        c = 299792458.0; % speed of light
        resolution = 0.003790984152165;
        scanDistance = scanRangeBin * resolution;
    
        sampleDelayToReference = (scanDistance - trueDistance) * 2 / c;
        fprintf("The sample delay to the reference should be: ");
        disp(sampleDelayToReference);
        
    elseif strcmp(testType, 'Ranging')

        fprintf("Move the ranging target out of the radar's field of view.\n");
        fprintf("Press any key to continue.\n")
        pause

        captureName = strcat(testType, '_', radarName, '_Clutter_C');

        % Check for existing files with the same name to prevent overwrite
        existingFiles = dir(localDataPath);
        for i = 1:length(existingFiles)
            if strcmp(existingFiles(i).name, strcat(captureName, '1', '.frames'))
                error("File already exists. Rename the file or delete the existing file. Overwriting files is dangerous.")
                return
            end
        end
        
        frameCount = 2000;
        captureCount = 1;

        % Send Frame Logger command with appropriate parameters
        frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
            captureName, frameCount, captureCount, frameRate, radarType, fullDataPath);
        frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
            frameLoggerOptions);
        [status,~] = system(frameLoggerCommand);
        
        fprintf("Radar go BRR. ")
        pause(frameCount/frameRate);
        fprintf("Data capture complete. Waiting for transfer.\n");

        checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
        tic
        while (length(checkFile) == 0)
            checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
            if (toc > 60)
                error('There is a data transfer issue. Verify your capture settings and scp directory. Ensure that your are already scp into the radar')
            end
        end
        fileName = checkFile(1).name;

        pause(3)
        [clutterFrames, ~, ~, ~, ~] = salsaLoad(strcat(localDataPath, fileName));

        figure(1)
        plot(clutterFrames)
        title('Radar Clutter')
        xlabel('Range Bin')

        fprintf("How many different distances are being tested?\n");
        numDistances = input('Number of distances: ');

        for i = 1:numDistances

            fprintf("Move the ranging target into position and enter the distance between the radar and the target in meters.\n");
            objectDistance = input('Distance: ');

            captureName = strcat(testType, '_', radarName, num2str(objectDistance), 'm_C');

            % Check for existing files with the same name to prevent overwrite
            existingFiles = dir(localDataPath);
            for i = 1:length(existingFiles)
                if strcmp(existingFiles(i).name, strcat(captureName, '1', '.frames'))
                    error("File already exists. Rename the file or delete the existing file. Overwriting files is dangerous.")
                    return
                end
            end

            % Send Frame Logger command with appropriate parameters
            frameLoggerOptions = sprintf('-s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s', ...
                captureName, frameCount, captureCount, frameRate, radarType, fullDataPath);
            frameLoggerCommand = sprintf('ssh root@192.168.7.2 "screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger %s " &', ...
                frameLoggerOptions);
            [status,~] = system(frameLoggerCommand);

            fprintf("Radar go BRR. ")
            pause(frameCount/frameRate);
            fprintf("Data capture complete. Waiting for transfer.\n");

            checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
            tic
            while (length(checkFile) == 0)
                checkFile = dir(fullfile(localDataPath, strcat(captureName, num2str(1), '.frames')));
                if (toc > 60)
                    error('There is a data transfer issue. Verify your capture settings and scp directory. Ensure that your are already scp into the radar')
                end
            end
            fileName = checkFile(1).name;

            pause(3)
            [objectFrames, ~, ~, ~, ~] = salsaLoad(strcat(localDataPath, fileName));

            figure(1)
            resolution = 0.003790984152165;
            range = linspace(0, 2*resolution, 512);
            plot(range, objectFrames - clutterFrames)
            title("Radar Frames")
            xlabel('Distance')
            ylabel('Magnitude')
            xline(objectDistance, 'r', 'Distance to Object')

            savefig(strcat(localDataPath, strcat(radarType, '_', num2str(objectDistance), 'm.fig')))

    end
end