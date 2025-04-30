close all; clear all; clc

folderNames = ["tin1", "tin2", "tin3", "tin4", "tin5", "tin6"];
localDataPath = '/data/compact-2/';

% folderNames = ["10297", "10754"];
% localDataPath = '/data/compact/';

for j = 1:length(folderNames)
    d = strcat(pwd, localDataPath, folderNames(j));
    localPathToCapture = strcat(localDataPath, folderNames(j));
    files = dir(fullfile(d, '*.frames'));
    listOfCaptures = {files.name};
    
    fprintf("Processing %s...\n", folderNames(j))
    hold on;
    for i = 1:length(listOfCaptures)
        captureName = string(listOfCaptures(i));
    
        [frameTot, framesBB, frameRate] = ProcessFrames(localPathToCapture, captureName);
    
        dataStorage = strcat(localPathToCapture(2:end), '/', captureName, '.png');
        
        figure(1)
        subplot(length(folderNames),1,j);
        data = median(abs(framesBB), 2);

        [peakMag, peakIndex] = findpeaks(data, 'MinPeakDistance', 20, 'MinPeakHeight', max(data) * 0.5);
        startMin = peakIndex(1);
        startMax = peakIndex(2);
        [soilMin, soilStartingPoint] = min(data(startMin:startMax));
        soilStartingPoint = soilStartingPoint + startMin;
        plot(data); hold on;
        scatter(soilStartingPoint, soilMin)

        figure(2)
        subplot(1,length(folderNames),j);
        data = abs(framesBB);
        start = 150;
        data = data(soilStartingPoint:soilStartingPoint+100, :);
        % data = imresize(data, [227 227]);
        imagesc(data);
        title(folderNames(j))
        break
    end
end
