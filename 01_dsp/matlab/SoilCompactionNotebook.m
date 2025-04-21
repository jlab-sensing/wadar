close all; clear all; clc

classificationData = table;

localDataPath = '/data/compact/compact';

d = strcat(pwd, localDataPath);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

figure(1)
hold on;
for i = 1:length(listOfCaptures)
    captureName = string(listOfCaptures(i));

    [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);

    dataStorage = strcat(localDataPath(2:end), '/', captureName, '.png');

    fprintf("%d/%d - Compact\n", i, length(listOfCaptures))
    data = median(abs(framesBB), 2);
    plot(data); hold on;

    classificationData(:, end+1) = {'Compact'; data};
end

localDataPath = '/data/compact/loose';

d = strcat(pwd, localDataPath);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

figure(2)
for i = 1:length(listOfCaptures)
    captureName = string(listOfCaptures(i));

    [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);

    frameRadargram = abs(framesBB ./ max(max(framesBB)));
    frameRadargram = imresize(frameRadargram, [512 512]);

    dataStorage = strcat(localDataPath(2:end), '/', captureName, '.png');

    fprintf("%d/%d- Loose \n", i, length(listOfCaptures))
    data = median(abs(framesBB), 2);
    plot(data); hold on;

    classificationData(:, end+1) = {'Compact'; data};
end