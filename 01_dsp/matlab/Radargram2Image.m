clear; close all; clc

c = 2.998e8;
f = 3e9;
lambda = c / f;

r = 1:512;
r = r ./ 512 .* 2;
pathLoss = (4*pi*r/lambda).^2;
pathLoss = pathLoss ./ max(pathLoss);
pathLoss = 1 - pathLoss;

localDataPath = '/data/compact/compact';
d = strcat(pwd, localDataPath);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

figure(1)
title("Compact Soil")
for i = 1:length(listOfCaptures)
% for i = 1:3
    captureName = string(listOfCaptures(i));
    fprintf("Processing %s...\n", captureName)

    [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
    
    radargram_envelope = abs(framesBB);
    radargram_envelope = movmedian(radargram_envelope, 10, 2);
    radargram_envelope = radargram_envelope .* transpose(pathLoss);
    radargram_envelope = radargram_envelope(128:end, :);
    normalizer = max(mean(radargram_envelope, 2));
    radargram_envelope = radargram_envelope / normalizer;
    radargram_envelope = resize(radargram_envelope, [512 512 1]);
    % radargram = radargram ./ (4900 - 3800);
    
    figure(1)
    % nexttile; imagesc(radargram_envelope)
    nexttile; plot(abs(transpose(framesBB)))

    radargram_IQ = imag(framesBB);
    radargram_IQ = radargram_IQ(128:end, :);

    % figure(2)
    % nexttile; imagesc(radargram_IQ)

    % plot(median(imag(framesBB),2))

    dataStorage = strcat(localDataPath(2:end), '/', captureName, '.png');
    fprintf("%s saved...\n", dataStorage)
    imwrite(radargram_envelope, dataStorage)
    title("Compact Soil")
end



localDataPath = '/data/compact/loose';
d = strcat(pwd, localDataPath);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

for i = 1:length(listOfCaptures)
% for i = 1:3
    captureName = string(listOfCaptures(i));
    fprintf("Processing %s...\n", captureName)

    [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
    
    radargram_envelope = abs(framesBB);
    radargram_envelope = movmedian(radargram_envelope, 10, 2);
    radargram_envelope = radargram_envelope .* transpose(pathLoss);
    radargram_envelope = radargram_envelope(128:end, :);
    normalizer = max(mean(radargram_envelope, 2));
    radargram_envelope = radargram_envelope / normalizer;
    radargram_envelope = resize(radargram_envelope, [512 512 1]);
    % radargram = radargram ./ (4900 - 3800);
    
    figure(1)
    % nexttile; imagesc(radargram_envelope)
    nexttile; plot(abs(transpose(framesBB)))

    radargram_IQ = imag(framesBB);
    radargram_IQ = radargram_IQ(128:end, :);

    % figure(2)
    % nexttile; imagesc(radargram_IQ)

    % plot(median(imag(framesBB),2))

    dataStorage = strcat(localDataPath(2:end), '/', captureName, '.png');
    fprintf("%s saved...\n", dataStorage)
    imwrite(radargram_envelope, dataStorage)
    title("Loose Soil")
end