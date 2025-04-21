close all; clear all; clc

% localDataPath = '/data/compact/compact';
% 
% d = strcat(pwd, localDataPath);
% files = dir(fullfile(d, '*.frames'));
% listOfCaptures = {files.name};
% 
% figure(1)
% for i = 1:length(listOfCaptures)
%     captureName = string(listOfCaptures(i));
% 
%     [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
%     % [compactCaptureFT, compactTag1FT] = ProcessFFT(compactFramesBB, frameRate, tag1Hz);
%     % [~, compactTag2FT] = ProcessFFT(compactFramesBB, frameRate, tag2Hz);
%     % [compactPeakBin1] = TagLocateCWT(compactTag1FT, false);
%     % [compactPeakBin2] = TagLocateCWT(compactTag2FT, false);
% 
%     frameRadargram = abs(framesBB ./ max(max(framesBB)));
%     frameRadargram = imresize(frameRadargram, 'OutputSize', [227 227]);
%     imgOut = cat(3, frameRadargram, frameRadargram, frameRadargram);
% 
%     dataStorage = strcat(localDataPath(2:end), '/', captureName, '.png');
% 
%     % set(gca, 'Visible', 'off');
%     % gcf = imagesc(frameRadargram);
%     % saveas(gcf, dataStorage)
%     fprintf("Saving %s...\n", dataStorage)
%     imwrite(imgOut, dataStorage)
% end
% 
% localDataPath = '/data/compact/loose';
% 
% d = strcat(pwd, localDataPath);
% files = dir(fullfile(d, '*.frames'));
% listOfCaptures = {files.name};
% 
% figure(2)
% for i = 1:length(listOfCaptures)
%     captureName = string(listOfCaptures(i));
% 
%     [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
%     % [compactCaptureFT, compactTag1FT] = ProcessFFT(compactFramesBB, frameRate, tag1Hz);
%     % [~, compactTag2FT] = ProcessFFT(compactFramesBB, frameRate, tag2Hz);
%     % [compactPeakBin1] = TagLocateCWT(compactTag1FT, false);
%     % [compactPeakBin2] = TagLocateCWT(compactTag2FT, false);
% 
%     frameRadargram = abs(framesBB ./ max(max(framesBB)));
%     frameRadargram = imresize(frameRadargram, 'OutputSize', [227 227]);
%     imgOut = cat(3, frameRadargram, frameRadargram, frameRadargram);
% 
%     dataStorage = strcat(localDataPath(2:end), '/', captureName, '.png');
% 
%     % set(gca, 'Visible', 'off');
%     % gcf = imagesc(frameRadargram);
%     % saveas(gcf, dataStorage)
%     fprintf("Saving %s...\n", dataStorage)
%     imwrite(imgOut, dataStorage)
% end

% =========================================================================

folderName = 'data/compact';
imds = imageDatastore(folderName, ...
    IncludeSubfolders=true, ...
    LabelSource="foldernames");

numImages = numel(imds.Labels);
idx = randperm(numImages,20);
I = imtile(imds,Frames=idx);
figure
imshow(I)

classNames = categories(imds.Labels);
numClasses = numel(classNames);
disp(classNames);
disp(numClasses);
[imdsTrain, imdsValidation, imdsTest] = splitEachLabel(imds,0.7,0.15,"randomized");

net = imagePretrainedNetwork(NumClasses=numClasses);
inputSize = net.Layers(1).InputSize;
disp(inputSize);

net = setLearnRateFactor(net,"conv10/Weights",10);
net = setLearnRateFactor(net,"conv10/Bias",10);

pixelRange = [-30 30];

imageAugmenter = imageDataAugmenter( ...
    RandXReflection=true, ...
    RandXTranslation=pixelRange, ...
    RandYTranslation=pixelRange);

augimdsTrain = augmentedImageDatastore(inputSize(1:2),imdsTrain, ...
    DataAugmentation=imageAugmenter);

augimdsValidation = augmentedImageDatastore(inputSize(1:2),imdsValidation);
augimdsTest = augmentedImageDatastore(inputSize(1:2),imdsTest);

options = trainingOptions("adam", ...
    InitialLearnRate=0.0001, ...
    ValidationData=augimdsValidation, ...
    ValidationFrequency=5, ...
    Plots="training-progress", ...
    Metrics="accuracy", ...
    Verbose=false);

net = trainnet(augimdsTrain,net,"crossentropy",options);

accuracy = testnet(net,augimdsTest,"accuracy")

im = imread("data/compact/compact/compact1.frames.png");

X = single(im);

scores = predict(net,X);
label = scores2label(scores,classNames);

figure
imshow(im)
title("Prediction: " + string(label))

% =========================================================================

% classificationData = table;
% 
% localDataPath = '/data/compact/compact';
% 
% d = strcat(pwd, localDataPath);
% files = dir(fullfile(d, '*.frames'));
% listOfCaptures = {files.name};
% 
% figure(1)
% hold on;
% for i = 1:length(listOfCaptures)
%     captureName = string(listOfCaptures(i));
% 
%     [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
% 
%     dataStorage = strcat(localDataPath(2:end), '/', captureName, '.png');
% 
%     fprintf("%d/%d\n", i, length(listOfCaptures))
%     data = median(abs(framesBB), 2);
%     % plot(data); hold on;
% 
%     classificationData(:, end+1) = {'Compact'; data};
% end
% 
% localDataPath = '/data/compact/loose';
% 
% d = strcat(pwd, localDataPath);
% files = dir(fullfile(d, '*.frames'));
% listOfCaptures = {files.name};
% 
% figure(2)
% for i = 1:length(listOfCaptures)
%     captureName = string(listOfCaptures(i));
% 
%     [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
% 
%     frameRadargram = abs(framesBB ./ max(max(framesBB)));
%     frameRadargram = imresize(frameRadargram, [512 512]);
% 
%     dataStorage = strcat(localDataPath(2:end), '/', captureName, '.png');
% 
%     fprintf("%d/%d\n", i, length(listOfCaptures))
%     data = median(abs(framesBB), 2);
%     % plot(data); hold on;
% 
%     classificationData(:, end+1) = {'Compact'; data};
% end