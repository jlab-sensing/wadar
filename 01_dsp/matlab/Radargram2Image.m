% localDataPath = '/data/compact';
% tag1Hz = 64;
% tag2Hz = 105;
% 
% d = strcat(pwd, localDataPath);
% files = dir(fullfile(d, '*.frames'));
% listOfCaptures = {files.name};
% 
% captureName = 'compact2.frames';
% 
% figure(1)
% hold on;
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
%     frameRadargram = imresize(frameRadargram, [512 512]);
% 
%     dataStorage = strcat(localDataPath(2:end), '/', captureName, '.png');
%     imwrite(frameRadargram, dataStorage)
%     imshow(dataStorage)
% end
% legend

imds = imageDatastore('data/compact','IncludeSubfolders', true, 'LabelSource', 'foldernames');
classNames = categories(imds.Labels);
[imdsTrain, imdsValidation, imdsTest] = splitEachLabel(imds, 0.7, 0.15, 0.15, "randomized");

options = trainingOptions("sgdm", ...
    MaxEpochs=4, ...
    ValidationData=imdsValidation, ...
    ValidationFrequency=30, ...
    Plots="training-progress", ...
    Metrics="accuracy", ...
    Verbose=false);
net = trainnet(imdsTrain,net_1,"crossentropy",options);
accuracy = testnet(net,imdsValidation,"accuracy")

scores = minibatchpredict(net,imdsValidation);
YValidation = scores2label(scores,classNames);

numValidationObservations = numel(imdsValidation.Files);
idx = randi(numValidationObservations,9,1);

figure
tiledlayout("flow")
for i = 1:9
    nexttile
    img = readimage(imdsValidation,idx(i));
    imshow(img)
    title("Predicted Class: " + string(YValidation(idx(i))))
end