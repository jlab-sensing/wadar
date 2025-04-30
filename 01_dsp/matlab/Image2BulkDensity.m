function net = Image2BulkDensity(folderName)

imds = imageDatastore(folderName, ...
    IncludeSubfolders=true, ...
    LabelSource="foldernames");

numImages = numel(imds.Labels);
idx = randperm(numImages,20);
I = imtile(imds,Frames=idx);
figure
imshow(I)

[imdsTrain, imdsValidation, imdsTest] = splitEachLabel(imds,0.7,0.15,"randomized");

classNames = categories(imds.Labels);
numClasses = numel(classNames);
net = imagePretrainedNetwork(NumClasses=numClasses);

net = setLearnRateFactor(net,"conv10/Weights",10);
net = setLearnRateFactor(net,"conv10/Bias",10);

% pixelRange = [-30 30];
% 
% imageAugmenter = imageDataAugmenter( ...
%     RandXReflection=true, ...
%     RandXTranslation=pixelRange, ...
%     RandYTranslation=pixelRange);

% augimdsTrain = augmentedImageDatastore(inputSize(1:2),imdsTrain, ...
%     DataAugmentation=imageAugmenter);
% 
% augimdsValidation = augmentedImageDatastore(inputSize(1:2),imdsValidation);
% augimdsTest = augmentedImageDatastore(inputSize(1:2),imdsTest);

options = trainingOptions("adam", ...
    InitialLearnRate=0.0001, ...
    ValidationData=imdsValidation, ...
    ValidationFrequency=5, ...
    Plots="training-progress", ...
    Metrics="accuracy", ...
    Verbose=false);

net = trainnet(imdsTrain, net,"crossentropy",options);

accuracy = testnet(net, imdsTest,"accuracy");
disp(accuracy)

end