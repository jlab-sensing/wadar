function Radargram2Image(localDataPath)
d = strcat(pwd, localDataPath);
files = dir(fullfile(d, '*.frames'));
listOfCaptures = {files.name};

for i = 1:length(listOfCaptures)
    captureName = string(listOfCaptures(i));
    fprintf("Processing %s [", captureName)

    [frameTot, framesBB, frameRate] = ProcessFrames(localDataPath, captureName);
    
    framesPerImage = 2000;
    imagesPerScan = width(framesBB) / framesPerImage;
    for j = 1:imagesPerScan
        frameRadargram = framesBB(:, j*framesPerImage-(framesPerImage-1):j*framesPerImage);
        frameRadargram = PreprocessRadargram(frameRadargram);
        % frameRadargram = abs(frameRadargram) ./ max(max(abs(frameRadargram)));
        frameRadargram = abs(frameRadargram) ./ 8e2;
        frameRadargram = imresize(frameRadargram, 'OutputSize', [227 227]);
        imgOut = cat(3, frameRadargram, frameRadargram, frameRadargram);
    
        dataStorage = strcat(localDataPath(2:end), '/', captureName, '-', num2str(j), '.png');
    
        fprintf("||")
        imwrite(imgOut, dataStorage)
    end
    fprintf("]\n")
end

imageFiles = dir(fullfile(d, '*.png'));
listOfImages = {imageFiles.name};
listOfImagePaths = strcat(pwd, localDataPath, '/', listOfImages);
out = imtile(listOfImagePaths);

figure
imshow(out)

end