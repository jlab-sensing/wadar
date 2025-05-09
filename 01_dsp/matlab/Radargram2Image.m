function Radargram2Image(localDataPath)

d = strcat(pwd, localDataPath);
listing = dir(d);
tbl = struct2table(listing);
tbl.date = datetime(tbl.datenum,ConvertFrom="datenum");
tbl = removevars(tbl,"datenum");
folders = tbl(tbl.isdir,:);
namedFolders = folders(~matches(folders.name,[".",".."]),:);

for idxFolder = 1:height(namedFolders)
    subfolderName = string(namedFolders.name(idxFolder));
    subd = strcat(d, "/", subfolderName);
    localSubd = strcat(localDataPath, "/", subfolderName);

    files = dir(fullfile(subd, '*.frames'));
    listOfCaptures = {files.name};

    for i = 1:length(listOfCaptures)
        captureName = string(listOfCaptures(i));
        fprintf("Processing %s [", captureName)
        [frameTot, framesBB, frameRate] = ProcessFrames(localSubd, captureName);

        frameRadargram = abs(framesBB);
        % frameRadargram = PreprocessRadargram(frameRadargram);
        % frameRadargram = abs(frameRadargram) ./ max(max(abs(frameRadargram)));
        % frameRadargram = abs(frameRadargram) ./ 8e2;
        % frameRadargram = imresize(frameRadargram, 'OutputSize', [227 227]);

        % Linear scaling. Tentative.
        for j = 1:height(frameRadargram)
            frameRadargram(j, :) = frameRadargram(j, :) * j / 512;
        end

        frameRadargram = imresize(frameRadargram, 'OutputSize', [227 227]);

        frameRadargram = mat2gray(frameRadargram);

        % nColors = 256;
        % yellowToBlue = [linspace(0, 1, nColors)', ...    % Red goes from 1 to 0
        %                 linspace(0, 1, nColors)', ...    % Green goes from 1 to 0
        %                 linspace(1, 0, nColors)'];       % Blue goes from 0 to 1
        % indexedImg = gray2ind(frameRadargram, nColors);
        % 
        % imgOut = ind2rgb(indexedImg, yellowToBlue);

        imgOut = cat(3, frameRadargram, frameRadargram, frameRadargram);

        localSubd = char(localSubd);
        dataStorage = strcat(localSubd(2:end), "/", captureName, ".png");

        fprintf("||")
        imwrite(imgOut, dataStorage)
        fprintf("]\n")
    end

    imageFiles = dir(fullfile(subd, '*.png'));
    listOfImages = {imageFiles.name};
    listOfImagePaths = strcat(pwd, localSubd, '/', listOfImages);
    out = imtile(listOfImagePaths);

    figure
    imshow(out)
    title(subfolderName)
end

end