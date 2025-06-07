function Frames2CSV(localDataPath)
% Converts frames to csv files so that I can process them in Python instead

d = strcat(pwd, localDataPath);
listing = dir(d);
tbl = struct2table(listing);
tbl.date = datetime(tbl.datenum,ConvertFrom="datenum");
tbl = removevars(tbl,"datenum");
folders = tbl(tbl.isdir,:);
namedFolders = folders(~matches(folders.name,[".",".."]),:);

folderNames = namedFolders.name;
scansPerDatum = 1;

for i = 1:length(folderNames)
    curFolder = strcat(localDataPath, '/', folderNames(i));

    d = strcat(pwd, curFolder);
    files = dir(string(fullfile(d, '*.frames')));
    listOfCaptures = {files.name};

    for j = 1:scansPerDatum:length(listOfCaptures)
        for i = j:j+scansPerDatum-1
            if (i - j) == 0
                captureName = string(listOfCaptures(i));
                [frameTot, framesBB, frameRate] = ProcessFrames(curFolder, captureName);
                widthFrames = width(frameTot);
            else
                captureName = string(listOfCaptures(i));
                [frameTot(:, end+1:end+widthFrames), framesBB(:, end+1:end+widthFrames), ~] = ProcessFrames(curFolder, captureName);
            end
        end
        
        fileName = fullfile(strcat(pwd, curFolder), captureName);
        fileName = erase(fileName, ".frames");
        fileName = strcat(fileName, "_frameTot.csv");
        writematrix(frameTot, fileName)
    end


end

% fileName = strcat(localDataPath(2:end), '.csv');
% 
% captureName = string(listOfCaptures(i));
% 
% [frameTot, framesBB, frameRate] = ProcessFrames(localPathToCapture, captureName);
% 
% dataStorage = strcat(localPathToCapture(2:end), '/', captureName, '.png');


end