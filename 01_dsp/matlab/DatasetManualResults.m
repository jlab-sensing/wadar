function DatasetManualResults(localDataPath)

d = strcat(pwd, localDataPath);
listing = dir(d);
tbl = struct2table(listing);
tbl.date = datetime(tbl.datenum,ConvertFrom="datenum");
tbl = removevars(tbl,"datenum");
folders = tbl(tbl.isdir,:);
namedFolders = folders(~matches(folders.name,[".",".."]),:);

folderNames = namedFolders.name;

for i = 1:length(folderNames)
    curFolder = strcat(localDataPath, '/', folderNames(i));
    DatasetManualResultLogger(char(curFolder));
end

end