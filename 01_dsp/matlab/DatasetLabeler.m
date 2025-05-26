function DatasetLabeler(localDataPath, scansPerDatum, VWC, teros12, tagFrequencies, tagDistance)
    d = strcat(pwd, '/', localDataPath);
    listing = dir(d);
    tbl = struct2table(listing);
    tbl.date = datetime(tbl.datenum,ConvertFrom="datenum");
    tbl = removevars(tbl,"datenum");
    folders = tbl(~tbl.isdir,:);
    framesFiles = folders(endsWith(folders.name, '.frames'), :);
    
    labelPath = strcat(localDataPath(2:end), '/label.json');
    
    S.names = framesFiles.name;
    S.scansPerDatum = scansPerDatum;
    S.VWC = VWC;
    S.teros12 = teros12;
    S.tagFrequencies = tagFrequencies;
    S.tagDistance = tagDistance;
    
    writestruct(S, labelPath)

    S = readstruct(labelPath);
    disp(S)
end