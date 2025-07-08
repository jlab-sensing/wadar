% This file assumes that DatasetLabeler and Dataset[]Results have been used
% already. If you got this data from the shared drive it should work
% without any prior function calls.

close all; clear all; clc

folderNames = {"/data/sandy-loam-ucsc-farm-dual-tag-dataset/yellow", ...
    "/data/silty-clay-loam-soquel-dual-tag-dataset/yellow", ...
    "/data/loam-pie-ranch-dual-tag-dataset/yellow"};

% folderNames = {"/data/sandy-loam-ucsc-farm-dual-tag-dataset/red", ...
%     "/data/silty-clay-loam-soquel-dual-tag-dataset/red", ...
%     "/data/loam-pie-ranch-dual-tag-dataset/red"};

for i = 1:length(folderNames)
    DatasetCalibration(folderNames{i});
end

allResults = table();
for i = 1:length(folderNames)
    thisFolder = folderNames{i};
    resultsPath = fullfile(pwd, thisFolder, 'dataset_results.csv');
    t = readtable(resultsPath);
    allResults = [allResults; t];
end

disp(allResults)