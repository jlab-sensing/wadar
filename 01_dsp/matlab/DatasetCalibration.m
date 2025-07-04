function DatasetCalibration(localDataPath)

d = strcat(pwd, localDataPath);
listing = dir(d);
tbl = struct2table(listing);
tbl.date = datetime(tbl.datenum,ConvertFrom="datenum");
tbl = removevars(tbl,"datenum");
folders = tbl(tbl.isdir,:);
namedFolders = folders(~matches(folders.name,[".",".."]),:);

folderNames = namedFolders.name;

radarPermittivity = [];
expectedVWC = [];
groundTruth = [];
terosDataPoints = [];

for i = 1:length(folderNames)
    curFolder = strcat(localDataPath, '/', folderNames(i));
    resultFile = char(strcat(curFolder, '/results.json'));
    labelFile = char(strcat(curFolder, '/label.json'));
    resultFile = readstruct(char(resultFile(2:end)));
    labelFile = readstruct(char(labelFile(2:end)));

    radarPermittivity = [radarPermittivity; resultFile.radarPermmitivity];
    expectedVWC = [expectedVWC; labelFile.VWC * ones(1, 3)];
    groundTruth(end+1) = labelFile.VWC;
    terosDataPoints = [terosDataPoints; labelFile.teros12];
end

perm2VWC = polyfit(radarPermittivity, expectedVWC, height(radarPermittivity) - 2);

figure
scatter(radarPermittivity, expectedVWC)
hold on
radarPermittivityPoints = linspace(min(min(radarPermittivity)) - 10, max(max(radarPermittivity)) + 10, 100);
plot(radarPermittivityPoints, polyval(perm2VWC, radarPermittivityPoints))
legend(["True Raw", "Model"])
xlabel("Permittivity")
ylabel("RAW")
title("Permittivity to RAW Calibration")

radarDataPoints = polyval(perm2VWC, radarPermittivity);
writematrix(perm2VWC, fullfile(fullfile(pwd, localDataPath), 'perm2VWC.csv'));

% This is TEROS' medium specific calibration process.
RAW2VWC = polyfit(terosDataPoints, expectedVWC, height(terosDataPoints) - 2);
terosDataPoints = polyval(RAW2VWC, terosDataPoints);
writematrix(RAW2VWC, fullfile(fullfile(pwd, localDataPath), 'RAW2VWC.csv'));

% This is the generic formula that can be used if there are not enough
% data points to perform a calibration.
% terosDataPoints = 3.879e-4 * terosDataPoints - 0.6956;

figure
x = linspace(0, 30, 100);
y = x;
plot(x, y); hold on

for i = 1:length(groundTruth) % chaos...
    if i == 1
        b = boxchart(ones(1,3) * groundTruth(i), radarDataPoints(i,:), ...
                     'BoxWidth', 0.5, 'DisplayName', "Radar");
        t = boxchart(ones(1,3) * groundTruth(i), terosDataPoints(i,:), ...
                     'BoxWidth', 0.5, 'DisplayName', "Commercial Sensor");
        firstBox = b;
        secondBox = t;

    else
        b = boxchart(ones(1,3) * groundTruth(i), radarDataPoints(i,:), ...
                     'BoxWidth', 0.5);
        t = boxchart(ones(1,3) * groundTruth(i), terosDataPoints(i,:), ...
             'BoxWidth', 0.5);
    end
    b.BoxFaceColor = [0 0.6 0];  % Set green
    t.BoxFaceColor = [0.6 0 0];  % Set green
end

xlabel("Ground truth soil moisture level (%)")
ylabel("Sensor soil moisture level (%)")

legend([firstBox secondBox], 'Location', 'northwest');

%% Results table

radarMean  = mean(radarDataPoints,   2);      
terosMean  = mean(terosDataPoints, 2);   

resultsTbl = table(groundTruth', radarMean, terosMean, ...
        'VariableNames', {'GroundTruthVWC', 'RadarMean', 'TerosMean'} );

disp(resultsTbl)  

writetable(resultsTbl, fullfile(fullfile(pwd, localDataPath), 'dataset_results.csv'));

end