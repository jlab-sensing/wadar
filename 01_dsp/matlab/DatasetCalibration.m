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

% perm2VWC = bestPolyfit(radarPermittivity, expectedVWC, 5); 
perm2VWC = polyfit(radarPermittivity, expectedVWC, 2);

figure(1);
set(gcf, "Color", "w"); 

tlo = findall(gcf, 'Type', 'tiledlayout');
if isempty(tlo)
    tlo = tiledlayout(3,1); 
end
nexttile; 

hold on;

scatter(radarPermittivity(:), expectedVWC(:), 70, 'b', 'filled', 'DisplayName', 'True Permittivity');
radarPermittivityPoints = linspace(min(min((radarPermittivity))) - 10, max(max(radarPermittivity)) + 10, 100);
plot(radarPermittivityPoints, polyval(perm2VWC, radarPermittivityPoints), '-k', 'LineWidth', 1.5, 'DisplayName', 'Model');
xlabel('Permittivity', 'FontSize', 12);
ylabel('VWC', 'FontSize', 12);
title('Radar Permittivity to VWC Calibration', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12, 'LineWidth', 1);
legend('Location', 'best', 'FontSize', 10, 'Box', 'off');
set(gcf, 'Units', 'inches', 'Position', [1 1 5 3.5]);
set(gca, 'TickDir', 'out');


radarDataPoints = polyval(perm2VWC, radarPermittivity);
writematrix(perm2VWC, fullfile(fullfile(pwd, localDataPath), 'perm2VWC.csv'));

% This is TEROS' medium specific calibration process.
RAW2VWC = polyfit(terosDataPoints, expectedVWC, 2);
calibratedTerosDataPoints = polyval(RAW2VWC, terosDataPoints);
writematrix(RAW2VWC, fullfile(fullfile(pwd, localDataPath), 'RAW2VWC.csv'));

% This is the generic formula that can be used if there are not enough
% data points to perform a calibration.
% terosDataPoints = 3.879e-4 * terosDataPoints - 0.6956;

figure(3);
set(gcf, "Color", "w"); 

tlo = findall(gcf, 'Type', 'tiledlayout');
if isempty(tlo)
    tlo = tiledlayout(3,1); 
end
nexttile; 

hold on;

scatter(terosDataPoints(:), expectedVWC(:), 70, 'b', 'filled', 'DisplayName', 'True Permittivity');
terosRAWpoints = linspace(min(min((terosDataPoints))) - 10, max(max(terosDataPoints)) + 10, 100);
plot(terosRAWpoints, polyval(RAW2VWC, terosRAWpoints), '-k', 'LineWidth', 1.5, 'DisplayName', 'Model');
xlabel('RAW', 'FontSize', 12);
ylabel('VWC', 'FontSize', 12);
title('TEROS12 RAW to VWC Calibration', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12, 'LineWidth', 1);
legend('Location', 'best', 'FontSize', 10, 'Box', 'off');
set(gcf, 'Units', 'inches', 'Position', [1 1 5 3.5]);
set(gca, 'TickDir', 'out');


figure(2); 
set(gcf, "Color", "w"); 

tlo = findall(gcf, 'Type', 'tiledlayout');
if isempty(tlo)
    tlo = tiledlayout(3,1); 
end
nexttile; 
x = linspace(0, 30, 100);
y = x;
plot(x, y, '-k', 'LineWidth', 1.5, 'DisplayName', "Ground Truth"); hold on

for i = 1:length(groundTruth)
    if i == 1
        b = boxchart(ones(1,3) * groundTruth(i), radarDataPoints(i,:), ...
                     'BoxWidth', 0.5, 'DisplayName', "Radar");
        t = boxchart(ones(1,3) * groundTruth(i), calibratedTerosDataPoints(i,:), ...
                     'BoxWidth', 0.5, 'DisplayName', "Commercial Sensor");
    else
        b = boxchart(ones(1,3) * groundTruth(i), radarDataPoints(i,:), ...
                     'BoxWidth', 0.5, 'HandleVisibility', 'off');
        t = boxchart(ones(1,3) * groundTruth(i), calibratedTerosDataPoints(i,:), ...
                     'BoxWidth', 0.5, 'HandleVisibility', 'off');
    end
    b.BoxFaceColor = [0 0.6 0];
    t.BoxFaceColor = [0.6 0 0];
end

xlabel("Sensor VWC (%)", 'FontSize', 12);
ylabel("Ground turht VWC (%)", 'FontSize', 12);

legend('Location', 'northwest', 'FontSize', 10, 'Box', 'off');

set(gca, 'FontSize', 12, 'LineWidth', 1);
set(gcf, 'Units', 'inches', 'Position', [1 1 5 3.5]);
set(gca, 'TickDir', 'out');
grid on;


%% Results table

radarMean  = mean(radarDataPoints,   2);      
terosMean  = mean(calibratedTerosDataPoints, 2);   

resultsTbl = table(groundTruth', radarMean, terosMean, ...
        'VariableNames', {'GroundTruthVWC', 'RadarMean', 'TerosMean'} );

disp(resultsTbl)  

writetable(resultsTbl, fullfile(fullfile(pwd, localDataPath), 'dataset_results.csv'));

%% Raw Data



end