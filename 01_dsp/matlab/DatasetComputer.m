function DatasetComputer(localDataPath)

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
terosDataPoints = [];

for i = 1:length(folderNames)
    curFolder = strcat(localDataPath, '/', folderNames(i));
    resultFile = char(strcat(curFolder, '/results.json'));
    labelFile = char(strcat(curFolder, '/label.json'));
    resultFile = readstruct(char(resultFile(2:end)));
    labelFile = readstruct(char(labelFile(2:end)));

    radarPermittivity = [radarPermittivity; resultFile.radarPermmitivity];
    expectedVWC = [expectedVWC; labelFile.VWC * ones(1, 3)];
    terosDataPoints = [terosDataPoints; labelFile.teros12];
end

perm2VWCFile = char(strcat(localDataPath, '/perm2VWC.csv'));
perm2VWC = readmatrix(char(perm2VWCFile(2:end)));

RAW2VWCFile = char(strcat(localDataPath, '/RAW2VWC.csv'));
RAW2VWC = readmatrix(char(RAW2VWCFile(2:end)));

terosDataPoints = mean(terosDataPoints, 2);

terosDataPoints = polyval(RAW2VWC, terosDataPoints);
radarDataPoints = polyval(perm2VWC, radarPermittivity);
radarDataPoints = max(zeros(height(radarDataPoints), 1), radarDataPoints);


% Plot crispified by ChatGPT 
xi = 1:9;

figure('Color', 'w');
hold on;

scatter(xi, radarDataPoints, 70, 'r', 'filled', 'DisplayName', 'Radar');
plot(xi, radarDataPoints, '-r', 'LineWidth', 1.5, 'HandleVisibility', 'off');

scatter(xi, terosDataPoints, 70, 'g', 'filled', 'DisplayName', 'Commercial Sensor');
plot(xi, terosDataPoints, '-g', 'LineWidth', 1.5, 'HandleVisibility', 'off');

xticks(xi);
xticklabels({'1030', '1100', '1130', '1200', '1230', '1300', '1330', '1400', '1430'});
xlabel('Time', 'FontSize', 12);
ylabel('VWC (%)', 'FontSize', 12);
title('Field Test', 'FontSize', 14);
ylim([0 30]);
grid on;

set(gca, 'FontSize', 12, 'LineWidth', 1);
legend('Location', 'best', 'FontSize', 10, 'Box', 'off');
set(gcf, 'Units', 'inches', 'Position', [1 1 5 3.5]);
set(gca, 'TickDir', 'out');



end