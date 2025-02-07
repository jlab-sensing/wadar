% tag frequency test plotting tool

bothTagHz = ["80 - 79", '80 - 75', '80 - 70', '80 - 65', ...
    '80 - 60', '80 - 85', '80 - 90', '80 - 95', '80 - 100', '80 - 105', ...
    '80 - 110', '80 - 115', '80 - 120', '79 - 80', '75 - 80', '70 - 80', ...
    "65 - 80", "60 - 80", "85 - 80", "90 - 80", "95 - 80", "100 - 80", ...
    "105 - 80", "110 - 80", "115 - 80", "120 - 80"];

% [magnitude1 magnitude2 SNR1 SNR2 peakVariance peakDiff]

data = [29511.66 33466.03 8.65 6.12 8 14;
    29143.87 35207.64 8.24 6.47 10 16;
    29985.23 37893.09 7.97 6.25 4 16;
    31371.95 37657.49 8.73 8.62 4 13;
    30430.48 35145.53 8.74 6.33 6 13;
    31166.70 34412.67 7.86 1.90 8 13;
    30481.05 35078.69 8.61 7.62 9 17;
    30576.04 34482.47 8.16 9.95 2 20;
    30229.85 46163.31 7.89 7.11 6 18;
    29636.32 35158.18 8.73 1.79 10 20;
    29494.30 34847.33 7.51 8.02 4 19
    32753.14 38515.89 8.73 5.48 7 19;
    50101.74 43579.66 7.48 8.45 11 4;
    30346.19 36836.40 7.99 9.76 8 13;
    33221.75 35480.77 6.32 9.41 14 12;
    32343.03 34893.75 4.30 9.38 7 7;
    31315.52 35380.99 8.37 9.43 6 10;
    29653.77 36141.76 6.25 9.06 2 18;
    30248.54 35357.34 2.47 8.61 3 20;
    29640.20 35229.66 6.17 8.67 3 20;
    29426.90 35711.08 9.73 8.88 3 20;
    41812.15 36336.94 7.52 8.25 6 20;
    33363.67 36516.42 -0.26 8.26 14 17;
    33005.29 37850.17 8.12 8.91 8 18;
    32582.55 37224.00 5.56 8.98 7 17;
    43515.53 45009.79 9.44 9.60 10 7;
    ];

close all

figure(1)
bar(bothTagHz, data(:,1:2))
title("Tag Magnitude Comparison")
xlabel("Tag 1 Hz - Tag 2 Hz")
ylabel("Magnitude")

figure(2)
bar(bothTagHz, data(:,3:4))
title("Tag SNR Comparison")
xlabel("Tag 1 Hz - Tag 2 Hz")
ylabel("SNR (dB)")

figure(3)
bar(bothTagHz, data(:,5))
title("Tag Peak Variance Comparison")
xlabel("Tag 1 Hz - Tag 2 Hz")
ylabel("Peak Bin")

figure(4)
bar(bothTagHz, data(:,6))
title("Tag Peak Difference Comparison")
xlabel("Tag 1 Hz - Tag 2 Hz")
ylabel("Peak Bin")


close all

% removing combos with noise = 60, 100, 120

rowToRemove = find(bothTagHz == '80 - 60');
bothTagHz(rowToRemove) = [];
data(rowToRemove, :) = [];

rowToRemove = find(bothTagHz == '60 - 80');
bothTagHz(rowToRemove) = [];
data(rowToRemove, :) = [];

rowToRemove = find(bothTagHz == '100 - 80');
bothTagHz(rowToRemove) = [];
data(rowToRemove, :) = [];

rowToRemove = find(bothTagHz == '80 - 100');
bothTagHz(rowToRemove) = [];
data(rowToRemove, :) = [];

rowToRemove = find(bothTagHz == '120 - 80');
bothTagHz(rowToRemove) = [];
data(rowToRemove, :) = [];

rowToRemove = find(bothTagHz == '80 - 120');
bothTagHz(rowToRemove) = [];
data(rowToRemove, :) = [];

figure(1)
bar(bothTagHz, data(:,1:2))
title("Tag Magnitude Comparison")
xlabel("Tag 1 Hz - Tag 2 Hz")
ylabel("Magnitude")
% hline(prctile(data(:,1)), 90)
lowerTagMagnitude = min(transpose(data(:,1:2)))
yline(prctile(lowerTagMagnitude,[50],"all"))

peakBinShift = 50;
dataToSoilMoisture(:,1) = abs(procSoilMoisture(data(:,5),0,'farm',0.1524));
dataToSoilMoisture(:,2) = procSoilMoisture(data(:,6)+peakBinShift,0,'farm',0.1524);


figure(3)
bar(bothTagHz, dataToSoilMoisture(:,1))
title("Tag ``Soil Moisture'' Variance")
xlabel("Tag 1 Hz - Tag 2 Hz")
ylabel("Soil Moisture")

figure(4)
bar(bothTagHz, dataToSoilMoisture(:,2))
title("Tag ``Soil Moisture'' Difference with 50 Peak Bin Shift")
xlabel("Tag 1 Hz - Tag 2 Hz")
ylabel("Soil Moisture")
ylim([0 0.30])
