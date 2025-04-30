function outputRAW = CalibrateVWC2RAW(inputVWC)

RAW = linspace(1000,4000,10000);

% for UCSC farm
VWC = (5.12018081e-10)*RAW.^3 - (0.000003854251138)*RAW.^2 + (0.009950433112)*RAW - 8.508168835941; 

[~, idxVWC] = min(abs(VWC-inputVWC));

outputRAW = RAW(idxVWC);

% figure
% plot(RAW, VWC)
% title("RAW vs. VWC")
% xlabel("RAW")
% ylabel("VWC")

end