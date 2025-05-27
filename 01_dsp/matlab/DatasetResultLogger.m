function DatasetResultLogger(localDataPath)

c=299792458;                    % Speed of light (m/s)
resolution = 0.003790984152165; % Range resolution (m)

labelFile = strcat(localDataPath(2:end), '/label.json');

label = readstruct(labelFile);

fprintf("Processing %s ...\n", localDataPath)

tag1Hz = label.tagFrequencies(1);
tag2Hz = label.tagFrequencies(2);
distance = label.tagDistance;

[peakDifference, SNRdB1, SNRdB2] = DatasetFramesProc(localDataPath, tag1Hz, tag2Hz, label.scansPerDatum);

for i = 1:length(peakDifference) % Removing obvious weird scans
    if abs(peakDifference(i) - median(peakDifference)) > 10
        peakDifference(i) = mean(peakDifference);
    end
end

t = ((peakDifference + ...
    distance/resolution) * resolution) / c; % Time of Flight (ToF) from on tag to the other (s)
radar_perm = ((c*t)/distance).^2;           % ToF converted to permittivity

VWC = label.VWC / 100;
TEROS_raw = CalibrateVWC2RAW(VWC);      % Not calibrated

S.peakDifferences = peakDifference;
S.radarPermmitivity = radar_perm;
S.adjacentTEROS = TEROS_raw;
S.SNRdB1 = SNRdB1;
S.SNRdB2 = SNRdB2;

resultPath = strcat(localDataPath(2:end), '/results.json');

writestruct(S, resultPath)

end