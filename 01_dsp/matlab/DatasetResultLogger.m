function DatasetResultLogger(localDataPath)

c=299792458;                    % Speed of light (m/s)
resolution = 0.003790984152165; % Range resolution (m)

labelFile = strcat(localDataPath(2:end), '/label.json');

label = readstruct(labelFile);

fprintf("Processing %s ...\n", localDataPath)

tag1Hz = label.tagFrequencies(1);
tag2Hz = label.tagFrequencies(2);
distance = label.tagDistance;

results = RunDualTagDataset(localDataPath, tag1Hz, tag2Hz, false);

peakDifferences = results.("Peak Difference");

t = ((peakDifferences + ...
    distance/resolution) * resolution) / c; % Time of Flight (ToF) from on tag to the other (s)
radar_perm = ((c*t)/distance).^2;           % ToF converted to permittivity

TEROS_raw = CalibrateVWC2RAW(3.28e-2);      % TODO: document

S.peakDifferences = peakDifferences;
S.radarPermmitivity = radar_perm;
S.adjacentTEROS = TEROS_raw;

resultPath = strcat(localDataPath(2:end), '/results.json');

writestruct(S, resultPath)

end