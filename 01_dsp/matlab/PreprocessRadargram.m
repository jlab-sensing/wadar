function processedRadargram = PreprocessRadargram(rawRadargram)

POST_SOIL_START = 100;

data = median(abs(rawRadargram), 2);

[~, peakIndex] = findpeaks(data, 'MinPeakDistance', 20, 'MinPeakHeight', max(data) * 0.5);
startMin = peakIndex(1);
startMax = peakIndex(2);
[~, soilStartingPoint] = min(data(startMin:startMax));
soilStartingPoint = soilStartingPoint + startMin;

processedRadargram = rawRadargram(soilStartingPoint:soilStartingPoint+POST_SOIL_START, :);

end