function terraTest(localDataPath, captureName, groundName)

tagHz = 80;
frameRate = 200;  

[frameTot, pgen, fs_hz, chipSet, ~] = salsaLoad(fullfile(localDataPath, captureName));
[noiseFloor, ~, ~, ~, ~] = salsaLoad(fullfile(localDataPath, groundName));

figure(1)
subplot(2,1,1)
plot(frameTot);
xlabel('Range Bin');
ylabel('');
title('Raw Radar Scan');
subplot(2,1,2)
plot(mean(frameTot - noiseFloor,2))

figure(2)
subplot(2,1,1)
imagesc(frameTot)
subplot(2,1,2)
imagesc([0 2000], [0 200], mean(frameTot - noiseFloor,2))

end