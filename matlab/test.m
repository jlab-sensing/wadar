
%%Create the radar object
radar = radarWrapper;

%%Open a connection to the radar module
radar.Open('BeagleBone!SPI device: 0!DEFAULT!NVA6201');
% radar.Open('BeagleBone!SPI device: 0!DEFAULT!NVA6100');

%Calibrate
tic
% result = radar.ExcecuteAction('MeasureAll');
toc
%Get a register value
IterationsDefaultValue = radar.Item('Iterations')

offsetdistance = radar.Item('OffsetDistanceFromReference')

% gain = radar.Item('Gain')

%Set a register value
radar.TryUpdateChip('Iterations','10');

%Check that it set the value
IterationsSetValue = radar.Item('Iterations')

tic;
t1=toc;
for i = 1:50
    newFrame1 = radar.GetFrameRaw;
end
t2=toc;
FPS = 50/(t2-t1)

newFrame2 = radar.GetFrameRaw;

plot(newFrame1);hold on
plot(newFrame2);hold off


radar.Close;