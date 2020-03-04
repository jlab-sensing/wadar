% freq[Hz]; re:Trc1_S11; im:Trc1_S11; re:Trc2_S41; im:Trc2_S41; 
cpx = zeros(length(f),6);

sXY = dlmread('VNA/bandPassDryIQreverse.csv',';',1,0);
f = sXY(:,1);
for i = 1:length(f) 
    cpx(i, 1) = complex(sXY(i,2),sXY(i,3));
end

sXY = dlmread('VNA/bandPass1GalIQreverse.csv',';',1,0);
for i = 1:length(f) 
    cpx(i, 2) = complex(sXY(i,2),sXY(i,3));
end

sXY = dlmread('VNA/bandPass1_25galIQreverse.csv',';',1,0);
for i = 1:length(f) 
    cpx(i, 3) = complex(sXY(i,2),sXY(i,3));
end

sXY = dlmread('VNA/bandPass1_5galIQreverse.csv',';',1,0);
for i = 1:length(f) 
    cpx(i, 4) = complex(sXY(i,2),sXY(i,3));
end

sXY = dlmread('VNA/bandPass1_75galIQreverse.csv',';',1,0);
for i = 1:length(f) 
    cpx(i, 5) = complex(sXY(i,2),sXY(i,3));
end

sXY = dlmread('VNA/bandPass2galIQreverse.csv',';',1,0);
for i = 1:length(f) 
    cpx(i, 6) = complex(sXY(i,2),sXY(i,3));
end

figure(); plot(f,rad2deg(unwrap(angle(cpx)))); title('phase'); grid on; legend
figure(); plot(f,abs(cpx)); title('mag'); grid on; legend


t = mod((-phi/(2*pi*f(10))),(1/f(10)))