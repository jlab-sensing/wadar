% freq[Hz]; re:Trc1_S11; im:Trc1_S11; re:Trc2_S41; im:Trc2_S41; 
sXY = dlmread('VNA/bandPassDryIQ.csv',';',1,0);
f = sXY(:,1);

cpx = zeros(length(f),1);
% for i = 1:500 
%     cpx(i) = complex(sXY(i,2),sXY(i,3));
% end
% magS11 = db(abs(cpx));
% cpx = zeros(500,1);
% 
% figure(); plot(f,magS11); title('Vivaldi S11 (underground)'); ylim([-100,10]); grid on

for i = 1:length(f) 
    cpx(i) = complex(sXY(i,2),sXY(i,3));
    %cpx(i) = sXY(i,2);
end
magS22 = unwrap(angle(cpx));%db(abs(ifft(hanning(length(cpx)).*cpx)))%unwrap(cpx);

figure(); plot(f,magS22); title(' '); grid on

%figure(); plot(f,magS22); title(' '); grid on

