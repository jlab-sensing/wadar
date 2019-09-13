% freq[Hz]; re:Trc1_S11; im:Trc1_S11; re:Trc2_S41; im:Trc2_S41; 
sXY = dlmread('lgVivaldi.csv',';',1,0);
f = sXY(:,1);

cpx = zeros(500,1);
% for i = 1:500 
%     cpx(i) = complex(sXY(i,2),sXY(i,3));
% end
% magS11 = db(abs(cpx));
% cpx = zeros(500,1);
% 
% figure(); plot(f,magS11); title('Vivaldi S11 (underground)'); ylim([-100,10]); grid on

for i = 1:500 
    cpx(i) = complex(sXY(i,4),sXY(i,5));
end
magS22 = db(abs(cpx));

figure(); plot(f,magS22); title('Lg Vivaldi S11 '); ylim([-100,10]); grid on


