startMaxim;

%Laser_Power(43, 1) % at 623nm
%maxA = 13494;%1;
%%
% more params
NpxX = 1392;
NpxY = 1040;
Xc = 625;%631;%NpxX/2;%600;%
Yc = 460;%476;%NpxY/2; 
RX = Xc;%NpxX/2;
RY = Yc;%NpxY/2; 
spaxelX = 29.7;%
spaxelY = 5.95;%
camPitch = 6.45;
spaxelperlamD = 1.70;%2;%
lamDps = 1/spaxelperlamD;
lamDpp = 2*lamDps/spaxelX; % Nyquist sampled
pixperlamD = 1/lamDpp;

%% Dark image - hide the beam
SXdarkCam = SXcam_Nimg(hCam, 0, 100, NpxX/2, NpxY/2, NpxX/2, NpxY/2, 0, 0, 30);
mean_std = [mean2(SXdarkCam), std2(SXdarkCam)]
max(max(abs(SXdarkCam)))
figure(); imagesc(log10(abs(SXdarkCam))),colorbar; colormap(CMRmap(100));
%save('C:\Lab\FPWC\hardware\SXdarkCam.mat', 'SXdarkCam')
%load('C:\Lab\FPWC\hardware\SXdarkCam.mat', 'SXdarkCam')
% save('C:\Users\cdelacroix\NewPrism\BB\SXdarkCam.mat', 'SXdarkCam')

%% Wide Field
% 100ms at laser 25 (425/681, 509) -> 2ms at laser 47
% superK 80ms (392/646 ,484) -> 400ms
[A,x,y,SAT]=SXcam_Nimg(hCam, SXdarkCam, 400, Xc, Yc, RX, RY, 0, 0, 30);
maxA = max(max(abs(A))), SAT % saturate at 6.4E4
figure(),imagesc(abs(A)),colorbar; colormap(CMRmap(100));
set(gca,'YDir','normal'),axis equal,axis([0  2*RX    0  2*RY])
figure(),imagesc(log10(abs(A)/maxA)),colorbar; colormap(CMRmap(100));
set(gca,'YDir','normal'),axis equal,axis([0  2*RX    0  2*RY])
plotmask;
caxis([-4,0])
%save('C:\Lab\FPWC\hardware\SXwide.mat', 'A')
% save('C:\Users\cdelacroix\NewPrism\BB\SXwide.mat', 'A')

%% max A at laser 47 + 2ms --> 1000ms
% superK 400ms --> 1000ms
%maxA = 25700*26760/4070/2*1000;
maxA = 35710*56670/7262/400*10000;


%% Close up
RX = 300;%NpxX/2;
RY = 350;%NpxY/2; 
[A,x,y,SAT]=SXcam_Nimg(hCam, SXdarkCam, 10000, Xc, Yc, RX, RY, 0, 0, 10);
max(max(abs(A))), SAT % saturate at 6.4E4
A = A/maxA;
save('C:\Lab\FPWC\hardware\SXclose.mat', 'A')

figure(),imagesc(log10(abs(A))),colorbar; colormap(CMRmap(100));
set(gca,'YDir','normal'),axis equal,axis([0  2*RX    0  2*RY])
plotmask;
caxis([-7,-5])


%% END
hCam.invoke('LinkEnabled', 0);



