%% SPIE figures
mywhite = [1 1 1];
mygreen = [0 .5 0];
myred = [1 .2 0];
myblue = [0 .2 1];
mygrey = [0.4,0.4,0.4];

fsz = 32;

%% perfect shaped pupil and SPF
P=load('/Users/cdelacroix/CODE/Matlab/FPWC/opticalModel/SPs/ripple3_256x256_ideal_undersized.txt')';
figure();imagesc(abs(P))
colormap(gray(256));colorbar; axis equal; axis tight

N=2^12;
P1 = zeros(N);
P1(1+(N-size(P,1))/2:(N+size(P,1))/2,1+(N-size(P,2))/2:(N+size(P,2))/2) = P;
F = fftshift(fft2(fftshift(P1)));
F1 = F(size(F,1)/2-200:size(F,1)/2+200,size(F,2)/2-175:size(F,2)/2+175);
xs = linspace(-size(F1,1)/2,size(F1,1)/2,5)/13;
ys = linspace(-size(F1,2)/2,size(F1,2)/2,5)/13;
figure();imagesc(xs,ys,log10(abs(F1)/max(max(abs(F1)))))%(1+(N-size(P,1))/2:(N+size(P,1))/2,1+(N-size(P,2))/2:(N+size(P,2))/2)))
colormap(CMRmap(100));axis equal; axis tight; set(gca,'YDir','normal')
colorbar; 
%caxis([-7,0])
caxis([-8,-3])
xlabel('f\lambda/D')
ylabel('f\lambda/D')
set(gca,'FontSize',fsz);set(gca,'color','none');
%% PSF with/without WFC
tmp = load('/Users/cdelacroix/Desktop/RESULTS/MONOwithWFC.mat');
F = tmp.I';
%tmp = load('/Users/cdelacroix/Desktop/RESULTS/MONOnoWFC.mat');
%F = tmp.I0';
xs = linspace(-size(F,1)/2,size(F,1)/2,5)/3.3;
ys = linspace(-size(F,2)/2,size(F,2)/2,5)/3.3;
figure();imagesc(xs,ys,log10(abs(F)))
colorbar; colormap(CMRmap(100));axis equal; axis tight;set(gca,'YDir','normal')
caxis([-8,-3])
xlabel('f\lambda/D')
ylabel('f\lambda/D')
set(gca,'FontSize',fsz);set(gca,'color','none');
%% WFC whole data
lwz = 4;
tmp = load('/Users/cdelacroix/Desktop/RESULTS/MONOWFCdata.mat');
tmp = tmp.dataEFCBatch;
itr = 19;
meas = tmp.measuredContrastAverage(1:itr);
esti = tmp.estimatedContrastAverage(1:itr);
estinc = tmp.estimatedIncoherentAverage(1:itr);
contrast0 = tmp.contrast0;
figure()
semilogy(0:itr, [contrast0; meas], '-o', 'color', mygreen,'linewidth',lwz), hold on, grid on
semilogy(0:itr-1, esti, '-s', 'color', myblue,'linewidth',lwz)
semilogy(0:itr-1, estinc, '-^', 'color', myred,'linewidth',lwz)
ylim([10^(-8), 10^(-4)]);yticks([10^(-8) 10^(-7) 10^(-6) 10^(-5) 10^(-4)])
legend('measured', 'estimated', 'incoherent');
set(gca,'FontSize',fsz);set(gca,'color','none');
set(gca,'linewidth',lwz);
%% SX params
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

%% SX MONO wide
coef = 15;
coef2 = 31;
tmp = load('/Users/cdelacroix/Desktop/RESULTS/MONOSXwide.mat');
A = tmp.A;
xs = linspace(-size(A,1)/2,size(A,1)/2,10)/coef;
ys = linspace(-size(A,2)/2,size(A,2)/2,10)/coef2;
figure(),imagesc(xs,ys,log10(abs(rot90(A,2))))
colorbar; colormap(CMRmap(100));axis equal; axis tight; set(gca,'YDir','normal')
%Xc =0, Yc= 0;
plotmask_2;
caxis([-4,0])
xlabel('f\lambda/D')
ylabel('f\lambda/D')
set(gca,'FontSize',fsz);set(gca,'color','none');
%axis([RX*.45  RX*(2-.40)    RY/2  2*RY])
axis([-RX*.55/coef  RX*0.55/coef    -RY/2/coef2  RY/coef2])

%% SX MONO close
coef = 20;
coef2 = 20;
tmp = load('/Users/cdelacroix/Desktop/RESULTS/MONOSXclose.mat');
A = tmp.A;
xs = linspace(-size(A,1)/2,size(A,1)/2,10)/coef;
ys = linspace(-size(A,2)/2,size(A,2)/2,10)/coef2;
figure(),imagesc(xs,ys,log10(abs(rot90(A,2))))
colorbar; colormap(CMRmap(100));axis equal; axis tight; set(gca,'YDir','normal')
%Xc =0, Yc= 0;
%plotmask_2;
caxis([-7,-5])
xlabel('f\lambda/D')
ylabel('f\lambda/D')
set(gca,'FontSize',fsz);set(gca,'color','none');
%axis([RX*.45  RX*(2-.40)    RY/2  2*RY])
%axis([-RX*.55/coef  RX*0.55/coef    -RY/2/coef2  RY/coef2])

%% SX BB close
RX = 300;%NpxX/2;
RY = 350;%NpxY/2; 
tmp = load('/Users/cdelacroix/Desktop/RESULTS/BBSXclose.mat');
A = tmp.A/10;
xs = linspace(-size(A,1)/2,size(A,1)/2,10)/coef;
ys = linspace(-size(A,2)/2,size(A,2)/2,10)/coef2;
figure(),imagesc(log10(abs(rot90(A,2))))
colorbar; colormap(CMRmap(100));axis equal; axis tight; set(gca,'YDir','normal')
%Xc =0, Yc= 0;
%plotmask_2;
plotmask
caxis([-7,-5])
xlabel('f\lambda/D')
ylabel('f\lambda/D')
set(gca,'FontSize',fsz);set(gca,'color','none');
%axis([RX*.45  RX*(2-.40)    RY/2  2*RY])
%axis([-RX*.55/coef  RX*0.55/coef    -RY/2/coef2  RY/coef2])


%% prism curves
x1 = [0.59	0.61	0.63	0.65	0.67	0.69	0.71	0.73];
y1 = [59.57674419	56.20131783	53.23162791	50.60767442	48.29054264	46.24341085	44.41395349	42.85116279];
x2 = [0.59	0.61	0.63	0.65	0.67	0.69	0.71	0.73];
y2 = [49.08837209	49.70573643	50.08620155	50.33906977	50.52294574	50.70116279	50.90511628	51.11162791];
figure(),plot(x1,y1,'color',myblue,'linewidth',lwz), hold on, grid on
plot(x2,y2,'color',mygreen,'linewidth',lwz)
xlabel('Wavelength (µm)')
ylabel('Spectral Resolution')
legend('1 element (COTS)', '2 elements: FS + ZnS');
set(gca,'FontSize',fsz);set(gca,'color','none');
axis([.59 .73 40 60])



%% close all
close all

