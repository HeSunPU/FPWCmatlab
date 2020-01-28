startMaxim;

%Laser_Power(43, 1) % at 623nm
%maxA = 13494;%1;

% more params
folder = 'C:\Users\cdelacroix\HCIFSdata\';
NpxX = 1392;
NpxY = 1040;
Xc = NpxX/2;Yc = NpxY/2; 
%RX = NpxX/2;RY = NpxY/2; 
%RX = NpxX/4;RY = NpxY/4; 
RX = 568;RY = 512; 
spaxelX = 38;%29.7;% % TBC
spaxelY = 5.95;%
camPitch = 6.45;
spaxelperlamD = 2;% *.9;%
lamDps = 1/spaxelperlamD;
lamDpp = 2*lamDps/spaxelX; % Nyquist sampled
pixperlamD = 1/lamDpp;
mask.rangeR = [5, 11];
mask.rangeAngle = 40;%42.5;
exp = 1000;%20000;%
Nimg = 1;%
%%
dark = SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
save([folder, 'dark.mat'], 'dark')
fitswrite(dark, [folder, 'dark.fits'])
%%
exp = 1000;%20000;%
Nimg = 1;%
dark = 1267;
% figure(1);
while true
    [A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
    img = A - dark;
%     img_crop = img(400:600, 400:600);%(924:end, 100:250);%
    img_crop = img(200:800, 200:800);
%    img = A;
%     imagesc(log10(abs(img)/max(max(abs(img))))),colorbar; colormap(CMRmap(100));
%     imagesc(log10(abs(img))),colorbar; colormap(CMRmap(100));
    figure(102), imagesc(log10(abs(img))),colorbar; colormap(CMRmap(100));
    hold on
    plot(568,512,'k*')
    hold off
    drawnow
%     figure, imagesc(log10(abs(img_crop))),colorbar; colormap(CMRmap(100));
%     imagesc(log10(abs(img_crop))),colorbar; colormap(CMRmap(100));
%     caxis([1, 5])
%     imagesc(abs(img_crop)),colorbar; colormap(CMRmap(100));
%     caxis([1e1, 1e5])
%    imagesc((abs(img)/max(max(abs(img))))),colorbar; colormap(CMRmap(100));
%    imagesc((abs(img))),colorbar; colormap(CMRmap(100));
%    imagesc(abs(img)),colorbar; colormap(CMRmap(100));
%    caxis([-4,0])
%    caxis([0,1000])
    
%     plotmask
%     set(gca,'YDir','normal')
%     axis equal,axis([0  2*RX    0  2*RY])
% 
%     max(max(abs(A)))
%     SAT
%     
%     path = [folder, 'flatfield_NOlens_705nm'];
%     save([path, '.mat'], 'A')
%     fitswrite(A, [path, '.fits'])
end



%%
hCam.invoke('LinkEnabled', 0);



