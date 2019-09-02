function [A,x,y,SAT]=SXcam_Nimg(hCam,darkCam,T,Xc,Yc,RX,RY,dx,dy,N)
%
% Checked & Calibrated on 08/24/12 by A.Carlotti
% Rechecked by Hari Subedi 
% updates by C. Delacroix

% A = the returned image array
% x & y = the vectors of the image
% hCam = camera handle
% darkCam = dark frame
% T = Exposure time (ms)
% Xc, Yc = center coordinates
% RX, RY = radii of the imaging region (in pixels)
% dx, dy = margin
% N = Number of images

% example:  T=100;Xc=1392/2;Yc=1040/2;RX=1392/2;RY=1040/2;dx=0;dy=0;N=1;
% [A,x,y,SAT]=SXcam_Nimg(hCam,100,0,0,686,510,20,20,1);
% imagesc(log10(abs(A)/max(max(abs(A))))), colorbar;



hCam.invoke('AutoDownload',1);

hCam.invoke('StartX',0);
hCam.invoke('StartY',0);

hCam.invoke('NumX',1392);
hCam.invoke('NumY',1040);

A=0;

SAT=0;

for i=1:N

    %disp(['# ' num2str(i)])
    hCam.invoke('Expose', 1e-3*T, 1, 0);

    while(~hCam.invoke('ImageReady'))
        pause(0.01);
    end
    Temp = double(hCam.invoke('ImageArray')) - darkCam';
    Temp = Temp(1+Xc-(RX+dx):Xc+RX+dx,1+Yc-(RY+dy):Yc+RY+dy);
    
    if max(max(Temp)) > 64000;%45000
        SAT=1;
    end    
    
    A=A+Temp;

end

A=A'/N;

%x=(X0-RX+0.5:X0+RX-0.5)*pixtolambda;
%y=(Y0-RY+0.5:Y0+RY-0.5)*pixtolambda;
x=(Xc-RX+0.5:Xc+RX-0.5);
y=(Yc-RY+0.5:Yc+RY-0.5);

hCam.invoke('DisableAutoShutdown', 1);