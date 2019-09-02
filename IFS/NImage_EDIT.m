function [A,x,y,SAT]=NImage_EDIT(T,X0,Y0,RX,RY,N)
%
% Checked & Calibrated on 08/24/12 by A.Carlotti
% Rechecked by Hari Subedi 
% [A,x,y]=NImage(T,X0,Y0,R,N)
% A = the returned image array
% x & y = the vectors of the image
% T = Exposure time (ms)
% X0 = X coordinate (in lambda/D) % X0 = 0. Added by Hari Subedi
% Y0 = Y coordinate (in lambda/D) % Y0 = 0. Added by Hari Subedi
% RX = 'radius' X of the imaging region (in pixels)
% RY = 'radius' Y of the imaging region (in pixels)
% N = Number of images


load('controls_camera_data.mat')
%pixtolambda = (6.45/(0.635*1000/(4.4*300/516)));  
%Xc=778;
%Yc=530;

pixtolambda = controls_camera_data.pixtolambda;
Xc=controls_camera_data.Xc;
Yc=controls_camera_data.Yc;


X0=round(X0/pixtolambda);
Y0=round(Y0/pixtolambda);

Lx=1392;
Ly=1040;

startMaxim;
hCam.invoke('AutoDownload',1);

hCam.invoke('StartX',Y0-(RX+20)+Xc);
hCam.invoke('StartY',X0-(RY+20)+Yc);

hCam.invoke('NumX',2*(RX+20));
hCam.invoke('NumY',2*(RY+20));

A=0;

SAT=0;

for i=1:N

    %disp(['# ' num2str(i)])
    hCam.invoke('Expose', 1e-3*T, 1, 0);

    while(~hCam.invoke('ImageReady'))
        pause(0.01);
    end
    Temp=double(hCam.invoke('ImageArray'));
    Temp=Temp(21:2*RX+20,21:2*RY+20);
    
    if max(max(Temp)) > 45000
        SAT=1;
    end    
    
    A=A+Temp;

end

A=A'/N;

x=(X0-RX+0.5:X0+RX-0.5)*pixtolambda;
y=(Y0-RY+0.5:Y0+RY-0.5)*pixtolambda;

hCam.invoke('DisableAutoShutdown', 1);