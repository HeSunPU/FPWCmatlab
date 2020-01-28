%% Section #1 - initialize the filter wheels of the superK
fw1=serial('COM12','BaudRate',115200,'Terminator','CR');
fopen(fw1)
fprintf(fw1,'pos=12');
% fclose(fw1)

%% Section #2 - initialize the starlight express camera
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
% RX = 512;RY = 512; 
RX = 568;RY = 512;
% RX = 610;RY = 512;
spaxelX = 38;%29.7;% % TBC
spaxelY = 5.95;%
camPitch = 6.45;
spaxelperlamD = 2;% *.9;%
lamDps = 1/spaxelperlamD;
lamDpp = 2*lamDps/spaxelX; % Nyquist sampled
pixperlamD = 1/lamDpp;
mask.rangeR = [5, 11];
mask.rangeAngle = 40;%42.5;
exp = 1000;%20000;% [ms]
Nimg = 100;%100;%

%% Section #3 - take darkFrame image, please turn off the laser first

[A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
darkFrame = A;
% process the dark image and find dead pixels
deadSpot = darkFrame > median(darkFrame(:)) + 200;
sum(sum(deadSpot))

%% Section #4 - take flat field images, please turn on the laser
target.channel = [3, 4, 6, 7, 8, 9, 10];

img = zeros(2*RY, 2*RX, 7);
for k = 1 : 7
    disp(num2str(k))
%     fprintf(fw1,['pos=', num2str(k)]);
    fprintf(fw1,['pos=', num2str(target.channel(k))]);
    pause(2)
    [A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
    img(:, :, k) = squeeze(A);
    figure(101), imagesc((abs(img(:, :, k)))),colorbar; colormap(CMRmap(100));
    drawnow
end

%% Section #5 - process and save the flat field images
% create a folder first to save the flat fields in 'C:/Lab/FPWCmatlab/IFS/'
fitsname = [600,620,640,650,670,694,720];
for k = 1 : 7
    temp =  img(:, :, k) - median(darkFrame(:));
    temp(deadSpot) = 0;
    temp = temp(:, 57:1136-56);
%     fitswrite(temp, ['flat', num2str(k), '.fits'])
    fitswrite(temp, ['flat', num2str(fitsname(1,k)), '.fits'])
end


%% Section #6 - auto-align the lenslet array of the IFS using the linear stages

%% 6.1 - connect the linear stage of the lenslet array
fpos    = get(0,'DefaultFigurePosition'); % figure default position
fpos(3) = 650; % figure window size;Width
fpos(4) = 450;
fpos
f = figure('Position', fpos,...
'Menu','None',...
'Name','APT GUI');
% Create ActiveX Controller
h = actxcontrol('MGMOTOR.MGMotorCtrl.1',[20 20 600 400 ], f);
h.StartCtrl
SN = 27003602;
set(h,'HWSerialNum', SN);

%% move the stage
pos = 6.3;
h.SetAbsMovePos(0,pos);
h.MoveAbsolute(0,1==0);
    
%% 6.2 - move the stage to find the peak PSF intensity
pos_list = 5:0.1:7;
peak = zeros(length(pos_list), 1);
spectral_image = zeros(1024, 1136, length(pos_list));
for k = 1 : length(pos_list)
    h.SetAbsMovePos(0,pos_list(k));
    h.MoveAbsolute(0,1==0);
    pause(10)
    [A,x,y,SAT]=SXcam_Nimg(hCam, camera.darkFrame, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
    A(deadSpot) = 0;
    figure(202), imagesc(A(400:600, 400:600)), colorbar;
    title(num2str(pos_list(k)));
    peak(k) = max(max(A));
    spectral_image(:, :, k) = A;
    drawnow
    pause(1)
end
% please select the positions which have the largest peak
figure, plot(pos_list, peak);
