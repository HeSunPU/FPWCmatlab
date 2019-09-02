fw1=serial('COM12','BaudRate',115200,'Terminator','CR');
fopen(fw1)
fprintf(fw1,'pos=12');
% fclose(fw1)

%%
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
exp = 10000;%20000;%
Nimg = 1;%100;%
%%
dark = SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
save([folder, 'dark.mat'], 'dark')
fitswrite(dark, [folder, 'dark.fits'])

%%
img = zeros(2*RY, 2*RX, 4);
[A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
img(:, :, 1) = A;

[A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
img(:, :, 2) = A;


[A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
img(:, :, 2) = A;


[A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
img(:, :, 4) = A;
%% take flat field images
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

%% find dead pixels
deadSpot = darkFrame > median(darkFrame(:)) + 200;
sum(sum(deadSpot))
%% save the flat field images
fitsname = [600,620,640,650,670,694,720];
for k = 1 : 7
    temp =  img(:, :, k) - median(darkFrame(:));
    temp(deadSpot) = 0;
    temp = temp(:, 57:1136-56);
%     fitswrite(temp, ['flat', num2str(k), '.fits'])
    fitswrite(temp, ['flat', num2str(fitsname(1,k)), '.fits'])
end

%% show fits images
img_processed = zeros(1024, 1024, 7);
for k = 1 : 7
    temp = fitsread(['flat', num2str(fitsname(1,k)), '.fits']);
    img_processed(:, :, k) = temp;
    figure(332), imagesc(temp), colorbar
    drawnow
    pause(1)
end

%% take flat field images
img = zeros(2*RY, 2*RX, 12);
for k = 1 : 12
    disp(num2str(k))
    fprintf(fw1,['pos=', num2str(k)]);
%     fprintf(fw1,['pos=', num2str(target.channel(k))]);
    pause(2)
    [A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
    img(:, :, k) = squeeze(A);
    figure(101), imagesc((abs(img(:, :, k)))),colorbar; colormap(CMRmap(100));
    drawnow
end

%% find dead pixels
deadSpot = darkFrame > median(darkFrame(:)) + 100;
sum(sum(deadSpot))

%% save the flat field images
fitsname = [550, 577, 600, 620, 632, 640, 650, 670, 694, 720, 740];
for k = 1 : 11
    temp =  img(:, :, k) - median(darkFrame(:));
    temp(deadSpot) = 0;
    temp = temp(:, 57:1136-56);
%     fitswrite(temp, ['flat', num2str(k), '.fits'])
    fitswrite(temp, ['flat', num2str(fitsname(1,k)), '.fits'])
end

%% show fits images
% img_processed = zeros(1024, 1024, 11);
for k = 1 : 11
    temp = fitsread(['flat', num2str(fitsname(1,k)), '.fits']);
%     img_processed(:, :, k) = temp;
    figure(332), imagesc(temp), colorbar
    drawnow
    pause(1)
end

%%
[A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
A(deadSpot) = median(darkFrame(:));
figure, imagesc(A-median(darkFrame(:))), colorbar

%% connect the linear stage of the lenslet array
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
    
%%
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

%%
for k = 1 : length(pos_list)
    figure(202), imagesc(spectral_image(400:800, 400:800, k)), colorbar;
    peak(k) = max(max(spectral_image(400:800, 400:800, k)));
    title(num2str(pos_list(k)));
    drawnow
    pause(1)
end
%%
for k = 1 : length(pos_list)
    figure(202), imagesc(spectral_image(450:550, 450:550, k)), colorbar;
    title(num2str(pos_list(k)));
%     peak(k) = max(max(spectral_image(:, :, k)));
    drawnow
    pause(1)
end
%%
exp_list = 20:20:500;%500:250:10000;
dark_median2 = zeros(length(exp_list), 1);
for k = 1 : length(exp_list)
    disp(k)
    [A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp_list(k), Xc, Yc, RX, RY, 0, 0, Nimg);
    dark_median2(k) =  median(A(:));
end
%% take flat field images
img = zeros(1024, 1136, 12);
for k = 1 : 12
    disp(num2str(k))
    fprintf(fw1,['pos=', num2str(k)]);
%     fprintf(target.laser,['pos=', num2str(k)]);
    pause(3)
    [A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
    img(:, :, k) = squeeze(A);
    figure(101), imagesc((abs(img(:, :, k)))),colorbar; colormap(CMRmap(100));
    drawnow
end


%%
for k = 1 : 12
    figure(102), imagesc(abs(img(400:500, 400:500, k)-dark(400:500, 400:500))),colorbar; colormap(CMRmap(100));
    pause(3)
end

%% save images as fits file
for k = 1 : 12
    fitswrite(img(:, :, k), ['channel', num2str(k), '_flat.fits'])
end


%%
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

%%
% stage_pos=linspace(0,10,11);
stage_pos=5.5:0.1:7;

img = zeros(1024, 1024, length(stage_pos));
for pos_index=1:length(stage_pos)
    h.SetAbsMovePos(0,stage_pos(pos_index));
    h.MoveAbsolute(0,1==0);
    pause(20)
    [A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
    img(:, :, pos_index) = squeeze(A);
    figure(100), imagesc((abs(img(:, :, pos_index)))),colorbar; colormap(CMRmap(100));
end

%%
for k = 16 : 26
    figure(101), imagesc((abs(img(400:500, 400:500, k)-dark(400:500, 400:500)))),colorbar; colormap(CMRmap(100));
    disp([num2str(k), ' ', num2str(stage_pos(k))]);
    drawnow
    pause(1)
end

%%
img_mean = zeros(length(stage_pos), 1);
img_max = zeros(length(stage_pos), 1);

for k = 1 : length(stage_pos)
    img_now = img(:, :,k);
    img_now(492, 564) = 0;
    img_mean(k) = mean(img_now(:));
    img_max(k) = max(img_now(:));
end

%%
% stage_pos=linspace(0,10,11);
stage_pos=5.5:0.1:7;

img = zeros(1024, 1024, length(stage_pos), 7);

for k = 1 : 7
    disp(num2str(k))
%     fprintf(fw1,['pos=', num2str(k)]);
    fprintf(target.laser,['pos=', num2str(target.channel(k))]);
    pause(2)
    for pos_index=1:length(stage_pos)
        h.SetAbsMovePos(0,stage_pos(pos_index));
        h.MoveAbsolute(0,1==0);
        if pos_index == 1
            pause(20)
        else
            pause(10)
        end
        [A,x,y,SAT]=SXcam_Nimg(hCam, 0, exp, Xc, Yc, RX, RY, 0, 0, Nimg);
        img(:, :, pos_index, k) = squeeze(A);
        figure(100), imagesc((abs(img(:, :, pos_index, k)))),colorbar; colormap(CMRmap(100));
    end
end