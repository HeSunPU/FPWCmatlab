
% Lab_initialization.m - initialize the QSI camera settings for lab
% Including turn on the camera, turn off the camera, take test image, take 
% dark image and calculate the standard deviation of each pixel
% Developed by He Sun on Mar. 5th, 2016

%% Initialize the DM - open drivers and put flat map on the surface
% NOTE: change these to match your install paths
addpath('C:\BostonMicromachines v5.2\BostonMicro');
addpath('C:\BostonMicromachines v5.2\BostonMicro\Matlab\v5.2');
addpath('C:\BostonMicromachines v5.2\Winx64\');
addpath('Z:\Matlab\data');
addpath('C:\Lab\HCIL\FPWC\HCIL\util');
% Parameters of DMs
BrdNum = 1;
% constants
dac_scale = (2^16)-1; 
MAX_V = 300;
% poke values
V1 = dac_scale/2; 
HVA_type = 'KILO2x952';
Size = 4096; %DM Size
error = SetUpHVA(BrdNum, HVA_type);
count = 0;
[error, Mode] = GetCurrentHVAmode(BrdNum);
error = TTLo_FVAL(BrdNum);
error = AbortFramedData(BrdNum);
% Open driver
if (Size == 32)
    error = SetUpHVA(BRDNum, 'HVA32');
elseif(Size == 492)
    error = SetUpHVA(BrdNum, 'SD492');
elseif(Size == 952)
    error = SetUpHVA(BrdNum, 'SD952');
elseif (Size == 1024)
    error = SetUpHVA(BrdNum, 'SD1024');
elseif(Size == 4096)
    error = SetUpHVA(BrdNum, 'KILO2x952');
else 
    disp('No HVA Select');
end

if (error)
    err_str = Decode_Error(error);
    disp(err_str)
    disp('Aborting program...')
    %$return error;
end;
% v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');

% 
%  PREFORMATTED
%  TEXT
% 
% v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW004#14_CLOSED_LOOP_200nm_Voltages_DM#1.txt','-ascii');
v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW018#40_CLOSED_LOOP_200nm_Voltages_DM#2.txt','-ascii');

DM12bits =(2^16-1)/300*[v1flat(1:1024); v2flat(1:1024); zeros(2048,1)];
% DM12bits = (2^16-1)/300*[v2flat(1:1024); v1flat(1:1024); zeros(2048,1)];

error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%% Finalize DMs - zero the input voltage
BrdNum = 1;
error = BurstHVA4096Frame1D(BrdNum, zeros(4096,1));

%% Initialize the camera - open mechanical shutter, set up the cooling temperature
% and define exposure properties
camera.name = 'QSICamera.CCDCamera';
camera.handle = Camera_ctrl(0, 'enable'); % connect the computer to camera, save the handle
Camera_ctrl(camera.handle, 'init', -15); % enable the camera, set up the camera temperature
Camera_ctrl(camera.handle, 'shutterpriority', 1);

%% Finalize the camera - close the mechanical shutter, delete the camera handle
camera = Camera_ctrl(camera, 'disconnect');
%% Take dark image for control
camera.handle = Camera_ctrl(camera.handle, 'shutter', 0);
pause(.5) % wait for shutter to close
% for k = 1 : 30
numIm = 30;
camera.exposure = 0.1;
camera.fastReadout = 0;
darkCam = takeImg(camera.handle, numIm, camera.exposure, [0,0], [500, 500], [4, 4]);
figure, imagesc(darkCam), colorbar
drawnow
mean_std = [mean2(darkCam), std2(darkCam)]
% end
%% Take test image - set up binning and take test image
camera.handle = Camera_ctrl(camera.handle, 'shutter', 1);
darkCam = camera.darkFrame;
numIm = 10;
camera.exposure = 0.1;%0.0001;%
img = takeImg(camera.handle, numIm, camera.exposure, [0,0], [500, 500], [4, 4]);
IntIm = rot90(img,1);
% figure, imagesc(abs(double(rot90(img,1)))), colorbar;
figure, imagesc(((double(rot90(img-darkCam,1))))), colorbar;
testImg = (double(rot90(img-darkCam,1)));
% figure, imagesc((abs(double(rot90(img-darkBias,1))))), colorbar;
colormap jet;
figure, imagesc(log10(abs(testImg)/max(testImg(:)))), colorbar
peakCountsPerPixPerSec = 5.1371e+08;
peakCountsPerPix = peakCountsPerPixPerSec*camera.exposure;
% peakCountsPerPix = 10.2274e+8;
% figure, imagesc(log10(abs(double(IntIm-darkCam)/peakCountsPerPix))), colorbar;
% figure, imagesc(log10(abs(double(IntIm-darkCam)/peakCountsPerPix/exptime))), colorbar;
% colormap jet;
% figure, imagesc(log10(abs(double(rot90(img-darkCam,1))))), colorbar;
%colormap jet;

%% Take dark image of Starlight Xpress Camera
StackNum = 30;
darkCam = 0;
for itr = 1 : 30
    [Idark, FITSparam] = hcil_SXcamera(FITSparam, folders);
    darkCam = Idark + darkCam;
end
darkCam = darkCam / StackNum;
cd('C:\Lab\HCIL\FPWC\HCIL\util');
save darkCam.mat darkCam
cd('C:\Lab\HCIL\FPWC\HCIL');

%% Take test image using Starlight Xpress Camera
[Itest, FITSparam] = hcil_SXcamera(FITSparam, folders);
Ilab = Itest - darkCam;
figure, imagesc(Ilab), colorbar;

%%
binPix = [2,2]; %[4,4]
size_pixels = [1000,1000];%[500,500]
start_pos = [0,0];%[0,0]
expTime = 0.03;%0.03;
Camera_ctrl(camera, 'exposureproperties', start_pos, size_pixels,...
            binPix);
camera = Camera_ctrl(camera, 'shutter', 1);
Camera_ctrl(camera, 'shutterpriority', 1);
set(camera.camera, 'FanMode', 'FanFull');
figure(1)
filename = 'FanOffMode.gif';
for itr = 1:20
    img = takeImg(camera, 5, expTime, start_pos, size_pixels, binPix);
    figure(1), imagesc(log10(abs(double(rot90(img(550-50:550+40,767-50:767+40),1))))), colorbar;
    title(['Iteration ' num2str(itr)]);
    caxis([2.7 4.5])
%     colormap jet;
    drawnow
    frame = getframe(1);
      im = frame2im(frame);
      [imind,cm] = rgb2ind(im,256);
      if itr == 1;
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
      else
          imwrite(imind,cm,filename,'gif','WriteMode','append');
      end
end
% figure(2), imagesc(log10(abs(double(rot90(img(550-50:550+40,767-50:767+40),1))))), colorbar;colormap jet;
% figure(3), imagesc(log10(abs(double(img)))), colorbar;colormap jet;

%% Calculate the variance matrix for QSI camera
folder = pwd;
numIm = 30;
expTime = 0.03;
img_stack = zeros(500,500,numIm);
for itr = 1:numIm
    img_stack(:,:,itr) = Camera_ctrl(camera, 'exposure');
end
StdCam = std(img_stack, 0);
cd('C:\Lab\HCIL\FPWC\HCIL\util');
save StdCam.mat StdCam
cd(folder);
%% find the center shifting


%% Align mask with camera (sfr), first do coarse move with APT_GUI to get
% at least one half on the focal plane

% Desired position of top left corner
maskY = 6; %pixels from top of image for tip CONFIRM
maskX_tip = 20; %pixels from left of image for left tip CONFIRM
maskX_edge = 95; % pixels from left of image for right edge CONFIRM


% Get exposure, laser power should be 50
camera.exposure = 0.01;
DM1command = zeros(DM.activeActNum, 1);
DM2command = zeros(DM.activeActNum, 1);
I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
figure(8), imagesc(log10(abs(I0))), colorbar

% figure(8), subplot(1,2,1); imagesc(log10(abs(I0))), colorbar

% get portion of image where light goes through the mask
I0_l10 = log10(real(I0));
[maskinds_row,maskinds_col]= find(I0_l10 >= -5);

[row_tip,i_tip] = min(maskinds_row); % find highest pixel that is part of the mask

col_tip = maskinds_col(i_tip); % find column associated with tip

col_edge = max(maskinds_col);

% initialize motors
mask_hstage = motor_op_mask(0, 'init','hor');
mask_vstage = motor_op_mask(0, 'init','vert');

% get current positions of stages
currentPosX = motor_op_mask(mask_hstage, 'pos');
currentPosY = motor_op_mask(mask_vstage, 'pos');

% determine required absolute position
% moveX = (col_tip-maskX_tip)*camera.pitch*10^3 + currentPosX; % need to be able to get current pos and check sign
moveX = (col_edge-maskX_edge)*camera.pitch*10^3 + currentPosX; %  check sign
moveY = (row_tip-maskY)*camera.pitch*10^3 + currentPosY; % need to convert pith to mm, CHECK SIGN

% move motors
motor_op_mask(mask_hstage, 'goto' , moveX);
motor_op_mask(mask_vstage, 'goto' , moveY);

I02 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
subplot(1,2,2); imagesc(log10(abs(I02))), colorbar
