function output = takeDarkCam(h_camera, exptime, numIm,  BinX, BinY)
% %%
% focus_pos = 234.4468;%251.1470;
% motor_op(h_stage, 'goto', focus_pos);

%%
% exptime = 0.001;
folder = pwd;

h_camera = Camera_ctrl(h_camera, 'shutter', 0);
Camera_ctrl(h_camera, 'shutterpriority', 0);

Camera_ctrl(h_camera, 'readoutspeed', 0);

start_pos = [0,0];
%bin_pixels = [4,4];
%bin_pixels = [2,2];
bin_pixels = [BinX, BinY];
%size_pixels = floor([2758,2208]./bin_pixels);
size_pixels = [500, 500];%[1000, 1000];

darkCam = takeImg(h_camera, numIm, exptime, start_pos, size_pixels, bin_pixels);

cd('C:\Lab\FPWC\hardware');
save darkCam.mat darkCam
cd(folder);
output = darkCam;
end