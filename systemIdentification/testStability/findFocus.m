% use Gaussian fit to find the focus
% Developed by He Sun on Jun. 30, 2017
h_stage = motor_op(0, 'init');
%% 1.
nStep = 100;
Imax = zeros(nStep, 1);
h_camera = camera.handle;
darkCam = camera.darkFrame;
exptime = 4e-4;%0.0015;%0.1;
start_pos = camera.startPosition;
size_pixels = camera.imageSize;
bin_pixels = [camera.binEta, camera.binXi];
for k = 1:nStep
    motor_op(h_stage, 'goto_wait', 301-k);
    img = takeImg(h_camera, 3, exptime, start_pos, size_pixels, bin_pixels);
    IntIm = rot90(img,1);
    testImg = (double(rot90(img-darkCam,1)));
    figure(100), imagesc(testImg), colorbar
    drawnow
    pause(0.5)
    Imax(k) = max(max(testImg));
end

%% 2. fit the gaussian beam
f = fit((301-(1:100))', Imax, 'gauss1');
figure(101), plot((301-(1:100))', Imax);
focus = f.b1;
f

%% take images for Gerchberg Saxton
nImage = 6;
images = zeros(500, 500, nImage);
for k = 1 : nImage
    motor_op(h_stage, 'goto_wait', focus - 50*(k-1));
    img = takeImg(h_camera, 30, exptime, start_pos, size_pixels, bin_pixels);
    IntIm = rot90(img,1);
    testImg = (double(rot90(img-darkCam,1)));
    images(:,:,k) = testImg;
    figure(102), imagesc(testImg), colorbar
    drawnow
    pause(0.5)
end
%% Calculate Focal Length after moving mask, make sure image is not saturated

camera.exposure =0.008;% 0.05;%4e-4;%0.01;
DM1command = zeros(DM.activeActNum, 1);
DM2command = zeros(DM.activeActNum, 1);
I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);

I = I0 / max(max(I0));
f_fit = fitFocalLength(I, target, coronagraph, camera)
figure(22);
imagesc(I0)
colorbar

% img = takeImg(h_camera, 3, exptime, start_pos, size_pixels, bin_pixels);
% % IntIm = rot90(img,1);
% % testImg = (double(rot90(img-darkCam,1)));
% % figure(100), imagesc(testImg)
% f_fit = fitFocalLength(I, target, coronagraph, camera)