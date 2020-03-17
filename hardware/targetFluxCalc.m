%% Take Test Image

darkCam = camera.darkFrame;
numIm = 1;
%     numIm = 1;
camera.exposure = 1e-3;%0.03;% 0.001;%0.0001;%0.1;%

img = takeImg(camera.handle, numIm, camera.exposure, [0,0], [500, 500], [4, 4]);

testImg = (double(rot90(img-darkCam,1)));
max(max(testImg))

figure(3)
imagesc((abs(testImg)))
% imagesc(log10(abs(testImg)))
colormap jet;
colorbar
%% Calculate DM Pitch
s = 324-252; % distance bt main psf and ghost psf in pixels
DMperiod = 2; % number of actuators per sine wave on DM
DMpitch = (1/DMperiod)*coronagraph.focalLength*target.starWavelength/(camera.binXi*camera.pitch*s)

%% Calc Camera Center and Target Flux
% try size(I0)
% catch
%     load I0
%     load psf
% end
laser_power = [6;16];%[7;49];%[7;31];%
exposure_time = [0.008;1e-3];%[0.2;0.0001];%[0.03;1e-4];
% exposure_time = [0.0008;0.0001];
wing_max = zeros(size(laser_power));
wm_ind = zeros(size(laser_power));
camCentImgs = zeros(500,500,2);
centX = zeros(size(laser_power));
centY = zeros(size(laser_power));
psf_max = zeros(size(laser_power));

figure
for i= 1:1:2%
    Laser_Power(laser_power(i,1),target.starChannel)
    camera.handle = Camera_ctrl(camera.handle, 'shutter', 1);
    darkCam = camera.darkFrame;
    numIm = 10;
%     numIm = 1;
    camera.exposure = exposure_time(i,1);%0.0001;%0.1;%
    img = takeImg(camera.handle, numIm, camera.exposure, [0,0], [500, 500], [4, 4]);
    IntIm = rot90(img,1);

    testImg = (double(rot90(img-darkCam,1)));
    testImgGauss = imgaussfilt(testImg,4); % apply gaussian filter to image to make sure real centre is obtained (4th order empirically works)
    camCentImgs(:,:,i) = testImg;
%     
%     peakCountsPerPixPerSec = 5.1371e+08;
%     peakCountsPerPix = peakCountsPerPixPerSec*camera.exposure;
%     
%     
%     camera.centre = [0,0];

    [centX(i),centY(i)] = find(ismember(testImgGauss, max(testImgGauss(:)))); %CHECK IF THIS SHOULD BE SWITCHED
    
    psf_max(i) = max(max(testImg)); % intensity at main blob centre
    
    if i == 1
        psf_centreline = testImg(1:centX(1)-11,centY(1)); % only want to look at one wing
        negslope = find((diff(psf_centreline))<0); % find where the slope switches direction
        [wing_max(i,1),wm_ind(i,1)] = max(testImg(1:negslope(end),centY(1))); %last index in negslope should be where main psf peak ends
    else
        wing_max(i,1) = testImg(wm_ind(i-1,1),centY(i-1));
        wm_ind(i,1) = wm_ind(i-1,1);
    end

    subplot(1,2,i)
    imagesc((abs(testImg)))
    colormap jet;
    colorbar
    title(['PSF for t=',num2str(exposure_time(i)),'s and P = ',num2str(laser_power(i)),'W'])
%     title(['PSF for t=',num2str(exposure_time(i)),'s'])
    hold on
    plot(centY,centX,'k*')
    plot(centY(1),wm_ind(i),'kp')
    %plot(camera.centre(2)*ones(size(psf,1)), 1:1:size(psf,1),'r*')
    hold off
    
    
end
%% For thorlabs laser
camera.center  = [centX(1),centY(1)] % right order?
target.flux = (wing_max(2)/wing_max(1))*psf_max(1)/exposure_time(2)


% tf_check = (camCentImgs(106,322,2)/camCentImgs(106,322,1))*camCentImgs( 143, 321,1)/exposure_time(2)

%% For SuperK monocrhomatic:

target_flux1 = psf_max(1)/exposure_time(1)
target_flux2 = psf_max(2)/exposure_time(2)
%%
numIm = 1;
camera.exposure = 0.01;
img = takeImg(camera.handle, numIm, camera.exposure, [0,0], [500, 500], [4, 4]);
max(max(img))
testImg = (double(rot90(img-darkCam,1)));
max(max(testImg))
figure
imagesc(log10(abs(testImg)))
colormap jet;
colorbar
testImgGauss = imgaussfilt(testImg,4); % apply gaussian filter to image to make sure real centre is obtained (4th order empirically works)
[centX,centY] = find(ismember(testImgGauss, max(testImgGauss(:))))


%% Fit Image with stage at focus value
camera.exposure = 0.1;
I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
figure(111), imagesc(log10(abs(I0))), colorbar;
% caxis(cRange);
drawnow
camera.exposure = 0.1;
DM1command = zeros(DM.activeActNum, 1);
DM2command = zeros(DM.activeActNum, 1);
I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);

figure(8), imagesc(log10(abs(I0))), colorbar

I = I0 / max(max(I0));
f = fitFocalLength(I, target, coronagraph, camera)

