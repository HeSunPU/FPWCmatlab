
% try size(I0)
% catch
%     load I0
%     load psf
% end
laser_power = [9;50];
exposure_time = [0.1;0.0001];
wing_max = zeros(size(laser_power));
wm_ind = zeros(size(laser_power));
camCentImgs = zeros(500,500,2);
centX = zeros(size(laser_power));
centY = zeros(size(laser_power));
psf_max = zeros(size(laser_power));

figure
for i= 1:1:2%
    Laser_Power(laser_power(i,1),1)
    camera.handle = Camera_ctrl(camera.handle, 'shutter', 1);
    darkCam = camera.darkFrame;
    numIm = 10;
%     numIm = 1;
    camera.exposure = exposure_time(i,1);%0.0001;%0.1;%
    img = takeImg(camera.handle, numIm, camera.exposure, [0,0], [500, 500], [4, 4]);
    IntIm = rot90(img,1);

    testImg = (double(rot90(img-darkCam,1)));
    testImg = imgaussfilt(testImg,4); % apply gaussian filter to image to make sure real centre is obtained (4th order empirically works)
    camCentImgs(:,:,i) = testImg;
    
    peakCountsPerPixPerSec = 5.1371e+08;
    peakCountsPerPix = peakCountsPerPixPerSec*camera.exposure;
    
    
%     camera.centre = [0,0];

    [centX(i),centY(i)] = find(ismember(testImg, max(testImg(:)))); %CHECK IF THIS SHOULD BE SWITCHED
    
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
    title(['PSF for t=',num2str(exposure_time(i)),'s and P = ',num2str(laser_power(i)),'W'])
    hold on
    plot(centY,centX,'k*')
    plot(centY(1),wm_ind(i),'kp')
    %plot(camera.centre(2)*ones(size(psf,1)), 1:1:size(psf,1),'r*')
    hold off
end

camera.center  = [centX(1),centY(1)] % right order?
target.flux = (wing_max(2)/wing_max(1))*psf_max(1)*10^4
%%
figure
subplot(1,2,1)
imagesc((abs(camCentImgs(:,:,1))))
colormap jet;
% title(['PSF for t=',num2str(exposure_time(i)),'s and P = ',num2str(laser_power(i)),'W'])
subplot(1,2,2)
imagesc((abs(camCentImgs(:,:,2))))
colormap jet;
% title(['PSF for t=',num2str(exposure_time(i)),'s and P = ',num2str(laser_power(i)),'W'])
