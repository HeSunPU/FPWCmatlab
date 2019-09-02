fluxBroad = zeros(18, 1);
% for k = 3 : 3 : 15
% for k = 3 : 2: 15
for k = 1 : 18
    figure(1001), imagesc(log10(abs(cubeRotate(:, :, k)/max(max(cubeRotate(:, :, k)))))), colorbar
%     figure(1001), imagesc(log10(abs(cubeRotate(:, :, k))/camera.IFSflux(k)/camera.exposure)), colorbar
    title(['HCIFS Data Cube Animation (', num2str(round(camera.IFSlam(k)*1e9)), 'nm)'])
%     title(num2str(camera.IFSlam(k)*1e9));
    caxis([-4, 0])
%     caxis([-7, -3])
    fluxBroad(k) = max(max(cubeRotate(:, :, k)));
    drawnow
    pause(1)    
end

%% create gif file
h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'HCIFS_datacube2.gif';
for k = 3 : 2 : 15
    % Draw plot for y = x.^n
    imagesc(log10(abs(cubeRotate(:, :, k)/max(max(cubeRotate(:, :, k)))))), colorbar
    title(['HCIFS Data Cube Animation (', num2str(round(camera.IFSlam(k)*1e9)), 'nm)'])
    caxis([-4, 0])
    drawnow
    % Capture the plot as an image 
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if k == 3 
      imwrite(imind,cm,filename,'gif', 'DelayTime',1, 'Loopcount',inf); 
    else 
      imwrite(imind,cm,filename,'gif', 'DelayTime',1,'WriteMode','append'); 
    end 
end

%%
fl = zeros(18, 1);
for k = 3 : 15
% k = 2;
target.starWavelength = camera.IFSlam(k);%660e-9;%
I = cubeRotate(:, :, k)/max(max(cubeRotate(:, :, k)));
figure(1002), imagesc(I), colorbar
f = fitFocalLength(I, target, coronagraph, camera);
% f = fitFocalLengthIFS(I, target, coronagraph, camera)
fl(k) = f;
end

%%
fprintf(fw1,'pos=12');
[imgIFS, datacube] = takeIFSImg(camera);
cubeRotate = rotateAndCropIFS(datacube, camera);
k = 12;
target.starWavelength = camera.IFSlam(k);
I = cubeRotate(:, :, k)/max(max(cubeRotate(:, :, k)));
figure(1003), imagesc(I), colorbar
f = fitFocalLength(I, target, coronagraph, camera)

