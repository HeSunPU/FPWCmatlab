camera.exposure = 0.1;
DM1command = zeros(DM.activeActNum, 1);
DM2command = zeros(DM.activeActNum, 1);
I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);

figure(8), imagesc(log10(abs(I0))), colorbar
%%
I = I0 / max(max(I0));
f = fitFocalLength(I, target, coronagraph, camera)

%%
distance_in_pixel = 366 - 221;
pitch_act = (target.starWavelength * f) /  (distance_in_pixel * camera.pitch * camera.binEta)

%%
DM1command = data.DMcommand(1:952, 5);
DM2command = data.DMcommand(953:end, 5);
n_images = 200;
I_set = zeros(83, 99, n_images);
for k = 1 : n_images
    I0 = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
    I_set(:, :, k) = I0;
    disp(['Image #', num2str(k), ', Contrast: ', num2str(mean(I0(darkHole.pixelIndex)))])
    figure(111), imagesc(log10(abs(I0))), colorbar;
    caxis(cRange);
    drawnow
    pause(1)
end