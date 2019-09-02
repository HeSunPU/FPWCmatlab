

for k = 2 : 16
    cur_img = img(:, :, k, 3);
    dark_median = median(dark(:));
    cur_img(dark>dark_median+100) = dark_median;
    cur_img_sub_dark = cur_img - dark_median;
    figure(202), imagesc( cur_img_sub_dark(400:500, 400:500)), colorbar
    caxis([0, 100])
    title(num2str(5.4+0.1*k))
    drawnow
    pause(1)
end

%%
img_filtered = imgaussfilt(cur_img_sub_dark, 1);

%%
reference_point = [317, 599];

%%
PSF_coord = [[556, 497]; [501, 449]; [416, 402]; [417, 372]; [415, 432]; [446, 760]; [433, 736]; ...
            [729, 777]; [716, 783]; [702, 759]; [787, 842]];

PSF = zeros(3, 3);

for k = 1 : size(PSF_coord, 1)
    PSF = PSF + cur_img_sub_dark(PSF_coord(k, 2)+(-1:1), PSF_coord(k, 1)+(-1:1));
%     figure(301), imagesc(cur_img_sub_dark(PSF_coord(k, 2)+(-1:1), PSF_coord(k, 1)+(-1:1)));
%     drawnow
%     pause(1)
end

PSF = PSF / size(PSF_coord, 1);

PSF_corner = 0.25 * (PSF(1, 1) + PSF(1, 3) + PSF(3, 1) + PSF(3, 3));
PSF_x_bound = 0.5 * (PSF(2, 1) + PSF(2, 3));
PSF_y_bound = 0.5 * (PSF(1, 2) + PSF(3, 2));
PSF_norm = PSF;
PSF_norm(2, 1) = PSF_x_bound;
PSF_norm(2, 3) = PSF_x_bound;
PSF_norm(1, 2) = PSF_y_bound;
PSF_norm(3, 2) = PSF_y_bound;
PSF_norm(1, 1) = PSF_corner;
PSF_norm(1, 3) = PSF_corner;
PSF_norm(3, 1) = PSF_corner;
PSF_norm(3, 3) = PSF_corner;

%%
detection = zeros(size(cur_img_sub_dark));
for k1 = 1+1 : 1024-1
    for k2 = 1+1 : 1024-1
        crop = cur_img_sub_dark(k1-1:k1+1, k2-1:k2+1);
%         crop_norm = crop / mean(abs(crop(:)));
        crop_norm = crop;
        detection(k1, k2) = sum(sum(PSF_norm * crop_norm)) / sum(sum(PSF_norm * PSF_norm));
    end
end

%%

pks = imregionalmax(cur_img_sub_dark, 8);
center = pks .* (cur_img_sub_dark >5);
figure, imagesc(pks), colorbar

%%
h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'data_cube.gif';
for k = 1 : size(temp, 1)
    % Draw plot for y = x.^n
    img_rotate = squeeze(temp(k, :, :));
    img_rotate = imrotate(img_rotate, 90-26.565);
    img_rotate = imgaussfilt(img_rotate);
    % figure,imagesc(log10(abs(img_rotate))), colorbar
    imagesc(img_rotate), colorbar
    drawnow
    % Capture the plot as an image 
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if k == 1 
      imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
      imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end 
end