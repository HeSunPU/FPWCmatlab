dark_median = median(dark(:));
for k = 1 : 7
    img_mon = flat_field2(:, :, k);
    img_mon(dark>dark_median+50) = dark_median;
    img_mon_sub_dark = img_mon - dark_median;
    img_mon_sub_dark_resize = imresize(img_mon_sub_dark, [1024, 1024]);
    fitswrite(img_mon_sub_dark_resize, ['channel0', num2str(target.channel(k)), '_flat.fits'])
    figure(202), imagesc(img_mon_sub_dark_resize), colorbar
    drawnow
    pause(1)
end

%%
PSF_img = PSF_img3;
PSF_img(dark>dark_median+50) = dark_median;
PSF_img_sub_dark = PSF_img - dark_median;
PSF_img_sub_dark_resize = imresize(PSF_img_sub_dark, [1024, 1024]);
fitswrite(PSF_img_sub_dark_resize, 'PSF_broadband.fits')
figure(303), imagesc(PSF_img_sub_dark_resize), colorbar