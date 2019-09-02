function img = rotateAndCropIFS(img, camera)
    for k = 1 : size(img, 3)
        img(:, :, k) = imrotate(img(:, :, k), 90-camera.philens, 'bicubic', 'crop');
    end
    img = img(camera.center(1)-camera.Neta_half:camera.center(1)+camera.Neta_half, ...
            camera.center(2)-camera.Nxi_half:camera.center(2)+camera.Nxi_half, :);
end