function img = takeImg(h, num, exptime, start_pos, size_pixels, bin_pixels)
    Camera_ctrl(h, 'exposureproperties', start_pos, size_pixels,...
            bin_pixels);
    tempImg = int32(zeros(size_pixels(2), size_pixels(1)));
    for itr = 1:num
        currentImg = Camera_ctrl(h, 'exposure', exptime);
        tempImg = tempImg + currentImg;
    end
    img = (1/num) * tempImg;
end