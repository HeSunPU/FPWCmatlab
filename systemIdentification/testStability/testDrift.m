% Test the stability of the system
% Developed by He Sun on Apr. 6th
x_index = zeros(1000, 1);
y_index = zeros(1000, 1);
peak = zeros(1000, 1);

for itr = 1 : 1000
    img = takeImg(h_camera, 30, exptime, start_pos, size_pixels, bin_pixels);
    IntIm = rot90(img,1);
    testImg = (double(rot90(img-darkCam,1)));
    [peak1, x] = max(max(testImg));
    [peak2, y] = max(max(testImg'));
    peak(itr) = 0.5 * (peak1 + peak2);
    x_index(itr) = x;
    y_index(itr) = y;
    figure(1), imagesc(testImg), colorbar;
    colormap jet;
    figure(2), plot(1:itr, x_index(1:itr), 'ro'); title('x');
    figure(3), plot(1:itr, y_index(1:itr), 'bs'); title('y');
    figure(4), plot(1:itr, peak(1:itr), 'k^'); title('peak');
    pause(400);
end