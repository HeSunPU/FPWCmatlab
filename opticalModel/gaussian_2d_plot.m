
mat = ones(101,152);
sigma = 10;
center = [51,76];

mat2 = gauss2d(mat, sigma, center);

imagesc(mat2)

function val = gaussC(x, y, sigma, center)
xc = center(1);
yc = center(2);
exponent = ((x-xc).^2 + (y-yc).^2)./(2*sigma);
val       = (exp(-exponent));
end

function mat = gauss2d(mat, sigma, center)
gsize = size(mat);
[R,C] = ndgrid(1:gsize(1), 1:gsize(2));
mat = gaussC(R,C, sigma, center);
end