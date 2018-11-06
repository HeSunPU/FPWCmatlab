true_coef = [ -0.0020 -0.0682 -0.0210 -0.0302 0.0302 -0.0357 0.0032 0.0708 zeros(1,12)];
%    -0.0173
%    -0.0397
%     0.0106
%     0.0226
%    -0.0451
%    -0.0369
%    -0.0284
%     0.0093
%     0.0112
%    -0.0253
%    -0.0065
%    -0.0028
M=442;
minZerN = 1;
maxZerN = 5;
%%
xpup = -1:(2/(M-1)):1;

ypup = -1:(2/(M-1)):1;

k = 1;
%%
for i = 1:length(xpup)

    for j = 1:length(ypup)

        if((xpup(i)^2 + ypup(j)^2)<= 1)

            [th(k), r(k)] = cart2pol(xpup(i), ypup(j));

            xVec(k) = i;

            yVec(k) = j;

            k = k+1;

        end

    end

end
%%
indx = sub2ind([M M], xVec, yVec);

ci = 1;

Phase_in = zeros(M, M);

for n = minZerN:maxZerN

    for m = -n:2:n

        ZP = zeros(length(xpup), length(ypup));

        ZP(indx) = zernfun(n, m, r, th, 'norm');

        Phase_in = Phase_in + true_coef(ci)*ZP;

        ci = ci + 1;
        figure(10), imagesc(ZP), colorbar
        pause(1)

    end

end

figure; imagesc(Phase_in); colorbar; axis xy equal tight;
%%
noiseStd = 0.1 * ones(20,1);
planeSize = [256, 256];
N = sum(minZerN+1:maxZerN+1); % the number of Zernike polynomials used
assert(length(noiseStd)==N, 'The length of the noise vector is not equal to the number of Zernike polynomials!!');
zernikeCoef = noiseStd .* rand(N, 1); % randomly generate the Zernike coefficients

% define the Cartesian coordinates of the phase error
x = -1 : (2/(planeSize(1)-1)) : 1;
y = -1 : (2/(planeSize(2)-1)) : 1;
X = repmat(x, planeSize(2), 1);
Y = repmat(y', 1, planeSize(1));

% generate the polar coordinates from the Cartesian coordinates
circularMask = (X.^2 + Y.^2) <= 1;
circularIndex = find(circularMask(:) == 1);
[theta, rho] =  cart2pol(X(circularIndex), Y(circularIndex));

% generate the phase error from the Zernike coefficients
k = 1;
phaseErr = zeros(planeSize);
for n = minZerN : maxZerN
    for m = -n : 2 : n
        ZP = zeros(planeSize);
        ZP(circularIndex) = zernfun(n, m, rho, theta, 'norm');
        phaseErr = phaseErr + zernikeCoef(k) * ZP;
        k = k + 1;
    end
end
figure; imagesc(phaseErr); colorbar; axis xy equal tight;
