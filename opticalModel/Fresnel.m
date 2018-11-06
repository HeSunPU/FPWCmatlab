function out = Fresnel(Uin, lambda, L, z)
%% The Fresnel propagation function
%
% Developed by He Sun on Feb. 8, 2017
% Modified from A.J.'s codes 'prop_PTP.m'
%
% This version requires a matrix with an even number of points in each
% dimension. It uses FFT. It returns an electric field of same size.
%
% Uin - source plane field
% lambda - wavelength of the light
% L - length of side of the source plane, which has to be square
% z - propagation distance

[M, N] =  size(Uin);
if(M ~= N)
    disp('Error: input field is not square!');
    return;
end

dx = L / M;
fx=-1/(2*dx):1/L:1/(2*dx)-1/L;    % freq coords
[FX,FY]=meshgrid(fx,fx);

H = fftshift( exp(-1i*pi*lambda*z*(FX.^2+FY.^2)) ); % transfer function
U1 = fft2(fftshift(Uin));      % shift, then fft source field
out = ifftshift(ifft2(H.*U1));    % inv fft, then center observation field   
end