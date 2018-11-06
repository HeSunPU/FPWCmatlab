function out = Fourier(Ein, f, lambda, Lx, Ly, dxi, Nxi, deta, Neta)
%% The Fourier propagation function
%
% Developed by He Sun on Feb. 8, 2017
% Modified from A.J.'s codes 'prop_PtoF_mft_xy.m'
%
% Ein -  pupil plane field
% f - focal length of the focusing optics in meters
% lambda - wavelength of the light
% Lx - horizontal length
% Ly - vertical length
% dxi - horizontal pixel size in focal plane
% Nxi - number of horizontal pixels in focal plane
% deta - vertical pixel size in focal plane
% Neta - number of vertical pixels in focal plane

[N, M] = size(Ein);

% Compute the pupil plane coordinates
if( mod(M,2) == 0 ) % input array has even dimensions
    dx = Lx / M;
    xs = (-M/2 : M/2-1)' * dx + dx / 2;
elseif( mod(M,2) == 1 ) % input array has odd dimensions
    dx = Lx / (M-1);
    xs = (-floor(M/2) : floor(M/2))' * dx;
end

if( mod(N,2) == 0 ) % input array has even dimensions
    dy = Ly / N;
    ys = (-N/2 : N/2-1) * dy + dy / 2;
elseif( mod(N,2) == 1 ) % input array has odd dimensions
    dy = Ly / (N-1);
    ys = (-floor(N/2) : floor(N/2)) * dy;
end

% Compute the image plane coordinates
if (mod(Nxi,2)==0)
    xis = (-Nxi/2:Nxi/2-1)'*dxi + dxi/2;
elseif (mod(Nxi,2)==1)
    xis = (-floor(Nxi/2):floor(Nxi/2))'*dxi;
end
if (mod(Neta,2)==0)
    etas = (-Neta/2:Neta/2-1)'*deta + deta/2;
elseif (mod(Neta,2)==1)
    etas = (-floor(Neta/2):floor(Neta/2))'*deta;
end
xis = xis.';
[XI, ETA] = meshgrid(xis, etas);

% Compute the Fourier propagation
rect_mat_pre = (exp(-2*pi*1i*(etas*ys)/(lambda*f)));
rect_mat_post  = (exp(-2*pi*1i*(xs*xis)/(lambda*f)));
% out = dx*deta*exp(1i*2*pi*f/lambda)/(1i*lambda*f)*exp(1i*pi/(lambda*f)*(XI.^2 + ETA.^2)).*(rect_mat_pre*Ein*rect_mat_post);
out = sqrt(dx*dy*deta*dxi)*exp(1i*2*pi*f/lambda)/(1i*lambda*f)*(rect_mat_pre*Ein*rect_mat_post);

end