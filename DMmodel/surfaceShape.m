function surf = surfaceShape(DM, command2D, index)
%% The deformable mirror model that convert command voltage to surface shape
% Developed by He Sun on Feb. 15, 2007
% Modified from 'hcil_dmVtoH_interp.m' written by A.J. Riggs
%
% surf - the output DM surface shape in meters
% DM - define DM model and parameters of devices
% index - the index of DM we are calculating
%% calculate the surface shape using different models

switch DM.model
    case 'influencingFunction'
        surf = influencingFunction(DM, command2D, index);
    otherwise
        disp('Currently, we only provide the influencing function mode. Others are under development!');
        return;
end


end

function out = influencingFunction(DM, command2D, index)
% the surface shape from linear superpostions of influencing functions

%% calculate the normalized influencing function of a single actuator
% define the influencing function shape by giving std and sampling width
sigma = 1.125;% 1.172; % std of Gaussian fit of influencing function in the unit of pitch size
width = 5; % sample width of influencing function in the unit of pitch size
dx = DM.Nact * DM.pitch / DM.DMmesh(2); 
dy = DM.Nact * DM.pitch / DM.DMmesh(1); % the size of each mesh grid in y direction
sigmax = sigma * DM.pitch; % std in meters in x direction
sigmay = sigma * DM.pitch; % std in meters in y direction
widthx = ceil(width * DM.pitch / dx);
widthy = ceil(width * DM.pitch / dy); % sampling width in pixels
% make the influencing function
if mod(widthx, 2) == 1
    xs = (-floor(widthx/2) : floor(widthx/2)) * dx;
else
    xs = (-widthx/2 + 1: widthx/2) * dx;
end
if mod(widthy, 2) == 1
    ys = (-floor(widthy/2) : floor(widthy/2)) * dy;
else
    ys = (-widthy/2 + 1: widthy/2) * dy;
end
[YS, XS] = meshgrid(ys, xs); % the coordinate mesh grid of influencing function
% infFunc = exp(-1/2 * ((XS/sigmax).^2 + (YS/sigmay).^2));
infFunc = exp(-4 * log(2) * ((XS/sigmax).^2 + (YS/sigmay).^2));

%% deform the DM surface according to the voltages of each actuator
surfaceAug = zeros(2 * DM.DMmesh(1), 2 * DM.DMmesh(2)); % augment the surface to twice large
xsSurf = (-DM.DMmesh(2) + 0.5 : DM.DMmesh(2) - 0.5) * dx;
ysSurf = (-DM.DMmesh(1) + 0.5 : DM.DMmesh(1) - 0.5) * dy; % the coordinates of surface mesh
xsActuator = (-DM.Nact/2 + 0.5 : DM.Nact/2 - 0.5) * DM.pitch;
ysActuator = (-DM.Nact/2 + 0.5 : DM.Nact/2 - 0.5) * DM.pitch; % the coordinates of each actuator center
xInfRange = -floor(size(infFunc,2)/2) : ceil(size(infFunc,2)/2) - 1;
yInfRange = -floor(size(infFunc,1)/2) : ceil(size(infFunc,1)/2) - 1;
switch index
    case '1'
        gain = DM.DM1gain .* command2D;
    case '2'
        gain = DM.DM2gain .* command2D;
    otherwise
        disp('We only have two DMs right now! Please set index as either 1 or 2.');
        return;
end
for k = 1 : DM.Nact
    for l = 1 : DM.Nact
        if(command2D(l, k) ~= 0) % Don't waste time on a non-poke
            [~, xIndex] = min(abs(xsSurf - xsActuator(l)));
            [~, yIndex] = min(abs(ysSurf - ysActuator(k))); % find the center pixel of a certain actuator
            surfaceAug(yIndex + yInfRange, xIndex + xInfRange) = surfaceAug(yIndex + yInfRange, xIndex + xInfRange) + ...
                gain(l, k) * infFunc;
        end
    end
end
out = surfaceAug(floor(DM.DMmesh(1)/2) + 1 : floor(DM.DMmesh(1)/2) + DM.DMmesh(1), ...
    floor(DM.DMmesh(2)/2) + 1 : floor(DM.DMmesh(2)/2) + DM.DMmesh(2));
out = rot90(out, 1);
out = flipud(out);
end