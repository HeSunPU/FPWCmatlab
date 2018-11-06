function mask = createMask(target, coronagraph, camera, type, side, rangeX, rangeY, rangeR, rangeAngle)
%% The function that creates the FPM or dark hole region mask
% Developed by He Sun on Feb. 16, 2017
% Modified from 'hcil_makeFPM.m' written by A.J. Riggs
%
% mask - the output mask, 1 for transmissive, 0 for solid
% target - defines the properties of light source
% coronagraph - defines the coronagraph type, shape and distances
% camera - defines the properties of camera, including pixel size, binning,
% noises and others
% type - defines the dark hole shape, 'wedge' or 'box'
% side - defines which side the dark holes are located
% rangeX - defines the horizontal range, only used for 'box' type
% rangeY - defines the vertical range, only used for 'box' type
% rangeR - difines the radial range, only used for 'wedge' type
% rangeAngle - defines the angular range, only used for 'wedge' type

%% calculate the focal plane coordinates of each pixel
fLambdaOverD = target.starWavelength * coronagraph.focalLength / coronagraph.SPwidth; % f * lambda / D
xs = (-camera.Nxi/2 + 0.5 : camera.Nxi/2 - 0.5) * camera.pitch * camera.binXi / fLambdaOverD;
ys = (-camera.Neta/2 + 0.5 : camera.Neta/2 - 0.5) * camera.pitch * camera.binEta / fLambdaOverD;
[XS, YS] = meshgrid(xs, ys); % focal plane Cartesian coordinates in f * lambda / D
RS = sqrt(XS.^2 + YS.^2);
TANS = YS./XS; % focal plane polar coordinates

%% create the mask with both side dark holes
mask = zeros(camera.Neta, camera.Nxi); % initialize the mask, which has same size as focal image
switch type
    case 'box'
        for k = 1 : camera.Neta
            for l = 1 : camera.Nxi
                if(YS(k, l) >= rangeY(1) && YS(k, l) <= rangeY(2) && abs(XS(k, l)) >= rangeX(1) && abs(XS(k, l)) <= rangeX(2))
                    mask(k, l) = 1;
                end
            end
        end
    case 'wedge'
        for k = 1 : camera.Neta
            for l = 1 : camera.Nxi
                if(RS(k, l) >= rangeR(1) && RS(k, l) <= rangeR(2) && abs(TANS(k ,l)) < tand(rangeAngle))
                    mask(k, l) = 1;
                end
            end
        end
    case 'circ'
        for k = 1 : camera.Neta
            for l = 1 : camera.Nxi
                if(RS(k, l) >= rangeR(1) && RS(k, l) <= rangeR(2))
                    mask(k, l) = 1;
                end
            end
        end
    otherwise
        disp('We only support box or wedge dark hole!')
        return;
end


%% only keep the side where dark holes are defined
switch side
    case 'L'
        mask(XS>0) = 0;
    case 'R'
        mask(XS<0) = 0; 
    case 'LR'
    otherwise
        disp('The dark holes should be only defined on left, right or both sides!');
        return;
end
end