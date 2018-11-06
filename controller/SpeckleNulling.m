function [command, probeImage] = SpeckleNulling(I, target, DM, coronagraph, camera, controller, darkHole, DM1command, DM2command, simOrLab)
% Probe one DM and take probing images
% Developed by He Sun on Sep. 30, 2018
%
% I - the measured image
% PSF- the point spread function core
% target - defines the properties of light source
% DM - defines the DM model and parameters of devices
% coronagraph - defines the coronagraph type, shape and distances
% camera - defines the properties of camera, including pixel size, binning,
% noises and others
% DM1command, DM2command - the current voltage commands of DMs
%%
centerEta = (camera.Neta + 1) / 2; % the center pixel coordinate
centerXi = (camera.Nxi + 1) / 2; % the center pixel coordinate
%%
if target.broadBandControl
    amp_max = 0;
    wavelength_max = 0;
    positionEta_max = 0;
    positionXi_max = 0;
    for k = 1 : target.broadSampleNum
        Iw = I(:, :, k) .* darkHole.mask;
        PSF_normalization = sum(sum(controller.PSF(:, :, k).^2));
        Iconv = conv2(Iw, controller.PSF(:, :, k), 'same') / PSF_normalization;

        figure(3), imagesc(Iconv), colorbar;
        drawnow

        % detect the index of maximal speckle
        [argmax1, argmax2] = argmax(Iconv);
        positionEta = (argmax1 - centerEta) * camera.binEta * camera.pitch / (target.starWavelength * coronagraph.focalLength / coronagraph.SPwidth); % vertical position of the brightest speckles in f * lamnda / D
        positionXi = (argmax2 - centerXi) * camera.binXi * camera.pitch / (target.starWavelength * coronagraph.focalLength / coronagraph.SPwidth); % horizontal position of the brightest speckles in f * lamnda / D
        amp = target.starWavelengthBroad(k) / (2*pi) * sqrt(Iconv(argmax1, argmax2));
        if amp > amp_max
            amp_max = amp;
            wavelength_max = target.starWavelengthBroad(k);
            positionEta_max = positionEta;
            positionXi_max = positionXi;
        end
        target.starWavelength = wavelength_max;
        positionEta = positionEta_max;
        positionXi = positionXi_max;
    end
    
else
    % I(:, centerXi:end) = 0;
    I = I .* darkHole.mask;
    PSF_normalization = sum(sum(controller.PSF.^2));
    Iconv = conv2(I, controller.PSF, 'same') / PSF_normalization;

    figure(3), imagesc(Iconv), colorbar;
    drawnow
    
%     BW = imregionalmax(abs(Iconv), 8);
%     figure(4), imagesc(BW .* darkHole.mask), colorbar;
%     drawnow
    
    % detect the index of maximal speckle
    [argmax1, argmax2] = argmax(Iconv);
    positionEta = (argmax1 - centerEta) * camera.binEta * camera.pitch / (target.starWavelength * coronagraph.focalLength / coronagraph.SPwidth); % vertical position of the brightest speckles in f * lamnda / D
    positionXi = (argmax2 - centerXi) * camera.binXi * camera.pitch / (target.starWavelength * coronagraph.focalLength / coronagraph.SPwidth); % horizontal position of the brightest speckles in f * lamnda / D
    amp = target.starWavelength / (2*pi) * sqrt(Iconv(argmax1, argmax2));
end
%% compute the coordinate on the DM
dx = coronagraph.SPwidth / DM.DMmesh(2);
dy = coronagraph.SPwidth / DM.DMmesh(1);
xs = (-DM.DMmesh(2)/2 + 0.5 : DM.DMmesh(2)/2 - 0.5) * dx;
ys = (-DM.DMmesh(1)/2 + 0.5 : DM.DMmesh(1)/2 - 0.5) * dy;
[XS, YS] = meshgrid(xs, ys);

%% Find optimal offset by fitting the contrasts with a sine wave
Nitr = 5;
offset = [0, pi/2, pi, 3*pi/2];
contrasts = zeros(4, 1);
probeImage = zeros(camera.Neta, camera.Nxi, 4);
for j = 1:4
    probeDM = controlShape(coronagraph, XS, YS, offset(j), amp, positionEta, positionXi);
    command = height2voltage(probeDM, DM.DMperfect, controller.whichDM, Nitr);
%     ItestBroad = zeros(camera.Neta, camera.Nxi, target.broadSampleNum);
%     for kWavelength = 1 : target.broadSampleNum
    targetmon = target;
%         targetmon.starWavelength = target.starWavelengthBroad(kWavelength);
    Itest = getImg(targetmon, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
%         ItestBroad(:, :, kWavelength) = Itest;
%     end
%     Itest = mean(Itest, 3);
    contrasts(j) =  mean(Itest(darkHole.pixelIndex));
    probeImage(:, :, j) = Itest;
end
% B = mean(contrasts);
% A = 0.5 * sqrt((contrasts(1)-contrast(3))^2 + (contrasts(2)-contrasts(4))^2);
theta = atan2(contrasts(1)-contrasts(3), contrasts(2)-contrasts(4));

optimalOffset = mod((1.5*pi-theta), 2*pi);
probeDM = controlShape(coronagraph, XS, YS, optimalOffset, amp, positionEta, positionXi);
command = height2voltage(0.6*probeDM, DM, controller.whichDM, Nitr);
% Itest = getImg(target, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
% mean(Itest(darkHole.pixelIndex))

%% Find optimal offset by line search
% Nitr = 5; % the iteration for computing DM command from probe surface shape
% numOffset = 36; % the number of offsets tested for the sine wave probing
% contrasts = zeros(numOffset, 1); % the measured contrast for different offsets
% for j = 1 : numOffset
%     offset = (j - 1) * (2 * pi / numOffset);
%     probeDM = controlShape(coronagraph, XS, YS, offset, amp, positionEta, positionXi);
%     command = height2voltage(probeDM, DM, controller.whichDM, Nitr);
%     Itest = getImg(target, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
%     contrasts(j) =  mean(Itest(darkHole.pixelIndex));
% %     disp([j, contrasts(j)*1e4])
% end
% [~, opt] =  min(contrasts);
% optimalOffset = (opt - 1) * (2 * pi / numOffset);
% probeDM = controlShape(coronagraph, XS, YS, optimalOffset, amp, positionEta, positionXi);
% command = height2voltage(probeDM, DM, controller.whichDM, Nitr);
% Itest = getImg(target, DM, coronagraph, camera, DM1command + command, DM2command, simOrLab);
% mean(Itest(darkHole.pixelIndex))

end

function surf = controlShape(coronagraph, XS, YS, offset, amp, positionEta, positionXi)
% Calculate the sine wave control shape on the deformable mirror
% Developed by He Sun on Oct. 1, 2018
%
wy = 2 * pi / (coronagraph.SPwidth / positionEta);
wx = 2 * pi / (coronagraph.SPwidth / positionXi);
surf = amp * sin(wx * XS + wy * YS + offset);
end


function command = height2voltage(surf, DM, index, Nitr)
% Use iterated calculation to find approximate DM command for a surface
% shape
%
% Select the DM gain matrix according to the DM index
switch index
    case '1'
        gain = DM.DM1gain;
    case '2'
        gain = DM.DM2gain;
    otherwise
        disp('The DM used for control should be either 1 or 2!');
        return;
end
% use simple gradient optimization to calculate the voltage
command2D = imresize(surf, [DM.Nact, DM.Nact]) ./ gain;
command2D = command2D .* DM.DMstop;
for k = 1 : Nitr
    currentSurf = surfaceShape(DM, command2D, index);
    surfBias = surf - currentSurf;
    command2D = command2D + imresize(surfBias, [DM.Nact, DM.Nact]);
    command2D = command2D .* DM.DMstop;
end
command = command2D(DM.activeActIndex);
end


function [argmax1, argmax2] = argmax(I)
% Find the max index of a 2-D array
% Developed by He Sun on Sep. 30, 2018
%
% I - the input 2-D array
[temp1, temp2] = max(I);
[~, argmax2] = max(temp1);
argmax1 = temp2(argmax2);
end