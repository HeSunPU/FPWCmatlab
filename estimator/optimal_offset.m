
%% compute the coordinate on the DM
dx = coronagraph.SPwidth / DM.DMmesh(2);
dy = coronagraph.SPwidth / DM.DMmesh(1);
xs = (-DM.DMmesh(2)/2 + 0.5 : DM.DMmesh(2)/2 - 0.5) * dx;
ys = (-DM.DMmesh(1)/2 + 0.5 : DM.DMmesh(1)/2 - 0.5) * dy;
[XS, YS] = meshgrid(xs, ys);

%% take unprobed image
I = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab); % take the unprobed image
% [~, ~, I] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command); % simulate the image
image(:, :, 1) = I; % save the unprobed image to the output variable
contrastMeasured = mean(I(darkHole.pixelIndex)); % the averaged measured contrast in the dark holes
disp('Unprobed image is taken.');
disp(['The averaged contrast in the dark holes is ', num2str(contrastMeasured)]);

%% choose the probe contrast
% if contrastEst <= 0 % if we currently do not have efficient estimation of contrast, for example the first iteration
%     contrastEst = contrastMeasured;
% end
contrastEst = contrastMeasured;

probeContrast = min(sqrt(contrastEst * 1e-5), 5e-4); % A heuristic law to calculate the probe contrast

disp(['Chosen probe contrast is ', num2str(probeContrast)]);
if data.itr > 0
    data.probeContrast(data.itr) = probeContrast;
end
%% compute the offsets of probed sinc waves
offsets = 0;
probeSP = probeShape(target, coronagraph, estimator, XS, YS, offsets(1), probeContrast); % the desired probe shape on pupil plane in meters
% Since the width of shaped pupil is different from DM, we should
% adjust the probe shape to DM plane
if coronagraph.SPwidth >= DM.widthDM
    marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
    marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));
    probeSPcrop = probeSP(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel);
    probeDM = imresize(probeSPcrop, DM.DMmesh);
else
    marginWidth = (DM.widthDM - coronagraph.SPwidth)/2;
    marginNpixel = round(marginWidth / DM.widthDM * DM.DMmesh(1));
    probeSPresized = imresize(probeSP, DM.DMmesh - 2 * marginNpixel);
    probeDM = zeros(DM.DMmesh);
    probeDM(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = probeSPresized;
end
% compute the DM voltage command to generate such surface shape
up = height2voltage(probeDM, DM.DMperfect, estimator.whichDM, 5);
opt_up = up;
up_values = py.numpy.array(up);
opt_cost = py.sensing.probe_cost(sensor, E_est_past_real, E_est_past_imag, P_est_past, command1, command2, beta, up_values);
disp(['offset: ', num2str(0), ' cost: ', num2str(opt_cost)]);
for angle = pi/6 : pi/6 : 2*pi-pi/6
    probeSP = probeShape(target, coronagraph, estimator, XS, YS, angle, probeContrast); % the desired probe shape on pupil plane in meters
    % Since the width of shaped pupil is different from DM, we should
    % adjust the probe shape to DM plane
    if coronagraph.SPwidth >= DM.widthDM
        marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
        marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));
        probeSPcrop = probeSP(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel);
        probeDM = imresize(probeSPcrop, DM.DMmesh);
    else
        marginWidth = (DM.widthDM - coronagraph.SPwidth)/2;
        marginNpixel = round(marginWidth / DM.widthDM * DM.DMmesh(1));
        probeSPresized = imresize(probeSP, DM.DMmesh - 2 * marginNpixel);
        probeDM = zeros(DM.DMmesh);
        probeDM(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = probeSPresized;
    end
    % compute the DM voltage command to generate such surface shape
    up = height2voltage(probeDM, DM.DMperfect, estimator.whichDM, 5);
    up_values = py.numpy.array(up);
    cost = py.sensing.probe_cost(sensor, E_est_past_real, E_est_past_imag, P_est_past, command1, command2, beta, up_values);
    if cost < opt_cost
        offsets = angle;
        opt_cost = cost;
        opt_up = up;
    end
    disp(['offset: ', num2str(angle*180/pi), ' cost: ', num2str(cost)]);
end
disp(['The optimal offset is ', num2str(offsets*180/pi)]);


%% subfunctions used before

function surf = probeShape(target, coronagraph, estimator, XS, YS, offset, probeContrast)
% Calculate the probe shape
%%
mx = (estimator.probeArea(2) - estimator.probeArea(1)) / coronagraph.SPwidth;
my = (estimator.probeArea(4) - estimator.probeArea(3)) / coronagraph.SPwidth; % frequency of the sinc wave, depending on the width of dark hole regions
wx = (estimator.probeArea(2) + estimator.probeArea(1)) / 2;
wy = (estimator.probeArea(4) + estimator.probeArea(3)) / 2; % frequency of the cosine wave, depending on the location of dark hole regions
SincAmp = target.starWavelength * sqrt(probeContrast) * sqrt(2 * pi); % amplitude of probe shape in meters;
if strcmpi(coronagraph.type, 'SPLC')
    surf = SincAmp * sinc(mx * (XS)) .* sinc(my * (YS)+2*pi) .* ... 
        cos(2 * pi * wx / coronagraph.SPwidth * XS + offset) .* cos(2 * pi * wy / coronagraph.SPwidth * YS);
else
    surf = SincAmp * sinc(mx * (XS)) .* sinc(my * (YS)) .* ... 
        cos(2 * pi * wx / coronagraph.SPwidth * XS + offset) .* cos(2 * pi * wy / coronagraph.SPwidth * YS);
end
surf = pi * surf; % A.J. added an extra factor in the old code, seems like some black art.
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
        disp('The DM used for probing should be either 1 or 2!');
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