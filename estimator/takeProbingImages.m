function [image, u, data] = takeProbingImages(contrastEst, target, DM, coronagraph, camera, darkHole, estimator, DM1command, DM2command, simOrLab, data)
% Probe one DM and take probing images
% Developed by He Sun on Feb. 22, 2017, revised from A. J. Riggs's
% "hcil_makeProbeSurf.m" file
%
% contrastEst - the estimated contrast, which is used to compute the
% amplitude of DM probing
% target - defines the properties of light source
% DM - defines the DM model and parameters of devices
% coronagraph - defines the coronagraph type, shape and distances
% camera - defines the properties of camera, including pixel size, binning,
% noises and others
% darkHole - defines the dark hole region
% estimator - defines the parameters of wavefront estimator
% DM1command, DM2command - the current voltage commands of DMs
%
%% initialize the output variables
% if ((strcmpi(estimator.type, 'EKF') || strcmpi(estimator.type, 'UKF')) && estimator.EKFpairProbing == 0)
if estimator.EKFpairProbing == 0
    image = zeros(camera.Neta, camera.Nxi, 1 + estimator.NumImg); % unprobed and probed images
else
    image = zeros(camera.Neta, camera.Nxi, 1 + 2 * estimator.NumImgPair); % unprobed and probed images
end
if strcmpi(estimator.whichDM, 'both')
    switch lower(estimator.type)
        case {'ekf', 'ukf'}
            u = zeros(2*DM.activeActNum, estimator.NumImg);
        otherwise
            if estimator.EKFpairProbing == 0
                u = zeros(2*DM.activeActNum, estimator.NumImg);
            else 
                u = zeros(2*DM.activeActNum, estimator.NumImgPair); % control commands for probing
            end
    end
else
    switch lower(estimator.type)
        case {'ekf', 'ukf'}
            u = zeros(DM.activeActNum, estimator.NumImg);
        otherwise
            if estimator.EKFpairProbing == 0
                u = zeros(DM.activeActNum, estimator.NumImg);
            else 
                u = zeros(DM.activeActNum, estimator.NumImgPair); % control commands for probing
            end
    end
end
Nitr = 5; % number of iterations used for calculating voltages of a specific shape

%% compute the coordinate on the DM
dx = coronagraph.SPwidth / DM.DMmesh(2);
dy = coronagraph.SPwidth / DM.DMmesh(1);
xs = (-DM.DMmesh(2)/2 + 0.5 : DM.DMmesh(2)/2 - 0.5) * dx;
ys = (-DM.DMmesh(1)/2 + 0.5 : DM.DMmesh(1)/2 - 0.5) * dy;
[XS, YS] = meshgrid(xs, ys);

%% take unprobed image
camera.exposure = camera.exposure0;
I = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab); % take the unprobed image
% [~, ~, I] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command); % simulate the image
image(:, :, 1) = I; % save the unprobed image to the output variable
contrastMeasured = mean(I(darkHole.pixelIndex)); % the averaged measured contrast in the dark holes
disp('Unprobed image is taken.');
disp(['The averaged contrast in the dark holes is ', num2str(contrastMeasured)]);

if contrastEst <= 0 % if we currently do not have efficient estimation of contrast, for example the first iteration
    contrastEst = contrastMeasured;
else
    contrastEst = max(contrastMeasured, 2e-10);
%     contrastEst = max(contrastEst, 2e-10);
end
disp(['Measured Contrast is ', num2str(contrastMeasured), ', Predicted Contrast is ', num2str(contrastEst)]);
%% choose the exposure time
if data.itr > 0
%     data.probeContrast(data.itr) = probeContrast;    
    
        if strcmpi(estimator.type, 'kalman') || strcmpi(estimator.type, 'batch')
            if camera.adaptive_exposure
    %             camera.exposure = sqrt(estimator.observationVarCoefficient / estimator.observationVarCoefficient3) / probeContrast;
                camera.exposure = 0.1 * 1.0 / (target.flux) / estimator.observationVarCoefficient3 / contrastEst;
                data.probe_exposure(data.itr) = camera.exposure;                
            else
                camera.exposure = camera.exposure0;
                data.probe_exposure(data.itr) = camera.exposure;
            end
            disp(['The chosen exposure time is ', num2str(camera.exposure), 's.']);
        elseif strcmpi(estimator.type, 'EKF')
            if data.itr == 1
                command = data.DMcommand(:, 1);
            else
                command = data.DMcommand(:, data.itr) - data.DMcommand(:, data.itr - 1);
            end
            Q = sum(command.^2) * estimator.processVarCoefficient + estimator.processVarCoefficient2;
            if data.itr == 1
                sigma_square = estimator.stateStd0 + Q;
            else
                sigma_square = mean(squeeze((data.P(1, 1, :, data.itr-1) + data.P(2, 2, :, data.itr-1)) / 2)) + Q;
            end
            if camera.adaptive_exposure
                camera.exposure = 1 * 1.0 / (target.flux) / sqrt(8*estimator.observationVarCoefficient3) / sigma_square;
                data.probe_exposure(data.itr) = camera.exposure;
            else
                camera.exposure = camera.exposure0;
                data.probe_exposure(data.itr) = camera.exposure;
            end
            disp(['The chosen exposure time is ', num2str(camera.exposure), 's.']);
        else
            disp([estimator.type, ' estimator is not available!']);
        end
    
    
    % take unprobed image again
    I = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
    image(:, :, 1) = I;
end



%% choose the probe contrast
% if contrastEst <= 0 % if we currently do not have efficient estimation of contrast, for example the first iteration
%     contrastEst = contrastMeasured;
% end
% probeContrast = min(sqrt(contrastEst * 1e-5), 5e-4); % A heuristic law to calculate the probe contrast
% probeContrast = sqrt(estimator.observationVarCoefficient / estimator.observationVarCoefficient3 + contrastEst.^2);

% probeContrast = sqrt((estimator.observationVarCoefficient/(camera.exposure^2) + contrastEst / (target.flux * camera.exposure)) / estimator.observationVarCoefficient3 + contrastEst.^2);
if strcmpi(estimator.type, 'kalman') || strcmpi(estimator.type, 'batch')
    if estimator.EKFpairProbing
        probeContrast = min(sqrt(contrastEst * 1e-5), 5e-4); % A heuristic law to calculate the probe contrast
%         probeContrast = sqrt((estimator.observationVarCoefficient0/(camera.exposure^2) + contrastEst / (target.flux * camera.exposure)) / estimator.observationVarCoefficient3 + contrastEst.^2);
    else
        probeContrast = sqrt((2 * estimator.observationVarCoefficient0/(camera.exposure^2) + 2 * contrastEst / (target.flux * camera.exposure)) / estimator.observationVarCoefficient3 + contrastEst.^2);
    end
elseif strcmpi(estimator.type, 'EKF')
    probeContrast = min(sqrt(contrastEst * 1e-5), 5e-4); % A heuristic law to calculate the probe contrast
%     probeContrast = abs(sqrt((estimator.observationVarCoefficient0 / (camera.exposure^2) + 2 * sigma_square^2) / estimator.observationVarCoefficient3) - contrastEst);
else
    disp([estimator.type, ' estimator is not available!']);
end
disp(['Chosen probe contrast is ', num2str(probeContrast)]);

if data.itr > 0
    data.probeContrast(data.itr) = probeContrast;
end

%% compute the offsets of probed sinc waves
switch lower(estimator.probeMethod)
    case 'empirical'   
        switch lower(estimator.type)
            case {'ekf', 'ukf'}
                if (estimator.EKFpairProbing == 0)
                    omega = pi/2/estimator.NumImg;
                    offsets = (omega * (data.itr-1) + (0 : estimator.NumImg-1))' * pi / estimator.NumImg;
                else
                    omega = pi/2/estimator.NumImgPair;
%                     omega = 1.5708;%/estimator.NumImgPair;
                    offsets = (omega * (data.itr-1) + (0 : estimator.NumImgPair-1))' * pi / estimator.NumImgPair;
                end
            otherwise
                if (estimator.EKFpairProbing == 0)
                    omega = pi/2/estimator.NumImg;
                    offsets = (omega * (data.itr-1) + (0 : estimator.NumImg-1))' * pi / estimator.NumImg;
                else
                    omega = pi/2/estimator.NumImgPair;
%                     omega = 1.5708;%1.5708/estimator.NumImgPair;%
                    offsets = (omega * (data.itr-1) + (0 : estimator.NumImgPair-1))' * pi / estimator.NumImgPair;
                end
        end
    otherwise
        disp('The probing offset has not been assigned!');
        return;
end
%% compute the probe commands
if estimator.EKFpairProbing
    probe_number = estimator.NumImgPair;
else
    probe_number = estimator.NumImg;
end
for k = 1 : probe_number
    if strcmpi(coronagraph.type, 'VORTEX')
        probeSP = probeShape(target, coronagraph, estimator, XS, YS, offsets(k), probeContrast, rem(k, 2));
    else
        probeSP = probeShape(target, coronagraph, estimator, XS, YS, offsets(k), probeContrast); % the desired probe shape on pupil plane in meters
    end
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
    if strcmpi(estimator.whichDM, 'both')
        command = zeros(DM.activeActNum*2, 1);
        command(1:DM.activeActNum) = height2voltage(probeDM, DM.DMperfect, '1', Nitr);
    else
        % compute the DM voltage command to generate such surface shape
        command = height2voltage(probeDM, DM.DMperfect, estimator.whichDM, Nitr);
%         command = min(sqrt(contrastEst * 1e8), 0.5) * rand(952, 1);
    end
    if estimator.EKFpairProbing
        switch lower(estimator.type)
            case {'ekf', 'ukf'}
                u(:, 2*k - 1) = command';
                u(:, 2*k) = -command'; 
            otherwise
                u(:, k) = command';
        end
    else
        u(:, k) = command';
    end
    
end
%% dont need for DH maintentance
if estimator.optimized_probe
    if strcmpi(estimator.type, 'batch') && estimator.EKFpairProbing == 1
        temp = load('opt_probe.mat');
        opt_probe = temp.opt_probe;
        u = sqrt(probeContrast/opt_probe.contrast) * opt_probe.up;
    elseif strcmpi(estimator.type, 'Kalman') && estimator.EKFpairProbing == 1
        temp = load('opt_probe.mat');
        opt_probe = temp.opt_probe;
        if estimator.NumImgPair == 1
            if rem(data.itr, 2) == 1
                u = sqrt(probeContrast/opt_probe.contrast) * opt_probe.up(:, 1);
            else
                u = sqrt(probeContrast/opt_probe.contrast) * opt_probe.up(:, 2);
            end
        elseif estimator.NumImgPair == 2
            u = sqrt(probeContrast/opt_probe.contrast) * opt_probe.up;
        else
            disp([estimator.type, ' estimator is not available!']);
        end
    elseif strcmpi(estimator.type, 'EKF') && estimator.EKFpairProbing == 0
        temp = load('opt_probe.mat');
        opt_probe = temp.opt_probe;
        if estimator.NumImg == 1 
            if rem(data.itr, 2) == 1
                u = sqrt(probeContrast/opt_probe.contrast) * opt_probe.up(:, 1);
            else
                u = sqrt(probeContrast/opt_probe.contrast) * opt_probe.up(:, 2);
            end
        elseif estimator.NumImg == 2
            u = sqrt(probeContrast/opt_probe.contrast) * opt_probe.up;
        else
            disp([estimator.type, ' estimator is not available!']);
        end
    else
        disp([estimator.type, ' estimator is not available!']);
    end
end
%% use optimization method to determine the probe shape and exposure time (dont need for DH maint)
if data.itr > 0 && estimator.activeSensing
    if data.itr == 1
        Enp_old = data.EfocalEst0;
        P_old = zeros(darkHole.pixelNum, 2, 2);
        P_old(:, 1, 1) = estimator.stateStd0;
        P_old(:, 2, 2) = estimator.stateStd0;
    else
        Enp_old = data.EfocalEst(:, data.itr-1);
        P_old = permute(data.P(:, :, :, data.itr-1), [3, 1, 2]);
    end
    E_old_real = py.numpy.array(real(Enp_old));
    E_old_imag = py.numpy.array(imag(Enp_old));
    P_old = py.numpy.array(P_old);
    if data.itr == 1
        u1c_values = py.numpy.array(data.DMcommand(1:DM.activeActNum, data.itr));
        u2c_values = py.numpy.array(data.DMcommand(DM.activeActNum+1:end, data.itr));
    else
        u1c_values = py.numpy.array(data.DMcommand(1:DM.activeActNum, data.itr) - data.DMcommand(1:DM.activeActNum, data.itr-1));
        u2c_values = py.numpy.array(data.DMcommand(DM.activeActNum+1:end, data.itr) - data.DMcommand(DM.activeActNum+1:end, data.itr-1));
    end
    if estimator.EKFpairProbing && strcmpi(estimator.type, 'batch')
%         u1p_values = py.numpy.array(min(sqrt(contrastEst * 1e8), 0.5) *rand(estimator.NumImgPair, DM.activeActNum));
        u1p_values = py.numpy.array(u');
        temp = py.sensing.optimal_probe11(estimator.sensor, u1p_values, camera.exposure, contrastEst, ...
                                estimator.rate, estimator.beta, estimator.sgd_itr);
        u = double(temp);
        u = u';
%         temp = load('opt_probe.mat');
%         opt_probe = temp.opt_probe;
%         u = sqrt(probeContrast/opt_probe.contrast) * [opt_probe.up1, opt_probe.up2];
    elseif estimator.EKFpairProbing && strcmpi(estimator.type, 'kalman')
%         u1p_values = py.numpy.array(u');
        u1p_values = py.numpy.array(min(sqrt(contrastEst * 1e8), 0.5) *rand(estimator.NumImgPair, DM.activeActNum));
%         temp = py.sensing.optimal_probe5(estimator.sensor, E_old_real, E_old_imag, P_old, ...
%                                 u1c_values, u2c_values, u1p_values, camera.exposure, contrastEst, ...
%                                 estimator.rate, estimator.sgd_itr);
%         temp = py.sensing.optimal_probe6(estimator.sensor, E_old_real, E_old_imag, P_old, ...
%                                 u1c_values, u2c_values, u1p_values, camera.exposure, contrastEst, ...
%                                 estimator.rate, estimator.beta, estimator.sgd_itr);
%         temp = py.sensing.optimal_probe7(estimator.sensor, E_old_real, E_old_imag, P_old, ...
%                                 u1c_values, u2c_values, u1p_values, camera.exposure, ...
%                                 estimator.rate, estimator.sgd_itr);
%         u1p_values_old = py.numpy.array(data.uProbe(:, :, data.itr-1)');
        temp = py.sensing.optimal_probe12(estimator.sensor, E_old_real, E_old_imag, P_old, ...
                                u1c_values, u2c_values, u1p_values, camera.exposure, contrastEst, ...
                                estimator.rate, estimator.beta, estimator.sgd_itr);

        u = double(temp);
        u = u';
    elseif ~estimator.EKFpairProbing && strcmpi(estimator.type, 'EKF')

        if strcmpi(estimator.whichDM, 'both')
%             u1p_values = py.numpy.array(mean(mean(abs(u)))*(rand(estimator.NumImg, DM.activeActNum)-0.5));
%             u2p_values = py.numpy.array(mean(mean(abs(u)))*(rand(estimator.NumImg, DM.activeActNum)-0.5));
%             temp = py.sensing.optimal_probe10(estimator.sensor, E_old_real, E_old_imag, P_old, ...
%                                     u1c_values, u2c_values, u1p_values, u2p_values, camera.exposure, ...
%                                     estimator.rate, estimator.sgd_itr);
%             u = [double(temp{1}), double(temp{2})];
%             u = u';
            
            u1p_values = py.numpy.array(u(1:DM.activeActNum, :)');
            u2p_values = py.numpy.array(u(DM.activeActNum+1:end, :));
            u1p_values_old = py.numpy.array(data.uProbe(1:DM.activeActNum, :, data.itr-1)');
            u2p_values_old = py.numpy.array(data.uProbe(DM.activeActNum+1:end, :, data.itr-1)');
            temp = py.sensing.optimal_probe16(estimator.sensor, E_old_real, E_old_imag, P_old, ...
                                    u1c_values, u2c_values, u1p_values, u2p_values, u1p_values_old, u2p_values_old, ...
                                    camera.exposure, estimator.rate, estimator.beta, estimator.sgd_itr);
            u = [double(temp{1}), double(temp{2})];
            u = u';            
        else
%             u1p_values = py.numpy.array(u');
            u1p_values = py.numpy.array(mean(mean(abs(u)))*(rand(estimator.NumImg, DM.activeActNum)-0.5));
%             temp = py.sensing.optimal_probe8(estimator.sensor, E_old_real, E_old_imag, P_old, ...
%                                     u1c_values, u2c_values, u1p_values, camera.exposure, ...
%                                     estimator.rate, estimator.sgd_itr);
%             temp = py.sensing.optimal_probe9(estimator.sensor, E_old_real, E_old_imag, P_old, ...
%                                     u1c_values, u2c_values, u1p_values, camera.exposure, ...
%                                     estimator.rate, estimator.beta, estimator.sgd_itr);
%             temp = py.sensing.optimal_probe9(estimator.sensor, E_old_real, E_old_imag, P_old, ...
%                                     u1c_values, u2c_values, u1p_values, camera.exposure, ...
%                                     5e-2, 100);
%             u1p_values_old = py.numpy.array(data.uProbe(:, :, data.itr-1)');
            
            temp = py.sensing.optimal_probe14(estimator.sensor, E_old_real, E_old_imag, P_old, ...
                                    u1c_values, u2c_values, u1p_values, camera.exposure, ...
                                    estimator.rate, estimator.beta, estimator.sgd_itr);
            u = double(temp);
            u = u';
            
%             cost_opt = 0;
%             gap = pi/36;
%             for offset_test = 0: gap: 2*pi-gap
%                 probeSP = probeShape(target, coronagraph, estimator, XS, YS, offset_test, probeContrast); % the desired probe shape on pupil plane in meters
%                 % Since the width of shaped pupil is different from DM, we should
%                 % adjust the probe shape to DM plane
%                 if coronagraph.SPwidth >= DM.widthDM
%                     marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
%                     marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));
%                     probeSPcrop = probeSP(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel);
%                     probeDM = imresize(probeSPcrop, DM.DMmesh);
%                 else
%                     marginWidth = (DM.widthDM - coronagraph.SPwidth)/2;
%                     marginNpixel = round(marginWidth / DM.widthDM * DM.DMmesh(1));
%                     probeSPresized = imresize(probeSP, DM.DMmesh - 2 * marginNpixel);
%                     probeDM = zeros(DM.DMmesh);
%                     probeDM(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = probeSPresized;
%                 end
%                 command = height2voltage(probeDM, DM.DMperfect, estimator.whichDM, Nitr);
%                 u = command';
%                 u1p_values = py.numpy.array(u');
%                 u1p_values_old = py.numpy.array(data.uProbe(:, :, data.itr-1)');
%                 cost_now = py.sensing.probe_cost14(estimator.sensor, E_old_real, E_old_imag, P_old, ...
%                                     u1c_values, u2c_values, u1p_values, u1p_values_old, camera.exposure, ...
%                                     estimator.rate, estimator.beta);
%                 if cost_now < cost_opt
%                     cost_opt = cost_now;
%                     u_opt = u;
%                 end
%                 disp(['offset: ', num2str(offset_test/pi*180), ' cost: ', num2str(cost_now)])
%             end
%             u = u_opt';
        end
        
    else
        disp([estimator.type, ' does not support active sensing!'])
    end
end

%% take probed images
% if ((strcmpi(estimator.type, 'EKF') || strcmpi(estimator.type, 'UKF')) && estimator.EKFpairProbing == 0)
if estimator.EKFpairProbing == 0 || strcmpi(estimator.type, 'EKF') || strcmpi(estimator.type, 'UKF')
    for k = 1 : estimator.NumImg
        % take images for positive or negative probing
        switch estimator.whichDM
            case '1'
                Iprobe = getImg(target, DM, coronagraph, camera, DM1command + u(:, k), DM2command, simOrLab);
            case '2'
                Iprobe = getImg(target, DM, coronagraph, camera, DM1command, DM2command + u(:, k), simOrLab);
            case 'both'
                Iprobe = getImg(target, DM, coronagraph, camera, DM1command + u(1:DM.activeActNum, k), DM2command + u(DM.activeActNum+1:end, k), simOrLab);
            otherwise
                disp('The DM used for probing should be either 1 or 2!');
                return;
        end
        image(:, :, k+1) = Iprobe;
        disp(['No. ', num2str(k), 'Probed image is taken.' ]);
        disp(['The averaged contrast in the dark holes of probed image is ', num2str(mean(Iprobe(darkHole.pixelIndex)))]);
    end
else
    %%
    for k = 1 : estimator.NumImgPair
        % take images for positive or negative probing
        switch estimator.whichDM
            case '1'
                Iplus = getImg(target, DM, coronagraph, camera, DM1command + u(:, k), DM2command, simOrLab);
                Iminus = getImg(target, DM, coronagraph, camera, DM1command - u(:, k), DM2command, simOrLab);
            case '2'
                Iplus = getImg(target, DM, coronagraph, camera, DM1command, DM2command + u(:, k), simOrLab);
                Iminus = getImg(target, DM, coronagraph, camera, DM1command, DM2command - u(:, k), simOrLab);                
            otherwise
                disp('The DM used for probing should be either 1 or 2!');
                return;
        end
        image(:, :, 2 * k) = Iplus;
        image(:, :, 2 * k + 1) = Iminus;
        disp(['No. ', num2str(k), ' pair of probed images is taken.' ]);
        disp(['The averaged contrast in the dark holes of positive probed image is ', num2str(mean(Iplus(darkHole.pixelIndex)))]);
        disp(['The averaged contrast in the dark holes of negative probed image is ', num2str(mean(Iminus(darkHole.pixelIndex)))]);
    end
end
end


%% subfunctions used before

function surf = probeShape(target, coronagraph, estimator, XS, YS, offset, probeContrast, probeAxis)
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
elseif strcmpi(coronagraph.type, 'SPC')
    surf = SincAmp * sinc(mx * (XS)) .* sinc(my * (YS)) .* ... 
        cos(2 * pi * wx / coronagraph.SPwidth * XS + offset) .* cos(2 * pi * wy / coronagraph.SPwidth * YS);
elseif strcmpi(coronagraph.type, 'VORTEX')
    if probeAxis == 1
        surf = SincAmp * sinc(mx * (XS)) .* sinc(my * (YS)) .* ... 
            cos(2 * pi * wx / coronagraph.SPwidth * XS + offset) .* cos(2 * pi * wy / coronagraph.SPwidth * YS);
    else
        surf = SincAmp * sinc(mx * (YS)) .* sinc(my * (XS)) .* ... 
        cos(2 * pi * wx / coronagraph.SPwidth * YS + offset) .* cos(2 * pi * wy / coronagraph.SPwidth * XS);
    end
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
    case 'both'
        gain = DM.DM1gain;
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