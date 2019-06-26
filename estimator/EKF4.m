function [EfocalEst, IincoEst, data] = EKF4(u, image, darkHole, model, estimator, controller, data, kWavelength)
%% The pixel-wise extended Kalman filter estimator
% Developed by He Sun on Feb. 24, 2017
%
% EfocalEst - the estimated coherent focal plane electric field
% IincoEst - the estimated incoherent focal plane intensity
% u - the DM probing commands
% image - the images used for wavefront estimation
% darkHole - defines the parameters of the dark holes
% model - the control Jacobian and other model related variables
% controller - the properties of the controller
% estimator - the properties of estimator
%
% check number of inputs
if nargin > 8
    disp('Wrong number of input parameters!!');
elseif nargin < 8
    kWavelength = 0;
end
% assert we do want to use Kalman filter estimator
% assert(strcmpi(estimator.type, 'EKF'), 'Not using pixel-wise EKF estimator now!');
% assert we give correct probimg command input
assert(size(u, 2) == estimator.NumImg, 'Not giving correct DM probing commands!');
% assert we give correct number of images
assert(size(image, 3) == 1 + estimator.NumImg, 'Not giving correct number of images!');
%% redefine the observation noise coefficient according to exposure noise
% if estimator.adaptive_exposure
    estimator.observationVarCoefficient = estimator.observationVarCoefficient0 / data.probe_exposure(data.itr)^2;
    estimator.observationVarCoefficient1 = estimator.observationVarCoefficient10 / data.probe_exposure(data.itr);
% end
%% Extract the images
InoPoke2D = image(:, :, 1);
InoPoke = InoPoke2D(darkHole.pixelIndex);
contrast_noProbe = max(mean(InoPoke), 2e-10);
Iobserv = zeros(estimator.NumImg + 1, darkHole.pixelNum);
for k = 1 : estimator.NumImg + 1
    Iobserv2D = image(:, :, k);
    Iobserv(k, :) = Iobserv2D(darkHole.pixelIndex);
end
% Compuate the influence caused by probing
switch estimator.whichDM
    case '1'
        G = model.G1;
    case '2'
        G = model.G2;
    otherwise
        disp('We only have DM 1 or 2 for probing!');
        return;
end
probe = zeros(estimator.NumImg, darkHole.pixelNum);
if estimator.linearProbe
    for k = 1 : estimator.NumImg
        probe(k, :) = transpose(G * u(:, k));
    end
else
    switch estimator.whichDM
        case '1'
            Gsq = model.G1sq;
        case '2'
            Gsq = model.G2sq;
        otherwise
            disp('We only have DM 1 or 2 for probing!');
            return;
    end
    for k = 1 : estimator.NumImg
        probe(k, :) = transpose(G * u(:, k)) + transpose(Gsq * u(:, k).^2);
    end
end
%% combine the phase information from model and amplitude information from
% measurement
if estimator.measuredAmp
    phase = atan2(imag(probe), real(probe));
    probe = amplitude .* (cos(phase) + 1i * sin(phase));
end
EfocalEst = zeros(darkHole.pixelNum, 1);
IincoEst = zeros(darkHole.pixelNum, 1);

% generate the control Jacobian and control command used according to
% the DM we use for control
if data.itr == 1
    command = data.DMcommand(:, 1);
else
    command = data.DMcommand(:, data.itr) - data.DMcommand(:, data.itr - 1);
end
switch controller.whichDM
    case '1'
        G = model.G1;
        command = command(1 : length(command)/2);
    case '2'
        G = model.G2;
        command = command(length(command)/2 + 1, length(command));
    case 'both'
        G = [model.G1, model.G2];
    otherwise
        disp('You can only use the first DM, second DM or both for wavefront control.');
        return;
end
% generate the covariance matrices
%     R = 2 * data.backgroundStd(data.itr)^2 * eye(2);
temp = zeros(estimator.NumImg+1);
temp(k, k) = (contrast_noProbe).^2 * estimator.observationVarCoefficient3 + (contrast_noProbe) * estimator.observationVarCoefficient1;
for k = 2 : estimator.NumImg+1
%     temp(k, k) = sum(u(:, k).^2)* estimator.observationVarCoefficient2;
%     temp(k, k) = mean(abs(Iobserv(k, :)).^2) * estimator.observationVarCoefficient3;
    temp(k, k) = max(mean(Iobserv(k-1, :)), 2e-10)^2 * estimator.observationVarCoefficient3 + max(mean(Iobserv(k-1, :)), 2e-10) * estimator.observationVarCoefficient1;
%     temp(k, k) = (contrast_noProbe + mean(abs(probe(k-1, :)).^2)).^2 * estimator.observationVarCoefficient3 + (contrast_noProbe + mean(abs(probe(k-1, :)).^2)) * estimator.observationVarCoefficient1;
end
% R = estimator.observationVarCoefficient * eye(estimator.NumImg+1);
R = estimator.observationVarCoefficient * eye(estimator.NumImg+1) + temp;
Q = (sum(command.^2) * estimator.processVarCoefficient + estimator.processVarCoefficient2) * eye(2); % for simulation
% Kalman filter for each pixel in the dark holes
for q = 1 : darkHole.pixelNum
    if kWavelength == 0
        if data.itr == 1
            xOld = [real(data.EfocalEst0(q)); imag(data.EfocalEst0(q))];
        else
            xOld = [real(data.EfocalEst(q, data.itr-1)); imag(data.EfocalEst(q, data.itr-1))];
        end
    else
        if data.itr == 1
            xOld = [real(data.EfocalEst0(q, kWavelength)); imag(data.EfocalEst0(q, kWavelength))];
        else
            xOld = [real(data.EfocalEst(q, kWavelength, data.itr-1)); imag(data.EfocalEst(q, kWavelength, data.itr-1))];
        end
    end
    update = [real(G(q, :)); imag(G(q, :))] * command; % the linear update of the state
    y = Iobserv(:, q);
    % Kalman filter estimation officially starts here
    xPriori = xOld + update; % predict a priori state estimate
    H = [2 * xPriori(1), 2 * xPriori(2);
        2 * (xPriori(1) + real(probe(:, q))), 2 * (xPriori(2) + imag(probe(:, q)))]; % the observation matrix is based on the priori state estimate
    if data.itr == 1
        temp = estimator.stateStd0 * eye(2);
        Ppriori = temp + Q;
    else
        if kWavelength == 0
            Ppriori = data.P(:, :, q, data.itr-1) + Q; % predict a priori estimate covariance
        else
            Ppriori = data.P(:, :, q, kWavelength, data.itr-1) + Q;
        end
    end
    hPriori = zeros(estimator.NumImg + 1, 1); % the nonlinear predict observation of the field
    hPriori(1) = xPriori(1)^2 + xPriori(2)^2;
    for k = 1 : estimator.NumImg
        hPriori(k + 1) = (xPriori(1) + real(probe(k, q)))^2 + (xPriori(2) + imag(probe(k, q)))^2;
    end
    residual = y - hPriori; % compute the measurement residual
    S = H * Ppriori * H' + R; % residual covariance
    K = Ppriori * H' / S; % optimal Kalman gain
    xPosteriori = xPriori + K * residual; % update a posteriori state estimate
    Pposteriori = (eye(2) - K * H) * Ppriori; % update a posteriori estimate covariance
    % Iterate the EKF to make more accurate estimation
    for iEKF = 1 : estimator.itrEKF
        H = [2 * xPosteriori(1), 2 * xPosteriori(2);
        2 * (xPosteriori(1) + real(probe(:, q))), 2 * (xPosteriori(2) + imag(probe(:, q)))]; % the new linear observation matrix
        hPosteriori = zeros(estimator.NumImg + 1, 1); % the nonlinear predict observation of the field
        hPosteriori(1) = xPosteriori(1)^2 + xPosteriori(2)^2;
        for k = 1 : estimator.NumImg
            hPosteriori(k + 1) = (xPosteriori(1) + real(probe(k, q)))^2 + (xPosteriori(2) + imag(probe(k, q)))^2;
        end
        S = H * Ppriori * H' + R; % residual covariance
        K = Ppriori * H' / S; % optimal Kalman gain
        residual = y - hPosteriori - H * (xPriori - xPosteriori); % new measurement residual includes a linear difference term
        xPosteriori = xPriori + K * residual; % update a posteriori state estimate
        Pposteriori = (eye(2) - K * H) * Ppriori; % update a posteriori estimate covariance
    end
    % save the data
    EfocalEst(q) = xPosteriori(1) + 1i * xPosteriori(2);
    IincoEst(q) = InoPoke(q) - sum(xPosteriori.^2);
    if kWavelength == 0
        data.P(:, :, q, data.itr) = Pposteriori;
        data.y(:, :, data.itr) = Iobserv';
    else
        data.P(:, :, q, kWavelength, data.itr) = Pposteriori;
        data.yBroadband(:, :, kWavelength, data.itr) = Iobserv';
    end
end
end