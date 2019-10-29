function [EfocalEst, IincoEst, data] = EKF2b(u, image, darkHole, model, estimator, controller, data, kWavelength)
%% The pixel-wise extended Kalman filter estimator with no non-probe image
% Developed by He Sun on Feb. 24, 2017
% Revised by Susan Redmond July 23, 2019
% No probing images taken or used in this revised version
%
% EfocalEst - the estimated coherent focal plane electric field
% IincoEst - the estimated incoherent focal plane intensity
% u - 0 since no probing happens here
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
% sfr added case 
switch lower(estimator.type)
    case 'ekf_speckle'
        assert(size(image, 3) == estimator.NumImg, 'Not giving correct number of images!');        
    otherwise
        assert(size(image, 3) == 1 + estimator.NumImg, 'Not giving correct number of images!');
end
%% redefine the observation noise coefficient according to exposure noise
if estimator.adaptive_exposure
    estimator.observationVarCoefficient = estimator.observationVarCoefficient0 / data.probe_exposure(data.itr)^2;
end
%% Extract the images
switch lower(estimator.type) %added by sfr
    case 'ekf_speckle'
        Iobserv = zeros(estimator.NumImg, darkHole.pixelNum);
        for k = 1 : estimator.NumImg
            Iobserv2D = image(:, :, k);
            Iobserv(k, :) = Iobserv2D(darkHole.pixelIndex);
%             Iobserv2D = zeros(size(image));%(:, :, k);
%             Iobserv(k, :) = Iobserv2D(darkHole.pixelIndex);
        end
    otherwise
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
% probe = transpose(G * u(:, k)); % u = 0 so this will be zero

%% combine the phase information from model and amplitude information from
% measurement

EfocalEst = zeros(darkHole.pixelNum, 1);
IincoEst = zeros(darkHole.pixelNum, 1);

% generate the control Jacobian and control command used according to
% the DM we use for control
if data.itr == 1
    if strcmpi(estimator.type, 'ekf_speckle') == 0
        command = data.DMcommand(:, 1);
    else
        command = data.DMcommand(:, 1) - data.DHcommand;
    end
else
    command = data.DMcommand(:, data.itr) - data.DMcommand(:, data.itr - 1);
end
switch controller.whichDM
    case '1'
        G = model.G1;
        command = command(1 : length(command)/2);
    case '2'
        G = model.G2;
        command = command(length(command)/2 + 1: length(command)); % SUSAN CHANGED THIS command = command(length(command)/2 + 1, length(command));
    case 'both'
        G = [model.G1, model.G2];
    otherwise
        disp('You can only use the first DM, second DM or both for wavefront control.');
        return;
end
% generate the covariance matrices

temp = zeros(estimator.NumImg);
R = estimator.observationVarCoefficient * eye(estimator.NumImg) + temp;

Q = zeros(3,3);
Q(1:2, 1:2) = (sum(command.^2) * estimator.processVarCoefficient + estimator.processVarCoefficient2) * eye(2); % for simulation
% Kalman filter for each pixel in the dark holes
for q = 1 : darkHole.pixelNum
    if kWavelength == 0
        if data.itr == 1
%             Q(3, 3) = max(estimator.incoherentStd^2, 0.1 * data.IincoEst0(q)^2);
            Q(3, 3) = max(estimator.incoherentStd^2, 0.1 * mean(data.IincoEst0.^2));
            xOld = [real(data.EfocalEst0(q)); imag(data.EfocalEst0(q)); data.IincoEst0(q)];
        else
    %         Q(3, 3) = max(estimator.incoherentStd^2, 0.1 * data.IincoEst(q, data.itr-1)^2);
            Q(3, 3) = max(estimator.incoherentStd^2, 0.1 * mean(data.IincoEst(:, data.itr-1).^2));
            xOld = [real(data.EfocalEst(q, data.itr-1)); imag(data.EfocalEst(q, data.itr-1)); data.IincoEst(q, data.itr-1)];
        end
    else
        if data.itr == 1
%             Q(3, 3) = max(estimator.incoherentStd^2, 0.1 * data.IincoEst0(q, kWavelength)^2);
            Q(3, 3) = max(estimator.incoherentStd^2, 0.3 * mean(data.IincoEst0.^2));
            xOld = [real(data.EfocalEst0(q, kWavelength)); imag(data.EfocalEst0(q, kWavelength)); data.IincoEst0(q, kWavelength)];
        else
    %         Q(3, 3) = max(estimator.incoherentStd^2, 0.1 * data.IincoEst(q, data.itr-1)^2);
            Q(3, 3) = max(estimator.incoherentStd^2, 0.3 * mean(data.IincoEst(:, kWavelength, data.itr-1).^2));
            xOld = [real(data.EfocalEst(q, kWavelength, data.itr-1)); imag(data.EfocalEst(q, kWavelength, data.itr-1)); data.IincoEst(q, kWavelength, data.itr-1)];
        end
    end
%     Q(3,3)
%     Q(3,3) = 10^-18; %SUSAN ADDED ***************
    update = [[real(G(q, :)); imag(G(q, :))] * command; 0]; % the linear update of the state
    y = Iobserv(:, q);
    % Kalman filter estimation officially starts here
    xPriori = xOld + update; % predict a priori state estimate
    % Probe = 0 here!
    H = [2 * (xPriori(1)), 2 * (xPriori(2)),...
        ones(estimator.NumImg, 1)]; % the observation matrix is based on the priori state estimate
    if data.itr == 1
        temp = zeros(3, 3);
        temp(1, 1) = estimator.stateStd0;
        temp(2, 2) = estimator.stateStd0;
        temp(3, 3) = estimator.incoherentStd0;
        Ppriori = temp + Q;
    else
        if kWavelength == 0
            Ppriori = data.P(:, :, q, data.itr-1) + Q; % predict a priori estimate covariance
        else
            Ppriori = data.P(:, :, q, kWavelength, data.itr-1) + Q;
        end
    end
    hPriori = zeros(estimator.NumImg, 1); % the nonlinear predict observation of the field
    for k = 1 : estimator.NumImg
        hPriori(k) = (xPriori(1))^2 + (xPriori(2))^2 + xPriori(3); %probe is zero
    end
    residual = y - hPriori; % compute the measurement residual
    S = H * Ppriori * H' + R; % residual covariance
    K = Ppriori * H' / S; % optimal Kalman gain
    xPosteriori = xPriori + K * residual; % update a posteriori state estimate
    Pposteriori = (eye(3) - K * H) * Ppriori; % update a posteriori estimate covariance
    % Iterate the EKF to make more accurate estimation
    for iEKF = 1 : estimator.itrEKF
        H = [2 * (xPosteriori(1)), 2 * (xPosteriori(2)), ones(estimator.NumImg, 1)]; % the new linear observation matrix
        hPosteriori = zeros(estimator.NumImg, 1); % the nonlinear predict observation of the field
        for k = 1 : estimator.NumImg
            hPosteriori(k) = (xPosteriori(1) + real(probe(k, q)))^2 + (xPosteriori(2) + imag(probe(k, q)))^2 + xPosteriori(3);
        end
        S = H * Ppriori * H' + R; % residual covariance
        K = Ppriori * H' / S; % optimal Kalman gain
        residual = y - hPosteriori - H * (xPriori - xPosteriori); % new measurement residual includes a linear difference term
        xPosteriori = xPriori + K * residual; % update a posteriori state estimate
        Pposteriori = (eye(3) - K * H) * Ppriori; % update a posteriori estimate covariance
    end
    % save the data
    EfocalEst(q) = xPosteriori(1) + 1i * xPosteriori(2);
    IincoEst(q) = xPosteriori(3);

    if kWavelength == 0
        data.P(:, :, q, data.itr) = Pposteriori;
        data.y(:, :, data.itr) = Iobserv';
    else
        data.P(:, :, q, kWavelength, data.itr) = Pposteriori;
        data.yBroadband(:, :, kWavelength, data.itr) = Iobserv';
    end
end
end