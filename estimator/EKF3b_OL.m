function [EfocalEst, IincoEst, data] = EKF3b_OL(u, image, darkHole, model, estimator, controller, data, kWavelength)
%% The pixel-wise extended Kalman filter estimator assuming no incoherent light
% Developed by He Sun on Feb. 24, 2017
% Revised by He Sun on Jan. 23, 2019
% Revised by He Sun on Mar. 25, 2019
% Revised by SFR on Oct 9, 2018
% In this revised version, we do not use the image without DM probes
%
% EfocalEst - the estimated OPEN LOOP coherent focal plane electric field
% IincoEst - the estimated incoherent focal plane intensity
% u - 0, no probes used
% image - the dithered image used for wavefront estimation
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
% assert(size(image, 3) == 1 + estimator.NumImg, 'Not giving correct number of images!');

%SFR\
switch lower(estimator.type)
    case 'ekf_speckle'
        assert(size(image, 3) == estimator.NumImg, 'Not giving correct number of images!');        
    otherwise
        assert(size(image, 3) == 1 + estimator.NumImg, 'Not giving correct number of images!');
end
%/
%% redefine the observation noise coefficient according to exposure noise
if estimator.adaptive_exposure %sfr added if back in
    estimator.observationVarCoefficient = estimator.observationVarCoefficient0 / data.probe_exposure(data.itr)^2;
    estimator.observationVarCoefficient1 = estimator.observationVarCoefficient10 / data.probe_exposure(data.itr);
end
%% Extract the images
InoPoke2D = image(:, :, 1);
InoPoke = InoPoke2D(darkHole.pixelIndex);
contrast_noProbe = max(mean(InoPoke), 2e-10);
Iobserv = zeros(estimator.NumImg, darkHole.pixelNum);

switch lower(estimator.type) %added by sfr
    case 'ekf_speckle' % set probe image to zero for speckle case
        Iobserv = zeros(estimator.NumImg, darkHole.pixelNum);
        for k = 1 : estimator.NumImg
            Iobserv2D = image(:, :, k);
            
%             Iobserv(k, :) = Iobserv2D(darkHole.pixelIndex);
%             
            % FOR CHECK ONLY, REMOVE *************************************

            if data.itr == 1
                Iobserv(k, :) = data.EfocalEst0.^2;
            else
                Iobserv(k, :) = (data.Efocaltrue(:,data.itr)).^2;
            end
            
        end
    otherwise
        for k = 1 : estimator.NumImg
            Iobserv2D = image(:, :, k+1);
            Iobserv(k, :) = Iobserv2D(darkHole.pixelIndex);
        end
end



% Compuate the influence caused by probing
switch estimator.whichDM
    case '1'
        G = model.G1;
    case '2'
        G = model.G2;
    case 'both'
        G = [model.G1, model.G2];
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
    if strcmpi(estimator.type, 'ekf_speckle') == 0
        command = data.DMcommand(:, 1);
    else
        command = data.DMcommand(:, 1) - data.DHcommand;
%         command = data.DMcommand(:, 1);

    end
else
%     command = data.DMcommand(:, data.itr) - data.DMcommand(:, data.itr - 1);
%     command = data.DMcommand(:, data.itr);
    command = data.DMcommand(:, data.itr) - data.DHcommand;
end
switch controller.whichDM
    case '1'
        G = model.G1;
        command = command(1 : length(command)/2);
    case '2'
        G = model.G2;
        command = command(length(command)/2 + 1: length(command));
    case 'both'
        G = [model.G1, model.G2];
    otherwise
        disp('You can only use the first DM, second DM or both for wavefront control.');
        return;
end
% generate the covariance matrices
%     R = 2 * data.backgroundStd(data.itr)^2 * eye(2);

temp = zeros(estimator.NumImg);
if strcmpi(estimator.type, 'ekf_speckle') == 0
    for k = 1 : estimator.NumImg
        %     temp(k, k) = sum(u(:, k).^2)* estimator.observationVarCoefficient2;
        %     temp(k, k) = mean(abs(Iobserv(k, :)).^2) * estimator.observationVarCoefficient3;
        temp(k, k) = max(mean(Iobserv(k, :)), 2e-10)^2 * estimator.observationVarCoefficient3 + max(mean(Iobserv(k, :)), 2e-10) * estimator.observationVarCoefficient1;
        %     temp(k, k) = (contrast_noProbe+mean(abs(probe(k, :)).^2)).^2 * estimator.observationVarCoefficient3 + (contrast_noProbe+mean(abs(probe(k, :)).^2)) * estimator.observationVarCoefficient1;
    end
end
% temp = mean(mean(abs(Iobserv)))^2 * estimator.observationVarCoefficient3 * eye(estimator.NumImg);
% temp = mean(mean(Iobserv))^2 * estimator.observationVarCoefficient3 * eye(estimator.NumImg);
% R = estimator.observationVarCoefficient * eye(estimator.NumImg+1);
R = estimator.observationVarCoefficient * eye(estimator.NumImg) + temp;
Q = (sum(command.^2) * estimator.processVarCoefficient + estimator.processVarCoefficient2) * eye(2); % for simulation

% figure
% plot(command)
% 

% Kalman filter for each pixel in the dark holes
for q = 1 : darkHole.pixelNum
    if kWavelength == 0
        if data.itr == 1
            xOld = [real(data.EfocalEst0(q)); imag(data.EfocalEst0(q))];
            xOld_perf = [real(data.EfocalEst0(q)); imag(data.EfocalEst0(q))];
            xOld_track(q,data.itr) = sum(xOld - xOld_perf);
%             disp(["xOld - Opt Model Output",num2str(xOld - xOld_perf)]);
        else
            xOld = [real(data.EfocalEst(q, data.itr-1)); imag(data.EfocalEst(q, data.itr-1))];
            xOld_perf = [real(data.EfocalEstOpenLoop(q, data.itr-1)); imag(data.EfocalEstOpenLoop(q, data.itr-1))];
            xOld_track(q,data.itr) = sum(xOld - xOld_perf);
%             disp(["xOld - Opt Model Output",num2str(xOld - xOld_perf)]);
        
        end
    else
        if data.itr == 1
            xOld = [real(data.EfocalEst0(q, kWavelength)); imag(data.EfocalEst0(q, kWavelength))];
        else
            xOld = [real(data.EfocalEst(q, kWavelength, data.itr-1)); imag(data.EfocalEst(q, kWavelength, data.itr-1))];
        end
    end
    update = [real(G(q, :)); imag(G(q, :))] * command; % the linear update of the state
    y = Iobserv(:, q); %Iobserv is a row vector
    % Kalman filter estimation officially starts here
    xPriori = xOld + update; % predict a priori state estimate
    
    
    
    H = [2 * (xPriori(1) + real(probe(:, q))), 2 * (xPriori(2) + imag(probe(:, q)))]; % the observation matrix is based on the priori state estimate
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
    hPriori = zeros(estimator.NumImg, 1); % the nonlinear predict observation of the field
    for k = 1 : estimator.NumImg
        hPriori(k) = (xPriori(1) + real(probe(k, q)))^2 + (xPriori(2) + imag(probe(k, q)))^2;
    end
    residual = y - hPriori; % compute the measurement residual
    S = H * Ppriori * H' + R; % residual covariance
    K = Ppriori * H' / S; % optimal Kalman gain
    
%     xPosteriori = xPriori + K * residual; % update a posteriori state estimate
    xPosteriori = xOld + K * residual; % update a posteriori state estimate

    Pposteriori = (eye(2) - K * H) * Ppriori; % update a posteriori estimate covariance
    % Iterate the EKF to make more accurate estimation (ADD THIS BACK IN
    % LATER
%     for iEKF = 1 : estimator.itrEKF
%         H = [2 * (xPosteriori(1) + real(probe(:, q))), 2 * (xPosteriori(2) + imag(probe(:, q)))]; % the new linear observation matrix
%         hPosteriori = zeros(estimator.NumImg, 1); % the nonlinear predict observation of the field
%         for k = 1 : estimator.NumImg
%             hPosteriori(k) = (xPosteriori(1) + real(probe(k, q)))^2 + (xPosteriori(2) + imag(probe(k, q)))^2;
%         end
%         S = H * Ppriori * H' + R; % residual covariance
%         K = Ppriori * H' / S; % optimal Kalman gain
%         residual = y - hPosteriori - H * (xPriori - xPosteriori); % new measurement residual includes a linear difference term
%         xPosteriori = xPriori + K * residual; % update a posteriori state estimate
%         Pposteriori = (eye(2) - K * H) * Ppriori; % update a posteriori estimate covariance
%     end
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
figure;
plot(xOld_track)
end