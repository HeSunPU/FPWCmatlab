function [EfocalEst, IincoEst, data] = UKF(u, image, darkHole, model, estimator, controller, data)
%% The pixel-wise unscented Kalman filter estimator
% Developed by He Sun on Apr. 26, 2017
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
% assert we do want to use Kalman filter estimator
% assert(strcmpi(estimator.type, 'UKF'), 'Not using pixel-wise UKF estimator now!');
% assert we give correct probimg command input
assert(size(u, 2) == estimator.NumImg, 'Not giving correct DM probing commands!');
% assert we give correct number of images
assert(size(image, 3) == 1 + estimator.NumImg, 'Not giving correct number of images!');

% tuning parameters for UKF
alpha = 1e-3; % alpha determine the spread of the sigma points around average
kappa = 0; % the secondary scaling parameter, usually set as 0
beta = 2; % beta is used to incorporate prior knowledge of the distribution. beta = 2 is optimal for Gaussian distribution
L = 3; % the state dimension
lambda = alpha^2 * (L + kappa) - L; % the scaling parameter
weightMean = zeros(2*L+1, 1); % initialize the weight for mean
weightCov = zeros(2*L+1, 1); % initialize the weight for covariance
weightMean(1) = lambda/(L + lambda);
weightCov(1) = lambda/(L + lambda) + (1 - alpha^2 + beta);
weightMean(2:end) = 1/(2 * (L + lambda)) * ones(2*L, 1);
weightCov(2:end) = 1/(2 * (L + lambda)) * ones(2*L, 1);
Pcoef = sqrt(L + lambda);

% Extract the images
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
for k = 1 : estimator.NumImg
    probe(k, :) = transpose(G * u(:, k));
end

EfocalEst = zeros(darkHole.pixelNum, 1);
IincoEst = zeros(darkHole.pixelNum, 1);
if data.itr < estimator.startKalman % for the first several iterations, use batch process estimator
    disp('Run some iterations to start the UKF first!!');
    return;
else % otherwise, run UKF estimator
    % generate the control Jacobian and control command used according to
    % the DM we use for control
    if data.itr == 2
        command = data.DMcommand(:, 1);
    else
        command = data.DMcommand(:, data.itr-1) - data.DMcommand(:, data.itr - 2);
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
    R = 2 * estimator.observationVarCoefficient * eye(estimator.NumImg+1);
    R(1,1) = estimator.observationVarCoefficient;
    Q = zeros(3,3);
    Q(1:2, 1:2) = estimator.processVarCoefficient * eye(2);
    % Unscented Kalman filter for each pixel in the dark holes
    for q = 1 : darkHole.pixelNum
        Q(3, 3) = max(estimator.incoherentStd^2, 0.1 * data.IincoEst(q, data.itr-1)^2);%0.03 * data.IincoEst(q, data.itr-1)^2;%estimator.incoherentStd^2;%
        xOld = [real(data.EfocalEst(q, data.itr-1)); imag(data.EfocalEst(q, data.itr-1)); data.IincoEst(q, data.itr-1)];
        y = Iobserv(:, q);
        update = [[real(G(q, :)); imag(G(q, :))] * command; 0]; % the linear update of the state
        % Kalman filter estimation officially starts here
        xPriori = xOld + update; % predict a priori state estimate
        Ppriori = data.P(:, :, q, data.itr-1) + Q; % predict a priori estimate covariance
%         Psqrt = chol(Ppriori, 'lower'); % the square root of state covariance matrix from cholesky decomposition
        Psqrt = sqrtm(Ppriori); % the square root of state covariance matrix
        xsample = zeros(L, 2*L+1);
        xsample(:, 1) = xPriori;
        ysample = zeros(estimator.NumImg+1, 2 * L + 1);
        % sample the state variables based on state covariance
        for j = 1 : L
            xsample(:, 2*j) = xPriori + Pcoef * Psqrt(:, j);
            xsample(:, 2*j+1) = xPriori - Pcoef * Psqrt(:, j);
        end
        % compute the observations corresponding to the state samples
        for j = 1 : 2*L+1
            ysample(1, j) = xsample(1, j)^2 + xsample(2, j)^2 + xsample(3, j);
            ysample(2:end, j) = (xsample(1, j) + real(probe(:, q))).^2 + (xsample(2, j) + imag(probe(:, q))).^2 + xsample(3, j);
        end
        % the observation mean and covariance matrices from unscented
        % transform
        ybar = ysample * weightMean;
        Py = (ysample - repmat(ybar, [1, 2*L+1])) * diag(weightCov) * (ysample - repmat(ybar, [1, 2*L+1]))' + R;
        Pxy = (xsample - repmat(xPriori, [1, 2*L+1])) * diag(weightCov) * (ysample - repmat(ybar, [1, 2*L+1]))';
        residual = y - ybar; % compute the measurement residual
        K = Pxy / Py; % optimal Kalman gain
        xPosteriori = xPriori + K * residual; % update a posteriori state estimate
        Pposteriori = Ppriori - K * Py * K'; % update a posteriori estimate covariance
        % iterate the unscented Kalman filter to make the estimation more accurate
        yPosteriori = zeros(estimator.NumImg+1, 1); % initialize the posteriori observations
        for iUKF = 1 : estimator.itrUKF
            % compute the posteriori observations
            yPosteriori(1) = xPosteriori(1)^2 + xPosteriori(2)^2 + xPosteriori(3);
            yPosteriori(2:end) = (xPosteriori(1) + real(probe(:, q))).^2 + (xPosteriori(2) + imag(probe(:, q))).^2 + xPosteriori(3);
            xPosteriori = xPriori + K * (y - yPosteriori - Pxy' * Ppriori^(-1) * (xPriori - xPosteriori));
        end
        % save the data
        EfocalEst(q) = xPosteriori(1) + 1i * xPosteriori(2);
        IincoEst(q) = xPosteriori(3);
        data.P(:, :, q, data.itr) = Pposteriori;
    end
end
end