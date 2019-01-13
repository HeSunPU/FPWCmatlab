function [system, stateEst]= onlineLearning(u, y, G, Q, R, x0, P0, uProbe, Nitr, delta1, delta2)
%% Online coronagraphic imaging system learning using E-M algorithm
% Developed by He Sun on Jun. 17, 2017
%
% This function is used to recover the system with the structure:
%       x_k = x_k-1 + G u_k + w_k
%       y_k = H x_k + n_k
%
% E-M algorithm, including Kalman filter, Kalman smoothing and gradient ascent maximum
% likelyhood estimation (MLE) is applied here. It is revised from the method in
% the paper "Parameter Estimation for Linear Dynamical Systems, Z. Ghahramani, G. E. Hinton, 1996"
%
% This algorithm can only be used for the linear state space model I list
% above, the Jacobian matrix, G, observation matrix, H, are fixed. The
% process covariance matrix, E(w_k * w_k^T) = Q, and the observation
% covariance matrix, E(n_k * n_k^T) = R, should also have some certain
% structure (scalar matrix). In this implementation, we assume a specific 
% relationship between G and H, H = 4 * (G * uProbe)'.
%
% u - the control inputs of training data set
% y - the observations of training data set
% G - the initial value of control Jacobian matrix
% Q - the initial value of process covariance matrix
% R - the initial value of observation noise covariance matrix
% x0 - the initial guess of starting state
% P0 - the covariance matrix of the starting state
% Nitr - the number of E-M iterations
% uProbe - the probe shape for pair-wise observation
% delta1 - online learning rate of G
% delta2 - online learning rate of H

%% pull the data and assert the dimension is correct
nState = size(G, 1); % dimension of state
nControl = size(G, 2); % dimension of control command
nObservation = size(y, 1); % dimension of observations
nStep = size(y, 2); % number of steps
assert(size(u, 1) == nControl, 'The dimension of initial Jacobian matrix and the control inputs do not match!');
assert(size(u, 2) == nStep, 'The steps of control inputs and observations do not match!');
assert(size(y, 1) == nObservation, 'The dimension fo initial observation matrix and control inputs do not match!');
assert(size(Q, 1) == nState, 'The dimension of states and initial process variance matrix do not match!');
assert(size(Q, 2) == nState, 'The dimension of states and initial process variance matrix do not match!');
assert(size(R, 1) == nObservation, 'The dimension of states and initial observation variance matrix do not match!');
assert(size(R, 2) == nObservation, 'The dimension of states and initial observation variance matrix do not match!');
assert(size(P0, 1) == nState, 'The dimension of states and initial state variance matrix do not match!');
assert(size(P0, 2) == nState, 'The dimension of states and initial state variance matrix do not match!');
assert(length(x0) == nState, 'The dimension of states and initial state do not match!'); % assert the dimensions of input vectors match

%% define the error term
% errorJacobian = zeros(Nitr + 1, 1);
% errorJacobian(1) = sum(sum((G - GTrue).^2));
% errorEfocal = zeros(Nitr, 1);
% errorObservationMat = zeros(Nitr + 1, 1);
% errorObservationMat(1) = sum(sum((H - HTrue).^2)); % error terms above only used in simulation
% Hinv = pinv(H);
% for itr = 1 : Nitr
%     xLeastSquare = Hinv * y;
% end
% errorObservation = zeros(Nitr, 1);
% errorJacobianDiff = zeros(Nitr, 1);
% errorStateDiff = zeros(Nitr, 1);
% errorJacobAndObservMat = zeros(Nitr+1, 1);
% errorJacobAndObservMat(1) = sum(sum((H - (G(:, 1:952) * uProbe)').^2));
% errorObservation(1) = sum(sum((y - H * xLeastSquare).^2));

%% the summation of uuT is used for update Jacobian matrix, however, since 
% it is very computattional expensive, so that we calculate first
sumUUT = zeros(nControl, nControl);
for k = 1 : nStep
    sumUUT = sumUUT + (u(:, k) * u(:, k)') / max(u(:, k)' * u(:, k), 10);
end
%% the E-M iterations start
for itr = 1 : Nitr
    %% E-Step - Forward propagation: Kalman filter esitmation of the state variables
    xPriori = zeros(nState, nStep);
    PPriori = zeros(nState, nState, nStep);
    xPosteriori = zeros(nState, nStep);
    PPosteriori = zeros(nState, nState, nStep); % define the variables
    xOld = x0;
    POld = P0;
    for k = 1 : nStep
        H = 4 * (G * uProbe(:, :, k))';
%         H = 4 * (G * uProbe)';
        % compute the priori variables
        xPriori(:, k) = xOld + G * u(:, k); % priori prediction of state variables
        PPriori(:, :, k) = POld + max(u(:, k)' * u(:, k), 10) * Q; % priori prediction of state covariance matrix
        % compute the posteriori variables
        residual = y(:, k) - H * xPriori(:, k); % residue between true measurements and model prediction
        S = H * PPriori(:, :, k) * H' + R; % observation estimation covariance matrix
        K = PPriori(:, :, k) * H' * S^(-1); % Kalman gain
        xPosteriori(:, k) = xPriori(:, k) + K * residual; % posteriori estimate of state variables
        PPosteriori(:, :, k) = (eye(nState) - K * H) * PPriori(:, :, k); % posteriori estimate of state covariance matrix
        xOld = xPosteriori(:, k);
        POld =  PPosteriori(:, :, k); % save the data for the use of next step
    end

    %% E-Step - Backward propagation: Rauch smoothing
    xSmoothed = zeros(nState, nStep);
    PSmoothed = zeros(nState, nState, nStep);
    xSmoothed(:, nStep) = xPosteriori(:, nStep);
    PSmoothed(:, :, nStep) = PPosteriori(:, :, nStep); % define the variables
    for k = nStep-1 : -1 : 1
        C = PPosteriori(:, :, k) * PPriori(:, :, k+1)^(-1);
        xSmoothed(:, k) = xPosteriori(:, k) + C * (xSmoothed(:, k+1) - xPriori(:, k+1));
        PSmoothed(:, :, k) = PPosteriori(:, :, k) + C * (PSmoothed(:, :, k+1) - PPriori(:, :, k+1)) * C';
    end
    C = P0 * PPriori(:, :, 1)^(-1);
    x0new = x0 + C * (xSmoothed(:, 1) - xPriori(:, 1));
    P0new = P0 + C * (PSmoothed(:, :, 1) - PPriori(:, :, 1)) * C';
    
    %% M-Step - Maximum likelyhood estimation of model
    % compute the update from the state transition
    sumXUT = zeros(nState, nControl);
    for k = 1 : nStep
        if k == 1
            sumXUT = sumXUT + (xSmoothed(:, k) - x0) * u(:, k)' / max(u(:, k)' * u(:, k), 10);
        else
            sumXUT = sumXUT + (xSmoothed(:, k) - xSmoothed(:, k-1)) * u(:, k)' / max(u(:, k)' * u(:, k), 10);
        end
    end
    % compute the update from the observation
    sumXYTUT = zeros(nState, nControl);
    sumXXTGUUT = zeros(nState, nControl);
    for k = 1 : nStep
        sumXYTUT = sumXYTUT + xSmoothed(:, k) * y(:, k)' * uProbe(:, :, k)';
        sumXXTGUUT = sumXXTGUUT + (xSmoothed(:, k) * xSmoothed(:, k)' + PSmoothed(:, :, k)) * G * (uProbe(:, :, k) * uProbe(:, :, k)');
%         sumXYTUT = sumXYTUT + xSmoothed(:, k) * y(:, k)' * uProbe';
%         sumXXTGUUT = sumXXTGUUT + (xSmoothed(:, k) * xSmoothed(:, k)' + PSmoothed(:, :, k)) * G * (uProbe * uProbe');
    end
    
    for updateItr = 1 : 100
        Gnew = G + delta1 * ((sumXUT - G * sumUUT) + delta2 * (4 * sumXYTUT - 16 * sumXXTGUUT));
        G = Gnew;
    end
    
    % update the observation noise covariance matrix
    sumR = zeros(nObservation, nObservation);
    for k = 1 : nStep
        Hnew = 4 * (Gnew * uProbe(:, :, k))';
%         Hnew = 4 * (Gnew * uProbe)';
        sumR = sumR + (y(:, k) - Hnew * xSmoothed(:, k)) * (y(:, k) - Hnew * xSmoothed(:, k))' + Hnew * PSmoothed(:, :, k) * Hnew';
    end
    Rnew = sumR / nStep;
    Rnew = trace(Rnew) / nObservation * eye(nObservation);
    
    % update the process noise covariance matrix
    sumQ = zeros(nState, nState);
    for k = 1 : nStep
        if k == 1
            sumQ = sumQ + ((xSmoothed(:, k)-x0new-Gnew * u(:, k))* (xSmoothed(:, k)-x0new-Gnew * u(:, k))' + ...
                PSmoothed(:, :, 1) + P0new) / max(u(:, k)' * u(:, k), 10);
        else
            sumQ = sumQ + ((xSmoothed(:, k)-xSmoothed(:, k-1)-Gnew * u(:, k))* (xSmoothed(:, k)-xSmoothed(:, k-1)-Gnew * u(:, k))' + ...
                PSmoothed(:, :, k) + PSmoothed(:, :, k-1)) / max(u(:, k)' * u(:, k), 10);
        end
    end
    Qnew = sumQ / nStep;
    Qnew = trace(Qnew) / nState * eye(nState);
%%    
    % plot the errors
%     errorJacobian(itr + 1) = sum(sum((G - GTrue).^2));
%     figure(1), semilogy(0:itr, errorJacobian(1:itr+1), '-o','MarkerSize',7,'LineWidth',2);
%     xlabel('Iteration','FontSize',16,'Interpreter','LaTeX'); 
%     ylabel('Jacobian Error','FontSize',16,'Interpreter','LaTeX');
%     set(gca,'FontSize',16,'FontName','Times','FontWeight','Normal');
%     drawnow
%     
%     errorEfocal(itr) = sum(sum((xSmoothed - [real(data.Efocal(index, :)); imag(data.Efocal(index, :))]).^2));
%     figure(2), semilogy(0:itr-1, errorEfocal(1:itr), '-o','MarkerSize',7,'LineWidth',2);
%     xlabel('Iteration','FontSize',16,'Interpreter','LaTeX');
%     ylabel('Efield Error','FontSize',16,'Interpreter','LaTeX');
%     set(gca,'FontSize',16,'FontName','Times','FontWeight','Normal');
%     drawnow
%     
%     errorObservationMat(itr + 1) = sum(sum((H - HTrue).^2));
%     figure(3), semilogy(0:itr, errorObservationMat(1:itr+1), '-o','MarkerSize',7,'LineWidth',2);
%     xlabel('Iteration','FontSize',16,'Interpreter','LaTeX');
%     ylabel('Observation matrix Error','FontSize',16,'Interpreter','LaTeX');
%     set(gca,'FontSize',16,'FontName','Times','FontWeight','Normal');
%     drawnow
    
%     errorObservation(itr+1) = sum(sum((y - H * xSmoothed).^2));
%     figure(4), semilogy(0:itr, errorObservation(1:itr+1), '-o','MarkerSize',7,'LineWidth',2);
%     xlabel('Iteration','FontSize',16,'Interpreter','LaTeX');
%     ylabel('Training Error','FontSize',16,'Interpreter','LaTeX');
%     set(gca,'FontSize',16,'FontName','Times','FontWeight','Normal');
%     drawnow
%     
%     errorJacobianDiff(itr) = sum(sum((G - Gnew).^2));
%     figure(5), semilogy(1:itr, errorJacobianDiff(1:itr), '-o','MarkerSize',7,'LineWidth',2);
%     xlabel('Iteration','FontSize',16,'Interpreter','LaTeX');
%     ylabel('change of Jacobian','FontSize',16,'Interpreter','LaTeX');
%     set(gca,'FontSize',16,'FontName','Times','FontWeight','Normal');
%     drawnow
%     
%     if itr >= 2
%         errorStateDiff(itr) = sum(sum((xSmoothed - xSmoothedOld).^2));
%         figure(6), semilogy(2:itr, errorStateDiff(2:itr), '-o','MarkerSize',7,'LineWidth',2);
%         xlabel('Iteration','FontSize',16,'Interpreter','LaTeX');
%         ylabel('Change of State estimate','FontSize',16,'Interpreter','LaTeX');
%         set(gca,'FontSize',16,'FontName','Times','FontWeight','Normal');
%         drawnow
%     end
%     
%     errorJacobAndObservMat(itr+1) = sum(sum((Hnew - (4 * Gnew(:, 1:952) * uProbe)').^2));
%     figure(7), semilogy(0:itr, errorJacobAndObservMat(1:itr+1), '-o', 'MarkerSize', 7, 'LineWidth', 2);
%     xlabel('Iteration','FontSize',16,'Interpreter','LaTeX');
%     ylabel('caculated - learned observ mat','FontSize',16,'Interpreter','LaTeX');
%     set(gca,'FontSize',16,'FontName','Times','FontWeight','Normal');
%     drawnow
    
    % save the update for the use of next iteration
    G = Gnew;
    Q = Qnew;
    R = Rnew;
    x0 = x0new;
    P0 = P0new;
end

%% save the outputs
system.G = G;
system.Q = Q;
system.R = R;
system.x0 = x0;
system.P0 = P0;
stateEst.x = xSmoothed;
stateEst.P = PSmoothed;
end
