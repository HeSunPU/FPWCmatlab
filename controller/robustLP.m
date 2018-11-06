function command = robustLP(x, G, deltaG, P)
% The robust linear programming controller
% Developed by He Sun on May. 23, 2017
%
% command - the control command of the DM
% G - the overall control Jacobian matrix
% deltaG - the uncertainty in the Jacobian matrix
% P - the covariance matrices of state estimate
%%
assert(size(G, 1) == length(x), 'The dimensions of Jacobian matrix and state variable do not match!');
% assert(length(x) == size(P, 3), 'The dimensions of the states and covariance matrices do not match!'); % check the dimensions of the inputs

Nact = size(G, 2);
Npix = size(G, 1);
maxVoltage = 2;

%% the optimization problem with elliptical uncertainty
cvx_begin
    cvx_precision low
    cvx_solver SDPT3
    variables maxContrast uc(Nact)
    minimize (maxContrast)
    subject to
        real(x) + real(G) * uc + norm(deltaG * uc, 1) <= maxContrast
        real(x) + real(G) * uc - norm(deltaG * uc, 1) >= -maxContrast
        imag(x) + imag(G) * uc + norm(deltaG * uc, 1) <= maxContrast
        imag(x) + imag(G) * uc - norm(deltaG * uc, 1) >= -maxContrast
%         maxContrast >= 1e-4
        maxContrast >= 4 * max([sqrt(squeeze(P(1, 1, :))); sqrt(squeeze(P(2, 2, :)))])
        uc <= maxVoltage
        uc >= -maxVoltage
cvx_end
command = uc;

end