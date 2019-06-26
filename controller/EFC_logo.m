function command = EFC_logo(x, logo, G, alpha)
% The electric field conjugation (EFC) controller
% Developed by He Sun on Feb. 16, 2017
%
% command - the control command of the DM
% G - the overall control Jacobian matrix
% alpha - the Tikhonov regularization parameter
command = - real((G' * G + alpha * eye(size(G, 2)))^(-1) * real(G' * (x-logo)));
end