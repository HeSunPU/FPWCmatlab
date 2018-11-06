function command = LPcontroller(x, G, varargin)
% The linear programming controller
% Developed by He Sun on Mar. 6, 2017
%
% command - the control command of the DM
% G - the overall control Jacobian matrix
% alpha - the Tikhonov regularization parameter
% varargin - the input that defines the mode of the linear controller,
%'strokeMin' or 'energyMin', and the bound
%
assert(length(varargin) <= 2, 'You give more inputs than needed.'); % function have at most two inputs
mode = varargin{1}; % mode - 'strokeMin' or 'energyMin'

% make sure the input field and Jacobian matrix have right dimensions
Nact = size(G, 2);
Ngroup = size(G, 1);
assert(length(x) == Ngroup, 'The dimension of G and x are not consistent.');

switch lower(mode) % select mode, 'strokeMin' or 'energyMin'
    case lower('cvxEnergyMin')
        cvx_begin
            voltageBound = 2;
            variables maxContrast u(Nact)
            minimize (maxContrast)
            subject to
                u >= -voltageBound
                u <= voltageBound
                x + G * u <= maxContrast
                x + G * u >= -maxContrast
        cvx_end
        command = u;
    case lower('cvxSOSstrokemin')
        cvx_begin
            voltageBound = 5;
            contrastBound = varargin{2};
            variables u(Nact)
            minimize (u'*u)
            subject to
                u >= -voltageBound
                u <= voltageBound
                x + G * u <= contrastBound
                x + G * u >= -contrastBound
        cvx_end
        command = u;
    case 'sosstrokemin'
        % min J = sum(|command|)
        %   s.t. |x + G * command| < maxContrast
        %   
        contrastBound = varargin{2};
        Q = eye(Nact);
        f = zeros(Nact, 1);
        A = [G; -G];
        b = [contrastBound - x;
            contrastBound + x];
        voltageBound = 5;
        lb = -voltageBound * ones(Nact, 1);
        ub = voltageBound * ones(Nact, 1);
        % use gurobi to calculate optimization problem
        % minimize      x^T * Q * x + f^T * x
        % subject to    A * x < b
        clear model;
        model.modelname = 'SOSstrokeMin';
        model.modelsense = 'min';
        model.Q = sparse(Q);
        model.obj = f;
        model.A = sparse(A);
        model.sense = '<';
        model.rhs = b;
        model.lb = lb;
        model.ub = ub;
        
        clear params;
        params.Method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex,
                           % 2=barrier, 3=concurrent, 4=deterministic
                           % concurrent
        params.outputflag = 0;
        result = gurobi(model, params);
        if (strcmpi(result.status, 'optimal'))
            command = result.x;
        else disp(['The optimization problem is ', result.status]);
        end
    case 'relaxedsosstrokemin'
        % min J = sum(|command|)
        %   s.t. |x + G * command| < maxContrast
        %   
        contrastBound = varargin{2};
        Q = [eye(Nact), zeros(Nact, Ngroup); zeros(Ngroup, Nact+Ngroup)];
        Q = zeros(size(Q));
        alpha = 1;
        f = [zeros(Nact, 1); alpha * zeros(Ngroup, 1)];
        A = [G, -eye(Ngroup); -G, -eye(Ngroup)];
        b = [contrastBound - x;
            contrastBound + x];
        voltageBound = 1;
        lb = [-voltageBound * ones(Nact, 1); zeros(Ngroup, 1)];
        ub = [voltageBound * ones(Nact, 1); 0.5 * contrastBound];
        % use gurobi to calculate optimization problem
        % minimize      x^T * Q * x + f^T * x
        % subject to    A * x < b
        clear model;
        model.modelname = 'relaxedSOSstrokeMin';
        model.modelsense = 'min';
        model.Q = sparse(Q);
        model.obj = f;
        model.A = sparse(A);
        model.sense = '<';
        model.rhs = b;
        model.lb = lb;
        model.ub = ub;
        
        clear params;
        params.Method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex,
                           % 2=barrier, 3=concurrent, 4=deterministic
                           % concurrent
        params.outputflag = 0;
        result = gurobi(model, params);
        if (strcmpi(result.status, 'optimal'))
            command = result.x(1:Nact);
        else disp(['The optimization problem is ', result.status]);
        end
    case 'strokemin'
        % min J = sum(|command|)
        %   s.t. |x + G * command| < maxContrast
        %   
        contrastBound = varargin{2};
        f = [ones(Nact, 1); zeros(Nact, 1)];
        A = [zeros(Ngroup, Nact), G;
            zeros(Ngroup, Nact), -G;
            -eye(Nact), -eye(Nact);
            -eye(Nact), eye(Nact)];
        b = [contrastBound - x;
            contrastBound + x;
            zeros(2 * Nact, 1)];
        voltageBound = 10;
        lb = ones(2 * Nact, 1) * voltageBound;
        ub = ones(2 * Nact, 1) * voltageBound;
        
        % use gurobi to calculate optimization problem
        % minimize      f^T * x
        % subject to    A * x < b
        %
        clear model;
        model.modelname = 'LPstrokeMin';
        model.modelsense = 'min';
        model.obj = f;
        model.A = sparse(A);
        model.sense = '<';
        model.rhs = b; 
        model.lb = lb;
        model.ub = ub;
        
        clear params;
        params.Method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex,
                           % 2=barrier, 3=concurrent, 4=deterministic
                           % concurrent
        params.outputflag = 0;
        result = gurobi(model, params);
        if (strcmpi(result.status, 'optimal'))
            command = result.x(Nact + 1 : end);
        else disp(['The optimization problem is ', result.status]);
        end
    case 'maxstrokemin'
        % min J = max(|command|)
        %   s.t. |x + G * command| < maxContrast
        %   
        contrastBound = varargin{2};
        f = [1; zeros(Nact, 1)];
        A = [zeros(Ngroup, 1), G;
            zeros(Ngroup, 1), -G;
            -ones(Nact, 1), -eye(Nact);
            -ones(Nact, 1), eye(Nact)];
        b = [contrastBound - x;
            contrastBound + x;
            zeros(2 * Nact, 1)];
%         voltageBound = 1;
%         lb = ones(2 * Nact, 1) * voltageBound;
%         ub = ones(2 * Nact, 1) * voltageBound;
%         sigma = 3e-18;
        % use gurobi to calculate optimization problem
        % minimize      f^T * x
        % subject to    A * x < b
        %
        clear model;
        model.modelname = 'LPstrokeMin';
        model.modelsense = 'min';
        model.obj = f;
        model.A = sparse(A);
        model.sense = '<';
        model.rhs = b; 
%         model.lb = lb;
%         model.ub = ub;
%         model.quadcon.Qc = sparse([zeros(Nact), zeros(Nact); zeros(Nact), G' * G]);
%         model.quadcon.q = [zeros(Nact, 1); 2 * G' * x];
%         model.quadcon.rhs = 0;
        
        clear params;
        params.Method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex,
                           % 2=barrier, 3=concurrent, 4=deterministic
                           % concurrent
        params.outputflag = 0;
        result = gurobi(model, params);
        if (strcmpi(result.status, 'optimal'))
            command = result.x(2 : end);
        else disp(['The optimization problem is ', result.status]);
        end
    case 'energymin'
        voltageBound = 5;
        f = [1; zeros(Nact, 1)];
        A = [-ones(Ngroup, 1), G;
            -ones(Ngroup, 1), -G];
        b = [-x; x;];
        lb = [0; -ones(Nact, 1) * voltageBound];
        ub = [inf; ones(Nact, 1) * voltageBound];
        
        % use gurobi to calculate optimization problem
        % minimize      f^T * x
        % subject to    A * x < b
        %               lb < x < ub
        %
        clear model;
        model.modelname = 'LPenergyMin';
        model.modelsense = 'min';
        model.obj = f;
        model.A = sparse(A);
        model.sense = '<';
        model.rhs = b;
        model.lb = lb;
        model.ub = ub;
        
        clear params;
        params.Method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex,
                           % 2=barrier, 3=concurrent, 4=deterministic
                           % concurrent
        params.outputflag = 0;
        result = gurobi(model, params);
        if (strcmpi(result.status, 'optimal'))
            command = result.x(2 : end);
        else disp(['The optimization problem is ', result.status]);
        end
end


end