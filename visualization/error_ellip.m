function [X, Y] = error_ellip(x, P, NP)
    alpha  = 2*pi/NP*(0:NP);
    circle = [cos(alpha);sin(alpha)];
    ns = 3; 
    C = chol(P)';
    ellip = ns*C*circle;
    X = x(1)+ellip(1,:);
    Y = x(2)+ellip(2,:);
end