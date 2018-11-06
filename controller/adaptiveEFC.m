function controller = adaptiveEFC(x, G, target, DM, coronagraph, camera, darkHole, controller, DM1command, DM2command, simOrLab)
% The electric field conjugation (EFC) that select the best regularization
% parameter
% Developed by He Sun on Feb. 27, 2017
%
% command - the control command of the DM
% G - the overall control Jacobian matrix

alpha = 10.^(-10:-0.5:-8);
contrast = zeros(length(alpha), 1);
DM1command0 = DM1command;
DM2command0 = DM2command;
for itr = 1 : length(alpha)
    command = EFC(x, G, alpha(itr));
    switch controller.whichDM
        case '1'
            DM1command = DM1command0 + command;
        case '2'
            DM2command = DM2command0 + command;
        case 'both'
            DM1command = DM1command0 + command(1:DM.activeActNum);
            DM2command = DM2command0 + command(DM.activeActNum + 1 : end);
        otherwise
            disp('You can only use the first DM, second DM or both for wavefront control.');
            return;
    end
    I = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
    contrast(itr) = mean(I(darkHole.pixelIndex));
    figure(5), loglog(alpha(1:itr), contrast(1:itr), '-o');
    drawnow
end
[~, I] = min(contrast);
controller.alpha = alpha(I);