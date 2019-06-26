function command = height2voltage(surf, DM, index, Nitr)
% Use iterated calculation to find approximate DM command for a surface
% shape
%
% Select the DM gain matrix according to the DM index
switch index
    case '1'
        gain = DM.DM1gain;
    case '2'
        gain = DM.DM2gain;
    otherwise
        disp('The DM used for probing should be either 1 or 2!');
        return;
end
% use simple gradient optimization to calculate the voltage
command2D = imresize(surf, [DM.Nact, DM.Nact]) ./ gain;
command2D = command2D .* DM.DMstop;
for k = 1 : Nitr
    currentSurf = surfaceShape(DM, command2D, index);
    surfBias = surf - currentSurf;
    command2D = command2D + imresize(surfBias, [DM.Nact, DM.Nact]);
    command2D = command2D .* DM.DMstop;
end
command = command2D(DM.activeActIndex);
end