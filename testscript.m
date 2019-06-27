clear surf_drift  DM1command

marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));

surf_drift = zeros(DM.DMmesh);
surf_drift(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = ...
    0.2e-10*(rand([DM.DMmesh - 2 * marginNpixel,1])-0.5);

surf_drift2 = zeros(DM.DMmesh);
surf_drift2(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = ...
    0.2e-10*(rand([DM.DMmesh - 2 * marginNpixel,1])-0.5);

% imagesc(surf_drift); colorbar;

command_drift = height2voltage(surf_drift, DM, target.driftDM, 5);
command_drift2 = height2voltage(surf_drift2, DM, target.driftDM, 5);

command_voltage = zeros(DM.Nact,DM.Nact);
command_voltage(DM.activeActIndex) = command_drift;
% imagesc(command_voltage);colorbar;

DM1command = command_drift;
img = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);

DM1command_2 = command_drift2;
img0 = getImg(target, DM, coronagraph, camera, DM1command_2, DM2command, simOrLab);


plot(abs(img0(darkHole.pixelIndex)-img(darkHole.pixelIndex))./img0(darkHole.pixelIndex))
