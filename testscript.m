clear surf_drift  DM1command
Nitr = 20;%4000; % iterations of control loop
cRange = [-8, -4]; %[-12, -3];% the range for display
simOrLab = 'simulation';%'lab';%  'simulation' or 'lab', run the wavefront correction loops in simulation or in lab
runTrial = 1;
Initialization;
DM2command = zeros(DM.activeActNum,1);


%%
def_surf_drift=0;


if def_surf_drift == 1
    marginWidth = (coronagraph.SPwidth - DM.widthDM)/2;
    marginNpixel = round(marginWidth / coronagraph.SPwidth * DM.DMmesh(1));
    
    surf_drift = zeros(DM.DMmesh);
    surf_drift(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = ...
        0.2e-10*(rand([DM.DMmesh - 2 * marginNpixel,1])-0.5);%/scale;
    
    surf_drift2 = zeros(DM.DMmesh);
    surf_drift2(marginNpixel+1 : end-marginNpixel, marginNpixel+1 : end-marginNpixel) = ...
        0.2e-10*(rand([DM.DMmesh - 2 * marginNpixel,1])-0.5);%/scale;
    
    % imagesc(surf_drift); colorbar;
    
    command_drift = height2voltage(surf_drift, DM, target.driftDM, 5)/scale;
    command_drift2 = height2voltage(surf_drift2, DM, target.driftDM, 5)/scale;
else
    %%
    command_drift = (rand([DM.activeActNum,1])-0.5)*2e-5;
    command_drift2 = (rand([DM.activeActNum,1])-0.5)*2e-5;
end
command_voltage = zeros(DM.Nact,DM.Nact);
command_voltage(DM.activeActIndex) = command_drift;
% imagesc(command_voltage);colorbar;

DM1command = command_drift;
img = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);

DM1command_2 = command_drift2;
img0 = getImg(target, DM, coronagraph, camera, DM1command_2, DM2command, simOrLab);

hold on
plot(abs(img0(darkHole.pixelIndex)-img(darkHole.pixelIndex))./img0(darkHole.pixelIndex))
