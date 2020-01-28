%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Run EKF and EFC to deal with speckle drift
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('/Users/susanredmond/Desktop/PhD/Research/FPWCmatlab')
%%
Nitr = 10;%4000; % iterations of control loop
cRange = [-8, -2]; %[-12, -3];% the range for display
simOrLab = 'simulation';%'lab';%  'simulation' or 'lab', run the wavefront correction loops in simulation or in lab
runTrial = 1;
%%
Initialization_Maint;


%%
camera.exposure = 0.2;
camera.exposure0 = 0.2;

soc = tcpip('127.0.0.1', 4651,'NetworkRole','server');
soc.InputBufferSize = 16348*16; %depends on your input size
soc.OutputBufferSize = 16348*16; %same

image_size = [camera.Neta,camera.Nxi];
DM_command_size = [DM.activeActNum,1];

% for i = 1:1:T
itr = 1;
while 1
    fopen(soc);
%     mp.dm1.V = reshape(fread(soc,48*48,'double'), [48,48])'; %reading example
    DM1command = reshape(fread(soc,DM.activeActNum,'double'), [DM.activeActNum,1]); %reading example
    DM2command = reshape(fread(soc,DM.activeActNum,'double'), [DM.activeActNum,1]); %reading example
%     
%     [Efocalmeasured, EfocalPlanet, Imeasured] = ...
%         opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
%     
    % Get actual electric field
    [EfocalStarNoise, EfocalPlanetNoise, InoNoise] = opticalModel(target, DM, coronagraph, camera, DM1command, DM2command);
    contrastPerfect = mean(InoNoise(darkHole.pixelIndex));
    EfocalPerfect = EfocalStarNoise(darkHole.pixelIndex)*sqrt(target.flux * camera.exposure);
    %
            
    image = getImg(target, DM, coronagraph, camera, DM1command, DM2command, simOrLab);
    contrast = mean(image(darkHole.pixelIndex))
    image = image*(target.flux * camera.exposure);
    Iobserv = image(darkHole.pixelIndex);
    
    Imean = mean(Iobserv)
    
    figure(1)
    if (mod(itr,2)==0)
        plot(itr/2,contrast,"b*") %even case
    else
        plot(ceil(itr/2),contrast,"r*") %odd case
    end
    hold on
    drawnow
%     figure(1)
%     subplot(1,2,1)
%     imagesc(log10(abs(image))); colorbar;
%     title('Image Log Scale')
%     subplot(1,2,2)
%     plot(1:DM.activeActNum,DM1command,DM.activeActNum+1:2*DM.activeActNum,DM2command)
%     legend('Drift Command','Dither+EFC Command')
%     xlim([1,2*DM.activeActNum])
%     drawnow
    

%     fwrite(soc, reshape(real(Efocalmeasured(darkHole.pixelIndex).'),1,[]), 'double'); %writing example
%     fwrite(soc, reshape(imag(Efocalmeasured(darkHole.pixelIndex).'),1,[]), 'double'); %writing example
    fwrite(soc, reshape((Iobserv.'),1,[]), 'double'); %writing example
    fwrite(soc, reshape(real(EfocalPerfect.'),1,[]), 'double'); %writing example
    fwrite(soc, reshape(imag(EfocalPerfect.'),1,[]), 'double'); %writing example

%     fwrite(soc, reshape(Imeasured,1,[]), 'double'); %writing example
    %more code ...
    fclose(soc);
    
%     E_arr(:,itr) = EfocalPerfect;
%     DM2command_arr(:,itr) = DM2command;
    itr= itr+1;
end




