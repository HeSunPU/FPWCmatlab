
%% User Demo for S-Driver
%
%
% clear all;
% close all;
% clc;
%%
% NOTE: change these to match your install paths
addpath('C:\BostonMicromachines v5.2\BostonMicro');
addpath('C:\BostonMicromachines v5.2\BostonMicro\Matlab\v5.2');
addpath('C:\BostonMicromachines v5.2\Winx64\');
addpath('Z:\Matlab\data');
% addpath('C:\Lab\HCIL\FPWC\HCIL\util');
addpath('C:\Lab\HCIL\FPWCmatlab\HCIL\util');

%% Enter parameters

BrdNum = 1;

% constants
dac_scale = (2^16)-1; 
MAX_V = 300;

% poke values
V1 = dac_scale/2; 
%HVA_type = 'KILOLongStroke';   
HVA_type = 'KILO2x952';
Size = 4096; %DM Size

error = SetUpHVA(BrdNum, HVA_type);
count = 0;
[error, Mode] = GetCurrentHVAmode(BrdNum);


%error = TTLo_AnyCommand(BrdNum, 0);
error = TTLo_FVAL(BrdNum);
%error = TTLo_SpecificActuator(BrdNum,Mode.size-1,1);

error = AbortFramedData(BrdNum);
%HVA_type = 'HVA1024';  Size = 1024; %DM Size
%HVA_type = 'HVA2040';  Size = 2048; %DM Size
%HVA_type = 'HVA4096';  Size = 4096; %DM Size
%HVA_type = 'HVA32';    Size = 32; %DM Size
%HVA_type = 'HVA140';   Size = 140; %DM Size
%HVA_type = 'HVA137';   Size = 137; %DM Size
% HVA_type = 'KILO492';   Size = 492; %DM Size

% % HVA_type = 'KILO952';   Size = 952; %DM Size
%HVA_type = 'TEST3X3';  Size = 9; %DM Size
%HVA_type = 'KILO2x952';  Size = 4096; %DM Size


%% Open driver
if (Size == 32)
	error = SetUpHVA(BRDNum, 'HVA32');
elseif(Size == 492)
	error = SetUpHVA(BrdNum, 'SD492');
elseif(Size == 952)
	error = SetUpHVA(BrdNum, 'SD952');
elseif (Size == 1024)
	error = SetUpHVA(BrdNum, 'SD1024');
elseif(Size == 4096)
	error = SetUpHVA(BrdNum, 'KILO2x952');
else 
    disp('No HVA Select');
end

if (error)
	err_str = Decode_Error(error);
	disp(err_str)
	disp('Aborting program...')
	%$return error;
end

%% Put a cross on the DM
% 
% a1 = zeros(17);
% a1(end,:) = 1;
% a1(:,end) = 1;
% cross = 2^12*[a1 fliplr(a1); flipud([a1 fliplr(a1)]) ];
% figure; imagesc(cross); axis xy equal tight; colorbar;
% 
% cross1D = [dm_2Dto1D(cross); zeros(1024-952,1)];
% % cross1D = [dm_2Dto1D(cross); zeros(1020-952,1)];
% DM12bits = [cross1D; cross1D];
% 
% cross25 = 5*cross1D/2^12;
% 
% error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%%    Zero out both DMs
% OrienSe    lect='FlipX';  
% error = SetDMorientation(BrdNum,OrienSele     ct);

error = BurstHVA4096Frame1D(BrdNum, zeros(4096,1));
% % error = BurstHVA4096Frame1D(BrdNum, 100*(2^16-1)/300*ones(2048,1));

%%
error = BurstHVA4096Frame1D(BrdNum, zeros(4096,1));
 


%% Flat maps for DM1 and DM2
% OrienSelect='FlipX';
% error = SetDMorientation(BrdNum,OrienSelect);
% New flat maps
% 
% 
% v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
% v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii')-20;
v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW004#14_CLOSED_LOOP_200nm_Voltages_DM#1.txt','-ascii');
v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW018#40_CLOSED_LOOP_200nm_Voltages_DM#2.txt','-ascii');

%v1flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#010-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
%v2flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#014-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
% v1flat = [v1flat; zeros(1024,1)];
% v2flat = [v2flat; zeros(1024,1)];
% DM12bits = (2^16-1)/300*[v1flat(1:1024); v2flat(1:1024); zeros(2048,1)];
% DM12bits = (2^16-1)/410*[v1flat; v2flat; zeros(2048,1)];
% DM12bits = (2^16-1)/300*[v1flat; v2flat; zeros(2048,1)];

DM12bits = (2^16-1)/300*[125*ones(1024, 1); 125*ones(1024, 1); zeros(2048,1)];

% a = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_&_014_CLOSED_LOOP_200nm_VOLTAGES_Combined.txt','-ascii');
% b = [v1flat; v2flat; zeros(2048,1)];
% figure; plot(a-b)
% DM12bits = (2^16-1)/300*load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_&_014_CLOSED_LOOP_200nm_VOLTAGES_Combined.txt','-ascii');

error = BurstHVA4096Frame1D(BrdNum, DM12bits);

% % % %%
% vtemp = v1flat(1:952);
% v1flatT = [ flipud(vtemp); zeros(1024-952,1) ];
% DM12bits = (2^16-1)/420*[v1flatT; v2flat; zeros(2048,1)];
% error = BurstHVA4096Frame1D(BrdNum, DM12bits);
%% Sinusoids
v1sin8 = load('C:\BostonMicromachines\25CW003#010\25CW003#010-300D40-S-AP-Final-8periodSin.txt','-ascii');
v1sin4 = load('C:\BostonMicromachines\25CW003#010\25CW003#010-300D40-S-AP-Final-4periodSinAmp2.txt   ','-ascii');


v2sin8 = load('C:\BostonMicromachines\25CW003#014\25CW003#014-300D40-S-AP-Final-8periodSin.txt','-ascii');
v2sin4 = load('C:\BostonMicromachines\25CW003#014\25CW003#014-300D40-S-AP-Final-4periodSin.txt','-ascii');
v2sin42 = load('C:\BostonMicromachines\25CW003#014\25CW003#014-300D40-S-AP-Final-4periodSinAmp2.txt','-ascii');
   

v1 = 0*v1sin8;
v2 = 1*v2sin4;
error = BurstHVA4096Frame1D(BrdNum, (2^16-1)/300*[v1; v2; zeros(2048,1)]);

%% Crosses on top of flats
% v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
% v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW004#14_CLOSED_LOOP_200nm_Voltages_DM#1.txt','-ascii');
v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW018#40_CLOSED_LOOP_200nm_Voltages_DM#2.txt','-ascii');
DM12bits = (2^16-1)/300*[v1flat(1:1024); v2flat(1:1024); zeros(2048,1)];
amp1V = 10; % Vol
amp2V = 10; % Volts
a1 = zeros(17); a1(end,:) = 1; a1(:,end) = 1;
cross = [a1 fliplr(a1); flipud([a1 fliplr(a1)]) ];
% figure; imagesc(cross); axis xy equal tight; colorbar;
% cross1D = [dm_2Dto1D(cross); zeros(1024-952,1)];
cross1D = [cross(DM.activeActIndex); zeros(1024-952,1)];

for itr = 1:200
DM12bits = (2^16-1)/300*[1*amp1V*(cross1D)+v1flat(1:1024); 0*amp2V*cross1D+v2flat(1:1024); zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
pause(1)
DM12bits = (2^16-1)/300*[0*amp1V*(cross1D)+v1flat(1:1024); 1*amp2V*cross1D+v2flat(1:1024); zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
pause(1)
% DM12bits = (2^16-1)/300*[1*amp1V*(cross1D)+v1flat(1:1024); 1*amp2V*cross1D+v2flat(1:1024); zeros(2048,1)];
% error = BurstHVA4096Frame1D(BrdNum, DM12bits);
% pause(1)
end
%% Circle on top of flats
v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
DM12bits = (2^16-1)/300*[v1flat(1:1024); v2flat(1:1024); zeros(2048,1)];
amp1V = 30; % Volts
amp2V = 50;
0; % Volts
[x,y] = meshgrid(1:34);
 Z = ((x-17).^2+(y-17).^2<=16^2);
circ1D = [dm_2Dto1D(Z); zeros(1024-952,1)];
DM12bits = (2^16-1)/300*[amp1V*(0*circ1D)+v1flat(1:1024); amp2V*circ1D+v2flat(1:1024); zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%% Uniform voltages
DM12bits = (2^16-1)/300*(100*ones(4096,1));
error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%% DM1 flat, DM2 zeroes
DM12bits = (2^16-1)/300*[v1flat; 0*v2flat; zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%% DM1 zeroes, DM2 flat
DM12bits = (2^16-1)/300*[0*v1flat; v2flat; zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
%% zeros for both DMs
DM12bits = (2^16-1)/300*[0*v1flat; 0*v2flat; zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);



%% uniform for DM2
v1flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#010-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
v2flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#014-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
DM12V_flat = (2^16-1)/300*[v1flat; v2flat];
DM12V_2 = (2^16-1)/300*[0*v1flat ; 0*ones(size(v2flat)); zeros(2048,1)];
DM12V_1 = (2^16-1)/300*[0*ones(size(v1flat));0*v2flat; zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12V_2+DM12V_1+DM12V_flat);
%% DM1 cross on top of flat

a1 = zeros(17); a1(end,:) = 1; a1(:,end) = 1;
cross = [a1 fliplr(a1); flipud([a1 fliplr(a1)]) ];
% figure; imagesc(cross); axis xy equal tight; colorbar;

cross1D = [dm_2Dto1D(cross); zeros(1024-952,1)];
% cross1D = [dm_2Dto1D(cross); zeros(1020-952,1)];
% DM12bits = [cross1D; cross1D];


% asdf = zeros(34);

% asdf(18,18) = 50;
% cross1D = [dm_2Dto1D(asdf); zeros(1024-952,1)];
% DM12bits = (2^16-1)/300*[v1flat; cross1D+v2flat];

DM12bits = (2^16-1)/300*[15*cross1D+v1flat; v2flat];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%% DM2 cross
% DM12bits = [0*cross1D; cross1D];error = BurstHVA4096Frame1D(BrdNum, DM12bits);
DM12bits = (2^16-1)/300*[v1flat; 15*cross1D+v2flat; zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
%% Both cross
% DM12bits = [cross1D; cross1D];error = BurstHVA4096Frame1D(BrdNum, DM12bits);
DM12bits = (2^16-1)/300*[15*cross1D+v1flat; 15*cross1D+v2flat];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
%% ZEROS9
DM12bits = 0*[cross1D; cross1D];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
%% 100V uniform bias
DM12bits = (2^16-1)/300*100*ones(4096,1);
error = BurstHVA4096Frame1D(BrdNum, DM12bits);



%% Flat maps and sine wave on DM1 and DM2
sineWave = ones(34,34);
% sineWave(1:4:end,1:4:end) = -1;
% sineWave(2:4:end,2:4:end) =  -1;
% sineWave(3:4:end,3:4:end) = -1;
% sineWave(4:4:end,4:4:end) =  -1;
% sineWave(:,1:4:end) = 0;
% sineWave(:,2:4:end) =  1;
% sineWave(:,3:4:end) = 0;
% sineWave(:,4:4:end) =  -1;
%sineWave(:,1:2:end) = -1;
%sineWave(1:2:end,:) = -1;

sineWave(:,1:4:end) = -1;
sineWave(:,2:4:end) = -1;

% sineWave(:,5:8:end) = -1;
% sineWave(:,6:8:end) = -1;
% sineWave(:,7:8:end) = -1;
% sineWave(:,8:8:end) = -1;

% sineWave(5:8:end, :) = -1;
% sineWave(6:8:end, :) = -1;
% sineWave(7:8:end, :) = -1;
% sineWave(8:8:end, :) = -1;

dDM2Vplus = 10*[dm_2Dto1D(sineWave); zeros(1024-952,1)];
dDM2Vminus = 10*[dm_2Dto1D(sineWave); zeros(1024-952,1)];
% v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
% v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW004#14_CLOSED_LOOP_200nm_Voltages_DM#1.txt','-ascii');
v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\Engineering DMs\C25CW018#40_CLOSED_LOOP_200nm_Voltages_DM#2.txt','-ascii');
%DM12bits = (2^16-1)/300*[v1flat+dDM2Vminus; v2flat];
DM12bits = (2^16-1)/300*[v1flat(1:1024); v2flat(1:1024)-dDM2Vplus; zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);


%% Flat maps and center dot on DM1 and DM2
dot = zeros(34,34);
dot(17:18,17:18) = ones(2,2);

dDM2Vplus = 10*[dm_2Dto1D(dot); zeros(1024-952,1)];
dDM2Vminus = 10*[dm_2Dto1D(dot); zeros(1024-952,1)];
v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
%DM12bits = (2^16-1)/300*[v1flat+dDM2Vminus; v2flat];
DM12bits = (2^16-1)/300*[v1flat(1:1024); v2flat(1:1024); zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
for itr = 1:100
DM12bits = (2^16-1)/300*[v1flat(1:1024)+dDM2Vplus; v2flat(1:1024); zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
pause(1)
DM12bits = (2^16-1)/300*[v1flat(1:1024) ; v2flat(1:1024)+dDM2Vplus; zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
pause(1)
end

%% Flat maps and test area by area
v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
DM12bits = (2^16-1)/300*[v1flat(1:1024); v2flat(1:1024); zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
pause(0.5)
gap = 1;
for q = 485:495
    test = zeros(34,34);
    figure(1)
    test(q) = 3;
    test = fliplr(test);
    disp(['Act No. ' num2str(q)]);
    imagesc(flipud(test')), axis xy equal, colorbar
    drawnow
    dDMVtest = 15*[test(DM_config.dm_ele); zeros(1024-952,1)];
    DM12bits = (2^16-1)/300*[v1flat(1:1024)+dDMVtest;v2flat(1:1024); zeros(2048,1)];
    error = BurstHVA4096Frame1D(BrdNum, DM12bits);
    pause(3)
end
% for i = 4:gap:30
%     for j = 4:gap:30
%         test = zeros(34,34);
%         figure(1)
%         test(i:i+gap-1,j:j+gap-1) = 3;
%         imagesc(flipud(test')), axis xy equal, colorbar
%         drawnow
%         dDMVtest = 15*[dm_2Dto1D(test); zeros(1024-952,1)];
%         DM12bits = (2^16-1)/300*[v1flat+dDMVtest;v2flat; zeros(2048,1)];
%         error = BurstHVA4096Frame1D(BrdNum, DM12bits);
%         pause(0.3)
%     end
% end
%% Flat maps and Crosses
% OrienSelect='FlipX';
% error = SetDMorientation(BrdNum,OrienSelect);

v1flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#010-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
v2flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#014-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
DM12bits = (2^16-1)/300*[v1flat+cross25; v2flat];
% DM12bits = (2^16-1)/300*[v1flat; v2flat; zeros(2048,1)];
% DM12bits = (2^16-1)/300*[ v1flat; v2flat+cross25];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%% Flat maps + sine wave
load newshapes
% figure; imagesc(rot90(sineightnew,1)-25);
sineightNoBias = 0.5*[dm_2Dto1D(rot90(sineightnew,1)-25); zeros(1024-952,1)];

v1flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#010-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
v2flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#014-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
% DM1 Sine
DM12bits = (2^16-1)/300*[v1flat+sineightNoBias; v2flat];
% DM12bits = (2^16-1)/300*[v1flat+sineightNoBias; v2flat; zeros(2048,1)];

error = BurstHVA4096Frame1D(BrdNum, DM12bits);
%% Flat maps + probes
% Make DM surface shapes for probing the whole controllable region.

% function ProbeSurf = hcil_makeProbeSurf(ProbeArea,D,lambda,psi,offsetX,offsetY,XS,YS,DesiredCont)
%clear all; close all; clc;

DesiredContrast=1e-3;
Nact = 34;
dDMVcube = zeros(Nact,Nact,4);

lambda=635e-9; % meters
Ddm=10e-3; % meters
Ndm=256;
offsetX=0;
offsetY=0;



dx = Ddm/2/Ndm;
xs = (-Ndm:Ndm-1)'*dx + dx/2;
XS = repmat(xs.',2*Ndm,1);
YS = XS.';  

SincAmp = lambda*sqrt(DesiredContrast)*2*pi;%2.51;  % sqrt(2*pi) = 2.51 % Amplitude is in meters!!!

% % % Imaginary probe: just a sinc*sinc function:
mx = 34/Ddm;
my = 34/Ddm;
ProbeSurf = SincAmp*sinc(mx*(XS+offsetX)).*sinc(my*(YS+offsetY));

gain = 5e-9; % meters per volt
DMV = (1/gain)*imresize(ProbeSurf,[Nact Nact],'bilinear');
dDMVcube(:,:,1) = DMV;
dDMVcube(:,:,2) = -DMV;

% figure; imagesc(ProbeSurf); axis xy equal tight; colorbar;
figure; imagesc(DMV); axis xy equal tight; colorbar;

v1flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#010-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
v2flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#014-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');

probe1 = [dm_2Dto1D(DMV); zeros(1024-952,1)];
DM12bits = (2^16-1)/300*[v1flat+10*probe1; v2flat];

%error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%%
% % % Real probe: sinc*sinc*sin function:
D=Ddm;
psi = pi/2;
ProbeArea = [1 17 -17 17];
mx = (ProbeArea(2)-ProbeArea(1))/D;
my = (ProbeArea(4)-ProbeArea(3))/D;
wx = (ProbeArea(2)+ProbeArea(1))/2;
wy = (ProbeArea(4)+ProbeArea(3))/2;
ProbeSurf = SincAmp*sinc(mx*(XS+offsetX)).*sinc(my*(YS+offsetY)).*cos(2*pi*wx/Ddm*XS+ psi).*cos(2*pi*wy/Ddm*YS);

figure; imagesc(ProbeSurf); axis xy equal tight; colorbar;
DMV = (1/gain)*imresize(ProbeSurf,[Nact Nact],'bilinear');
% ProbeSurf = rot90(ProbeSurf,1);
dDMVcube(:,:,3) = DMV;
dDMVcube(:,:,4) = -DMV;
figure; imagesc(DMV); axis xy equal tight; colorbar;

probe2 = [dm_2Dto1D(DMV); zeros(1024-952,1)];
DM12bits = (2^16-1)/300*[v1flat+10*probe2; v2flat];

%error = BurstHVA4096Frame1D(BrdNum, DM12bits);
% save dDMVcube dDMVcube
%% Rotated sine wave
load newshapes
angle=30;%degrees
sinrot = ( imrotate(rot90(sineightnew,1)-25,angle,'crop'));
figure(2); imagesc(sinrot); axis xy;
sinrot1d = 8*1/2*[dm_2Dto1D(sinrot); zeros(1024-952,1)];

v1flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#010-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
v2flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#014-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
% DM1 Sine
DM12bits = (2^16-1)/300*[v1flat+sinrot1d; v2flat];
% DM12bits = (2^16-1)/300*[v1flat; v2flat+sinrot1d];
% DM12bits = (2^16-1)/300*[v1flat+sineightNoBias; v2flat; zeros(2048,1)];

error = BurstHVA4096Frame1D(BrdNum, DM12bits)

%% DM2 Sine
% DM12bits = (2^16-1)/300*[v1flat; sineightNoBias+v2flat];
DM12bits = (2^16-1)/300*[v1flat; sineightNoBias+v2flat; zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);


%% F
Fmatr = zeros(34);
Fmatr(8:12,8:27) = 1;
Fmatr(13:32,8:12) = 1;
Fmatr(18:22,13:22) = 1;
Fmatr=2^12*flipud(Fmatr);

Fm2 = flipud(Fmatr);
F2 =[dm_2Dto1D(Fm2); zeros(1020-952,1)];
% figure; imagesc(Fmatr); axis xy equal tight;

F = [dm_2Dto1D(Fmatr); zeros(1020-952,1)];
DM12bits = [F; 0*F2];
% DM12bits = [0*F; F2];

error = BurstHVA4096Frame1D(BrdNum, DM12bits);


%% Sine wave on DM
load newshapes
% sine = ((2^16-1)/200)*rot90((sineightnew-25)*0.3+50,1);
sine = ((2^16-1)/200)*rot90((sineightnew-25)*0.45+50,1);
figure(1); imagesc(sine/2^12);
s = [dm_2Dto1D(sine); zeros(1020-952,1)];
 DM12bits = [s; 0*s];
% DM12bits = [0*s; s];
% DM12bits = [0*F; F2];

error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%% Flat maps and Zernikes
addpath('C:\Lab\HCIL\FPWC\HCIL\util');
minZerN = 2;
maxZerN = 4;
coef_vals = zeros(12,1);
 coef_vals(1) = -0.015*200;
 %coef_vals(2) = 0.034*1000;
 coef_vals(3) = 0.015*200;
fitMap_DM2 = map_from_zernCoef(34,coef_vals,minZerN,maxZerN);
figure, imagesc(fitMap_DM2)
dDM2V = [dm_2Dto1D(fitMap_DM2); zeros(1024-952,1)];
v1flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#010-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
v2flat = load('C:\Lab\HCIL\FPWC\HCIL\maps\25CW003#014-300D40-S-AP-Final-VoltagesForFlat.txt','-ascii');
DM12bits = (2^16-1)/300*[v1flat; v2flat+dDM2V];
%DM12bits = (2^16-1)/300*[100*ones(size(v1flat)); 0*ones(size(v2flat))+dDM2V];
%DM12bits = (2^16-1)/300*[v1flat; v2flat];
% DM12bits = (2^16-1)/300*[v1flat; v2flat; zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);

%% Loop between crosses on DMs
v1flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#010_CLOSED_LOOP_200nm_VOLTAGES.txt','-ascii');
v2flat = load('C:\BostonMicromachines v5.2\Flatmap Data for Princeton\C25CW003#014_CLOSED_LOOP_200nm_Voltages.txt','-ascii');
DM12bits = (2^16-1)/300*[v1flat(1:1024); v2flat(1:1024); zeros(2048,1)];
amp1V = 10; % Volts
amp2V = 10; % Volts
a1 = zeros(17); a1(end,:) = 1; a1(:,end) = 1;
cross = [a1 fliplr(a1); flipud([a1 fliplr(a1)]) ];
% figure; imagesc(cross); axis xy equal tight; colorbar;
cross1D = [dm_2Dto1D(cross); zeros(1024-952,1)];
DM12bits = (2^16-1)/300*[amp1V*cross1D+v1flat(1:1024); amp2V*cross1D+v2flat(1:1024); zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, DM12bits);
for jj=1:100
    % DM1 cross
    DM12bits = (2^16-1)/300*[amp1V*cross1D+v1flat(1:1024); 0*cross1D];error = BurstHVA4096Frame1D(BrdNum, DM12bits);
    pause(1)
    % DM2 cross
    DM12bits = (2^16-1)/300*[0*cross1D; amp2V*cross1D+v2flat(1:1024)];error = BurstHVA4096Frame1D(BrdNum, DM12bits);
    pause(1)
    DM12bits = (2^16-1)/300*[amp1V*cross1D+v1flat(1:1024); amp2V*cross1D+v2flat(1:1024); zeros(2048,1)];
    error = BurstHVA4096Frame1D(BrdNum, DM12bits);
    pause(1)
%     % Crosses on both
%     DM12bits = [cross1D; cross1D];error = BurstHVA4096Frame1D(BrdNum, DM12bits);
%     pause(1)
end


%%
% %% Poke Actuators automatically
% k = 1; % Put the actuator# you want to start at here
% endactuator = 952;
% for i = 1:endactuator %endactuator has to be at leasthow many you want to poke
%           error = PokeDM(BrdNum, k, V1);
% %           inputstr = strcat('Actuator # ',num2str(k),' poked, continue? (n to quit)');
% pause(0.01);          
% error = PokeDM(BrdNum, k, 0);
% %          answ = input(inputstr,'s');
% %     if answ == 'n' return; end
%   
% k = k+1; % for next sequential actuator 
%  
% end
% %% Pokes manually
% k = 1024; % you want
% 
% 
% for i = 1024:2048 %how many you want to poke
%            error = PokeDM(BrdNum, k, V1);
%            inputstr = strcat('Actuator # ',num2str(k),' poked, continue? (n to quit)');
%            answ = input(inputstr,'s');
%            error = PokeDM(BrdNum, k, 0);
%            if answ == 'n' 
%                return; 
%            end
%            k = k+1; % for next sequential actuator 
% 
% end
% % Clear DM
% error = ClearHVA(BrdNum); 
% % Clear DM
% error = ClearHVA(BrdNum);
% 
% %% 
% 
% %
% 
% for k = 1:9:60
% 	TestCmds = round(ones(4096,1)*(k/MAX_V)*dac_scale);
% 	
% 	if (Size == 36)
% 		error = BurstHVA36Frame1D(BrdNum, TestCmds);
% 	elseif(Size == 144)
% 		error = BurstHVA144Frame1D(BrdNum, TestCmds);
%     elseif(Size == 1024)
%         %error = SetUpHVA(BrdNum, 'KILO952');
%         error = BurstHVA1024Frame1D(BrdNum, TestCmds);
%     else	Size = 4096;
% 		error = BurstHVA4096Frame1D(BrdNum, TestCmds);
% 	end;
% 	
% 	pause(1);
% end;
% 
% ZeroArray = zeros(Size, 1);
% error = BurstHVA4096Frame1D(BrdNum, ZeroArray);
% 
% n%% Load and apply shape
% %	
% %	The load() function is used to read the shape value.
% %	This assumes the shape is saved as a Matlab .mat file.
% %	This may need to be changed to suit the user's file format.
% %
% %	The data may also need adjustment to DAC values if the shape is
% %	saved as voltages or deflections.
% MAX_V = 200;
% load('KILO2x952_CLOSED_LOOP.mat');
% % load('KILO_S_VMAP.mat');
% TestShape = floor(dac_scale*(voltage_map_min./MAX_V));
% 
% 
% % if (length(TestShape(:)) ~= Size)
% % 	disp('Loaded shape is the wrong size.')
% % end	
% 
% if (Size == 36)
% 	error = BurstHVA36Frame1D(BRDNum, TestShape);
% elseif(Size == 144)
% 	error = BurstHVA144Frame1D(BrdNum, TestShape);
%      elseif(Size == 952)
%      error = BurstHVA4096Frame1D(BrdNum, TestShape);
% else
% 	Size = 4096;
% 	error = BurstHVA4096Frame1D(BrdNum, TestShape);
%     
%  end;
% pause (2);
% 
% ZeroArray = zeros(Size, 1);
% error = BurstHVA4096Frame1D(BrdNum, ZeroArray);