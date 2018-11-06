function [] = initializeDM(DM)
%% Initialize the DM hardware drivers
% Developed by He Sun on Feb. 23, 2017, revised from A. J. Riggs's
% "hcil_main.m" file and BMC example codes
%
%% add the DM drivers folders to the working paths
addpath('C:\BostonMicromachines v5.2\BostonMicro');
addpath('C:\BostonMicromachines v5.2\BostonMicro\Matlab\v5.2');
addpath('C:\BostonMicromachines v5.2\Winx64\');
addpath('Z:\Matlab\data');

%% Initialize the parameters of the DMs
BrdNum = 1; % Parameters of DMs
dac_scale = (2^16)-1; % constants used to convert voltages in volts to commands in bits
MAX_V = 300; % maximum DM voltage
HVA_type = 'KILO2x952'; % DM type
Size = 4096; %DM command size
error = SetUpHVA(BrdNum, HVA_type); % set up the DM type
[error, Mode] = GetCurrentHVAmode(BrdNum);
error = TTLo_FVAL(BrdNum);
error = AbortFramedData(BrdNum);

%% Open drivers
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
end;

%% poke voltages to create flat DM surfaces
v1flat = [DM.DM1bias; zeros(1024 - DM.activeActNum, 1)];
v2flat = [DM.DM2bias; zeros(1024 - DM.activeActNum, 1)];
FlatCommandBits = dac_scale/MAX_V*[v1flat(1:1024); v2flat(1:1024); zeros(2048,1)];
error = BurstHVA4096Frame1D(BrdNum, FlatCommandBits);

end