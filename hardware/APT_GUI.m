% Sample Code from Thorlabs_APT_MATLAB.docx
% APT_GUI.m

% clear; 
% close all; clc;
% global h h2; % make h a global variable so it can be used outside the main 
          % function. Useful when you do event handling and sequential
          % move 
          
%% Create Matlab Figure Container 
fpos = get(0,'DefaultFigurePosition'); % figure default position 
fpos(3) = 650; % figure window size; Width
fpos(4) = 450; % Height

f = figure('Position', fpos,... 
    'Menu','None',...
    'Name','Horizontal Stage-APT GUI'); 
g = figure('Position', fpos,... 
    'Menu','None',...
    'Name','Vertical Stage-APT GUI'); 
%% Create ActiveX Controller
h = actxcontrol('MGMOTOR.MGMotorCtrl.1',[20 20 600 400 ], f);
h2 = actxcontrol('MGMOTOR.MGMotorCtrl.1',[20 20 600 400 ], g);
%% Initialize 

% Set the Serial Number 
SNh = 83815669; % put in the serial number of the hardware
SNv = 83815646; % Our second motor controller (for vertical postioning).
set(h,'HWSerialNum', SNh);
set(h2,'HWSerialNum', SNv);
% h = actxcontrol('MGMOTOR.MGMotorCtrl.1',[0,0,30,30]);
% h2 = actxcontrol('MGMOTOR.MGMotorCtrl.1',[0,0,30,30]);
% Start Control 
h.StartCtrl;
h2.StartCtrl;
h.SetStageAxisInfo(0, 0, 30, 0.5, 0.5, 0.5);
h2.SetStageAxisInfo(0, 0, 30, 0.5, 0.5, 0.5);
% Indentify the device
h.Identify;
h2.Identify;

pause(5); % waiting for the GUI to load up
%% Controlling the Hardware 
%h.MoveHome(0,0);  % Home the stage. First 0 is the channel ID (channel 1)
    % second 0 is to move immediately
%% Event Handling
% h.registerevent({'MoveComplete' 'MoveCompleteHandler'});

%% Sending Moving Commands 
% timeout = 10; % timeout for waiting for the move to be completed 
% % h.MoveJog(0,10); % Jog
% 
% % Move an absolute distance
% newPosH=24.87;  % for the run and planet calib, horizontal
% newPosV=26.15;  % for the run only
% % newPosH= 23.5; % For calibration of center star only, vertical
% % newPosV=25.45;% for all calibration
% 
% h.SetAbsMovePos(0,newPosH); 
% h.MoveAbsolute(0,1==0);
% h2.SetAbsMovePos(0,newPosV); 
% h2.MoveAbsolute(0,1==0);
% 
% t1 = clock; % current time 
% % while(etime(clock,t1)<timeout) 
% %     % wait while the motor is active; timeout to avoid dead loop
% %     s = h.GetStatusBits_Bits(0);
% %     if (IsMoving(s) == 0) 
% %         pause(2); % pause 2 seconds; 
% %         h.MoveHome(0,0); 
% %         disp('Home Started!'); 
% %         break;
% %     end
% % end

%% Close the GUI (which is in a MATLAB figure)
% pause on;
% pause(5);
% pause off;

% close(f);
% close(g);

