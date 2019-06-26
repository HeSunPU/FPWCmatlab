% motor_op - provides basic control of a single Thorlabs LTS 300 motor stage.
%
% Matthew Hasselfield - Nov. 26, 2008
% Revised and tested by He Sun in Princeton HCIL - Sep. 2, 2015
%
% Revised and tested by Susan Redmond in Princeton HCIL - 2019  

% Brief usage:
%   Initialization:
%       h = motor_op(0, 'init');
%
%   Motion:
%       motor_op(h, 'goto', 5.4); % For LTS 300, distance range 0 to 300
%       motor_op(h, 'goto_wait', 5.4);  %Careful, this times out...
%       motor_op(h, 'goto_home');
%       motor_op(h, 'stop');
%       current_pos = motor_op(h, 'pos');
%      
%   Velocity and acceleration control:
%       max_vel = motor_op(h, 'get_vel');
%       motor_op(h, 'set_vel', new_max_vel);
%       accel = motor_op(h, 'get_accel');
%       motor_op(h, 'set_accel', new_accel);
%   
%   Clean up:
%       motor_op(h, 'cleanup');
%
% Notes:
%   You must initialize the motor before using it.  The 'init'
%   function returns a 'handle' that you must pass as the first
%   argument in all subsequent commands.  The handle is actually a
%   structure that contains a few useful fields:
%      h.stage    the activeX object for the Thorlabs control top-level.
%      h.ctrl     the activeX object for the stage we're controlling.
%      h.figure   the handle of the hidden figure where our controls live.
%

function output = motor_op(handle, cmd, varargin)

    n_argin = size(varargin,2);
    if (~strcmp(cmd,'init'))
        c = handle.ctrl;
        h = handle.stage;
        f = handle.figure;
    end
    
    motor_id = 0;
    switch(cmd)
        case 'init'
            % Create controls on a hidden window
            clear handle;
            fpos = get(0, 'DefaultFigurePosition');
            fpos(3) = 650;
            fpos(4) = 450;
            handle.figure = figure('Position', fpos,'Menu', 'None', 'Name', 'Camera Stage APT GUI');
            f = handle.figure;
            %set(f, 'Visible', 'off');
            %set(f, 'NextPlot', 'new');

            % Start system
            c = actxcontrol('MG17SYSTEM.MG17SystemCtrl.1', [0 0 100 100]);
            handle.ctrl = c;
            c.StartCtrl;
            [a,n_motor] = c.GetNumHWUnits(6, 0);
            if n_motor == 0
                disp('No motors found!');
            end
%             if n_motor ~= 1
%                 disp('Wrong number of motors found...');
%                 close(f)
%                 output = -1;
%             end
%             [a, serial_number] = c.GetHWSerialNum(6, 0, 0);
            % Start motor
            if strcmp(varargin,'cam') == 1
                serial_number = 45862339;
            end
            if strcmp(varargin,'hor') == 1
                serial_number = 83815669;%SNh
            end
            if strcmp(varargin,'vert') == 1
                serial_number = 83815646;%SNv
            end
            handle.stage = actxcontrol('MGMOTOR.MGMotorCtrl.1',[0,0,300,300]);
            h = handle.stage;
            h.HWSerialNum = serial_number;
            h.StartCtrl;
            h.Identify;
            %pause(5);
            output = handle;
            
        case 'pos'
            output = h.GetPosition_Position(motor_id);
            
        case 'goto_wait'
            h.SetAbsMovePos(motor_id, varargin{1});
            output = h.MoveAbsolute(motor_id, true);
            
        case 'goto'
            h.SetAbsMovePos(motor_id, varargin{1});
            output = h.MoveAbsolute(motor_id, false);
            
        case 'goto_home'
            output = h.MoveHome(motor_id, true);
            
        case 'stop'
            % Please somebody fix this...
            %motor_op(handle,'goto',motor_op(handle,'pos',0));
            %output = h.StopImmediate(motor_id);
            output = h.StopProfiled(motor_id);
            
        case 'get_vel'
            [status, min_v, accel, max_v] = h.GetVelParams(motor_id, 0,0,0);
            output = max_v;
            
        case 'set_vel'
            [status, min_v, accel, max_v] = h.GetVelParams(motor_id, 0,0,0);
            if n_argin > 1
                min_v = varargin{2};
            end
            output = h.SetVelParams(motor_id, min_v, accel, varargin{1});

        case 'get_accel'
            [status, min_v, accel, max_v] = h.GetVelParams(motor_id, 0,0,0);
            output = accel;
            
        case 'set_accel'
            [status, min_v, accel, max_v] = h.GetVelParams(motor_id, 0,0,0);
            output = h.SetVelParams(motor_id, min_v, varargin{1}, max_v);

        case 'wait_free'
            while abs(motor_op(handle,'pos',0) - varargin{1}) < varargin{2}
                pause(0.1);
            end
            output = motor_op(handle,'pos',0);

        case 'cleanup'
            h.StopCtrl;
            c.StopCtrl;
            if ishandle(f)~=0
                close(f);
            end
            
            output = 0;
            
        otherwise
            disp([ 'unknown command ' cmd ]);
            output = -1;
            
    end

end