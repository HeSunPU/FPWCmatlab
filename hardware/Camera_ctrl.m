% camera_ctrl - provides basic control of QSI RS 6.1s CCD camera
%
% He Sun from Princeton HCIL - Sep. 4, 2015
%
% Brief useage:
%   Turn on the camera and Enable the handle:
%           h = Camera_ctrl(0, 'enable');
%   Initialization:
%           Camera_ctrl(h, 'init', temperature); % temperature: range -50 to
%           50
%   Stop fan and cooler:
%           Camera_ctrl(h, 'finalize');
%   Disable the handle:
%           Camera_ctrl(h, 'disable');
%   Open or close shutter;
%           h = Camera_ctrl(h, 'shutter', openflag); % openflag: 1 for open
%           0 for close
%   Set readout speed:
%           Camera_ctrl(h, 'readoutspeed', readoutflag); % readoutflag: 1
%           for fast readout, 0 for high image quality
%   Exposure properties:
%           Camera_ctrl(h, 'exposureproperties', start_pos, size_pixels,...
%                       bin_pixels); % the last three input arguments are
%                       all 2 dimension vectors, the default values are
%                       [0,0], [2758,2208],[1,1]
%   Set shutter priority:
%           Camera_ctrl(h, 'shutterpriority', shutterflag); % shutterflag:
%           0 for mechanical, 1 for electrical
%   Take pictures:
%           img = Camera_ctrl(h, 'exposure', exptime); % exptime is the
%           exposure time you can change
%   Show camera realtime picture:
%           Camera_ctrl(h, 'realtime'); % can be used during calibration

function output = Camera_ctrl(handle, cmd, varargin)

    n_argin = size(varargin,2);
    switch(lower(cmd))
        case 'enable'
            clear handle
            handle.camera=actxserver('QSICamera.CCDCamera');
            % confirm the camera is connected
            temp=get(handle.camera, 'Connected');
            if temp ~= 1
                set(handle.camera, 'Connected', 1);
            end
            % get the CCD serial number
%             handle.serialnum = get(handle.camera, 'SerialNumber');
            handle.serialnum = 00602768;
            handle.shutter = 1;
            handle.defaultstartpos = [0,0];
            handle.defaultsizepixels = [2758,2208];
            handle.defaultbinpixels = [1,1];
            output = handle;
        case 'init'
            assert(n_argin==1, 'Wrong number of input arguments');
            % set current camera is the main camera
            temp=get(handle.camera, 'IsMainCamera');
            if temp ~= 1
                set(handle.camera, 'IsMaincamera', 1);
            end
            % turn on the camera fan
            temp=get(handle.camera, 'FanMode');
            if strcmpi(temp,'FanFull') ~= 1
                set(handle.camera, 'FanMode', 'FanFull');
            end
%             set(handle.camera, 'FanMode', 'FanOff');
            % enable the CCD cooler
            temp=get(handle.camera, 'CoolerOn');
            if temp ~= 1
                set(handle.camera, 'CoolerOn', 1);
            end
            % set camera cooling temperature
            ccdtempc = cell2mat(varargin(1));
            temp=get(handle.camera, 'CanSetCCDTemperature');
            if temp == 1
                set(handle.camera, 'SetCCDTemperature', ccdtempc);
            end
            % set camera gain to low gain
            temp=get(handle.camera, 'CameraGain');
            if temp ~= 1
                set(handle.camera, 'CameraGain', 1);
            end
            output = 1;
        case 'shutter'
            assert(n_argin==1);
            openflag = cell2mat(varargin(1));
            if  openflag== 1;
                % Set the camera to manual shutter mode
                set(handle.camera, 'ManualShutterMode', 1);
                % Open the shutter as specified
                set(handle.camera, 'ManualShutterOpen', 1);
                handle.shutter = 1;
            else
                % Set the camera to manual shutter mode
                set(handle.camera, 'ManualShutterMode', 1);
                % Close the shutter as specified
                set(handle.camera, 'ManualShutterOpen', 0);
                % Set the camera to auto shutter mode
                set(handle.camera, 'ManualShutterMode', 0);
                handle.shutter = 0;
            end;
            output = handle;
        case 'exposureproperties'
            assert(n_argin==3,'Wrong number of input arguments');
            start_pos = cell2mat(varargin(1));
            size_pixels = cell2mat(varargin(2));
            bin_pixels = cell2mat(varargin(3));
            assert(length(start_pos)==2, 'Wrong dimension of start position');
            assert(length(size_pixels)==2, 'Wrong dimension of picture size');
            assert(length(bin_pixels)==2, 'Wrong dimension of binned pixels');
            %assert((start_pos(1)+size_pixels(1))*bin_pixels(1)<=h.defaultsizepixels(1), 'x pixels exceeds chip range!');
            %assert((start_pos(2)+size_pixels(2))*bin_pixels(2)<=h.defaultsizepixels(2), 'x pixels exceeds chip range!');
            set(handle.camera, 'StartX', start_pos(1));
            set(handle.camera, 'StartY', start_pos(2));
            set(handle.camera, 'NumX', size_pixels(1));
            set(handle.camera, 'NumY', size_pixels(2));
            set(handle.camera, 'BinX', bin_pixels(1));
            set(handle.camera, 'BinY', bin_pixels(2));
            output = 1;
        case 'shutterpriority'
            assert(n_argin==1,'Wrong number of input arguments');
            shutterflag = cell2mat(varargin(1));
            set(handle.camera, 'shutterpriority', shutterflag);
            output =1;
        case 'readoutspeed'
            assert(n_argin==1,'Wrong number of input arguments');
            readoutflag = cell2mat(varargin(1));
            set(handle.camera, 'readoutspeed', readoutflag);
            output =1;
        case 'exposure'
            assert(n_argin==1, 'Wrong number of input arguments');
            exptime = cell2mat(varargin(1));
            assert(handle.shutter==1||handle.shutter==0, 'The shutter status is incorrect');
            invoke(handle.camera, 'StartExposure', exptime, handle.shutter);
            % Wait for the exposure to complete
            status=get(handle.camera, 'ImageReady');
            donestatus=1;
            while status ~= donestatus
                status=get(handle.camera, 'ImageReady');
            end;
            output = get(handle.camera, 'ImageArray')';
        case 'realtime'
            f = figure(100);
            set(f, 'Name', 'Real Time Picture');
            %set(handle.camera, 'ShutterPriority', 1);
            while(ishandle(f))
                img = Camera_ctrl(handle, 'exposure', 0.0003);
                figure(100),imagesc(img), axis xy tight, colorbar;
                drawnow
            end
            output = 1;
        case 'finalize'
            % turn off the camera fan
            temp=get(handle.camera, 'FanMode');
            if strcmpi(temp,'FanOff') ~= 1
                set(handle.camera, 'FanMode', 'FanOff');
            end
            % disable the CCD cooler
            temp=get(handle.camera, 'CoolerOn');
            if temp ~= 0
                set(handle.camera, 'CoolerOn', 0);
            end
            Camera_ctrl(handle, 'shutter', 0);
            output = 1;
        case 'disable'
            % disconnect camera
            temp=get(handle.camera, 'Connected');
            if temp == 1
                set(handle.camera, 'Connected', 0);
            end
            output = 0;
            
        otherwise
            disp([ 'unknown command ' cmd ]);
            output = -1;
    end
end