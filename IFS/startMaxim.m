% Run this when the camera starts up.

% Set up COM objects
hCam = actxserver('MaxIm.CCDCamera');
% hDoc = actxserver('MaxIm.Document');
hApp = actxserver('MaxIm.Application');

% Set up time units and constants
seconds = 1;
milliseconds = 0.001;
msec = 0.001;
minutes = 60;
hours = 60*60;
BITMAX = (2^16) - 1;

% Set up camera link
hCam.invoke('LinkEnabled', 1);
