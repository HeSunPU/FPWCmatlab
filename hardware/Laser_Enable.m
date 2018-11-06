function []=Laser_Enable(enableStatus)

% Modified by A.J. on 20 Nov 2014 to be used with the MCLS1 in the HCIL.

% this function can change the power (A) of the channel (C)
% A = power in ampere 
% C = number of the channel
%
% For C = 2, A=43.95 gives 42000 counts (without subtracting the dark)

F=serial('COM3','BaudRate',115200,'DataBits',8,'StopBits',1);
fopen(F);

if(strcmpi('on',enableStatus))
    fprintf(F,'%s\r','system=1');
    %fprintf(F,'%s\r','enable=1');
    disp('Laser is now enabled')
else
    fprintf(F,'%s\r','enable=0');
    fprintf(F,'%s\r','system=0');
    disp('Laser is now disabled')
end



fclose(F);
delete(F)
clear F

pause(3)