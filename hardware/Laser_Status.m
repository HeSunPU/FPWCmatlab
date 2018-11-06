function [A]=Laser_Status();

% this function check the output power (A) of the channel (C)
% C = number of the channel

F=serial('COM7','BaudRate',115200,'DataBits',8,'StopBits',1);
fclose(F);
fopen(F);

fprintf(F,'%s\r','statword?');
pause(2)
    
A=fscanf(F);
pause(1)

fclose(F);

delete(F)
clear F