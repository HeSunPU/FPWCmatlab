function []=Laser_Power(A,C);

% Modified by A.J. on 20 Nov 2014 to be used with the MCLS1 in the HCIL.

% this function can change the power (A) of the channel (C)
% A = power in ampere 
% C = number of the channel
%
% For C = 2, A=43.95 gives 42000 counts (without subtracting the dark)

F=serial('COM3','BaudRate',115200,'DataBits',8,'StopBits',1);
fopen(F);

if     C==1
    max_power = 68.09;
elseif C==2
    max_power = 63.89;
elseif C==3
    max_power = 41.59;
elseif C==4
    max_power = 67.39;
else
    disp('No Way! C shall be 1,2,3 or 4  only')
    max_power = 0;
end

if A>max_power     
    disp(['No Way! must be less than ' num2str(max_power) 'mA only'])
    
elseif A==0
    fprintf(F,'%s\r',['channel=' num2str(C)]);
    pause(2)
    
    fprintf(F,'%s\r','enable=0');
    pause(1)
    
else
    fprintf(F,'%s\r',['channel=' num2str(C)]);
    pause(2)
    
    fprintf(F,'%s\r',['enable=' num2str(C)]);
    pause(2)

    fprintf(F,'%s\r',['current=' num2str(A) ]);
    pause(1)   
end

fclose(F);
delete(F)
clear F