% ManDemo is compatible with MATLAB and GNU Octave (www.octave.org).  
% Maneuvering trials demonstrated by simulating m-file vessel models:
% Type: >> mssHelp for more information
%
%   1)  Turning circle for the Mariner-class cargo vessel and a container ship
%       (see mariner.m and container.m)
%   2)  Zigzag manuvers for the Mariner class vessel and a container ship
%       (see mariner.m and container.m)
%   3)  Pullout maneuver for the Mariner-class cargo vessel and the Esso 
%       Osaka tanker (see mariner.m and tanker.m)
%
%   0)  Quit
echo off

while(1)
    demos= [
        'exTurnCircle'
        'exZigZag    '
        'exPullout   ' ];
    clc
    help ManDemo
    n = input('Select a demo number: ');
    if ( n <= 0 || n > 3 )
        break
    end
    demos = demos(n,:);
    eval(demos)
    clear
end

clear n demos
clc
