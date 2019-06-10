function [Max_Thrust] = Rocket_Engines_Th(mbs_data)
%% Function which determines which engine is used
% -depends on the total mass, thus the total thrust required.
% -I've set 40% margin aprox to lift the lander, so the thrusters have 1.4
% capacity to lift its weight (T>1.4*Weight)
% All the thrust units are [N]


global MBS_user MBS_info
mass=mbs_data.m(6);

if (30000>mass) &&( mass>=10000)
    %CECE RL-10 Thruster
    Max_Thrust= 66700 ;
    T_m=51.9;
elseif ( mass>=30000) %For a big mass imaginary engine
    Max_Thrust= 100000*3;
      T_m=51.9;          
elseif (10000>mass) &&( mass>=4000)
    % OMS 
     Max_Thrust= 26700;
     T_m=23.3;
elseif (4000>mass)&&(mass>=900)
    % RS-41 
     Max_Thrust= 11100;
     T_m=16.43;
else
    %AVUM RD-843
    Max_Thrust= 2452;
    T_m=15.71;
end
 mbs_data.m(6)=mbs_data.m(6)+Max_Thrust/T_m;
end