function Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)
%
% PxF(3,1) : absolute position vector of the external force application point
% RxF(3,3) : absolute rotation matrix of the body
% VxF(3,1) : absolute velocity vector of the external force application point
% OMxF(3,1) : absolute angular velocity vector of the body
% AxF(3,1) : absolute acceleration vector of the external force application point
% OMPxF(3,1) : absolute angular acceleration vector of the body
%
% => All above vectors are expressed in the inertial reference frame !
%
% mbs_data : multibody data structure
% tsim : current time
% ixF : index of the external force sensor ('F' type in MBsysPad)
%        (can be obtained via the 'mbs_get_F_sensor_id' function)
%
% Swr(9,1) = [Fx; Fy; Fz; Mx; My; Mz; dxF];
%   - Force components (expressed in the inertial frame) : Fx, Fy, Fz
%   - Pure torque components (expressed in the inertial frame) : Mx, My, Mz
%   - Application point local coordinates vector (expressed in the body-fixed frame) : dxF(1:3,1);
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

Fx=0.0; Fy=0.0; Fz=0.0;
Mx=0.0; My=0.0; Mz=0.0;
idpt = mbs_data.xfidpt(ixF);
dxF = mbs_data.dpt(:,idpt);
Fi=zeros(3,1);
%/*-- Begin of user code --*/
%
% Use the 'mbs_get_F_sensor_id' function to get easily the force sensor
% indices, e.g. :
[F1,F2,F3,F4,F5]  = mbs_get_F_sensor_id(MBS_info,'Ground_F1','Ground_F2','Ground_F3','Ground_F4','Thruster');
%[F2,F3] = mbs_get_F_sensor_id(MBS_info,'myFsensor_2','myFsensor_3');
%
K_g=200000;  %Ground Parameters
D_g=K_g/50;
switch(ixF)
    case {F1,F2,F3,F4}
        gap=0;
        
        if (PxF(3)<gap)
            Fz=- K_g*(PxF(3))-D_g*VxF(3);
            
            nu=0.4;
            N=Fz;
            Beta = 39;
            Fx=-nu*N*atan(Beta* VxF(1));
            Fy =-nu*N*atan(Beta* VxF(2));
            F=[Fx ; Fy ; Fz]; % force vector in the local frame
            Fi = F;
        else
            Fz =0;
            Fx =0;
            Fy =0;
            
        end
        
    case F5
        
        if PxF(3)>3
            Max_Thrust=MBS_user.max_thrust;
            Fx=0.0;
            Fy=0.0;
            Max_Usage=910; %s
            Min_Thrust=0.12*Max_Thrust;
            
            K_V=1;
            D_V=0.15;
            if  MBS_user.Fuel==1
                if PxF(3)>30
                    V_0=9;
                elseif PxF(3)>3
                    V_0=0.38 ;
                else
                    K_V=0;
                    D_V=0;
                end
                Fz=Max_Thrust*-( K_V*(VxF(3)+V_0)+D_V*(AxF(3)));
                if Fz<0
                    Fz=0;
                end
            else
                Fz=0;
                MBS_user.State=0;
            end
        else
            Fz=0;
            MBS_user.State=0;
        end
        
        %             Control Velocidad
        % K_c=1000000;
        % D_c=10000;
        %            Control_Speed=K_c*(-VxF(3)-1)+D_c*(AxF(3));
        %
        %                  if Control_Speed>Max_Thrust
        %                     Fz=Max_Thrust;
        %                  elseif  Control_Speed<Min_Thrust
        %                     Fz=0;
        %                  else
        %                     Fz=Control_Speed;
        %                 end
        
        F=[Fx ; Fy ; Fz]; % force vector in the local frame
        Fi = RxF'*F;
        
        
        MBS_user.Thrust(MBS_user.step)=Fz;
        MBS_user.Tank_fuel(MBS_user.step)=mbs_data.m(6)*(1-0.5372)/((1-0.5372)*MBS_user.Init_Mass)*100;
end

%/*-- End of user code --*/

Swr = [Fi(1); Fi(2); Fi(3); Mx; My; Mz; dxF];

return
