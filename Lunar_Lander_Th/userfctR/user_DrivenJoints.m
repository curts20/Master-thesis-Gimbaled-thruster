function [q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
%[q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)
%
% mbs_data : multibody data structure
% tsim : current time
%
% q, qd, qdd : updated column vectors of generalized coordinates
%
%
% mbs_data.q : generalized coordinates [column vector]
% mbs_data.qd : generalized velocities [column vector]
% mbs_data.qdd : generalized accelerations [column vector]
% mbs_data.nqc : number of driven variables
% mbs_data.qc : indices of driven variables [column vector]

global MBS_user MBS_info

q   = mbs_data.q;
qd  = mbs_data.qd;
qdd = mbs_data.qdd;

%/*-- Begin of user code --*/
% 
 actuator_id_x=mbs_get_joint_id(MBS_info,'EMA_Joint_x');
 actuator_id_y=mbs_get_joint_id(MBS_info,'EMA_Joint_y');
 
 if q(3)>3.3
 K_Th=0.5;          %Valores Para: 3 rad K=0.5 D=1.5
 D_Th=1.5;
 else
    K_Th=0;        
    D_Th=0; 
 end
% I_Th=0;
% 
% 
% %                Control Parameter x and y
% 
% 
% 
 despl_x=(K_Th*(q(5))+D_Th*(qd(5)));
% 

if despl_x<-0.075/2
    despl_x=-0.075/2;
elseif despl_x>0.075/2
    despl_x=0.075/2;
end
% % 
% % % 
%  despl_y=(K_Th*(q(4))+D_Th*(qd(4)));
% % 

% if despl_y<-0.075/2
%     despl_y=-0.075/2;
% elseif despl_y>0.075/2
%     despl_y=0.075/2;
% end

% q(actuator_id_y)=despl_y;
q(actuator_id_x)=despl_x;
% q(actuator_id_y)=despl_y;
%/*-- End of user code --*/

return
