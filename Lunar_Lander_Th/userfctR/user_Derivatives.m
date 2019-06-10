function [uxd] = user_Derivatives(ux,mbs_data,tsim)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%[uxd] = user_Derivatives(ux,mbs_data,tsim)
%
% ux : user vector of state variables
% mbs_data : multibody data structure
% tsim : current time
%
% uxd : user vector of state derivatives

global MBS_user MBS_info
uxd = zeros(mbs_data.Nux,1);
% 
%     error_tilt_x =mbs_data.q(4);
%    ux(2) = error_tilt_x;
%    
%      error_tilt_y =mbs_data.q(5);
%     ux(1) = error_tilt_y;
%     
%     error_Speed_Z=mbs_data.qd(3);
%     uxd(1) = error_Speed_Z;
%/*-- End of user code --*/

return
