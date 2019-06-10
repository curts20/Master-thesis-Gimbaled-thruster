%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Tue May  7 01:37:58 2019
%
%	==> Project name : Lunar_Lander_Th
%	==> using XML input file 
%
%	==> Number of joints : 22
%
%	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
%	==> Flops complexity : 109
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [h,Jac] = cons_hJ(s,tsim,usrfun)

 h = zeros(6,1);
 Jac = zeros(6,22);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C7 = cos(q(7));
  S7 = sin(q(7));
  C8 = cos(q(8));
  S8 = sin(q(8));

% = = Block_0_0_0_0_0_7 = = 
 
% Augmented Joint Position Vectors   

  Dz193 = q(19)+s.dpt(3,15);
 
% Trigonometric Variables  

  C17 = cos(q(17));
  S17 = sin(q(17));
  C18 = cos(q(18));
  S18 = sin(q(18));

% = = Block_0_0_0_0_0_8 = = 
 
% Augmented Joint Position Vectors   

  Dz223 = q(22)+s.dpt(3,17);
 
% Trigonometric Variables  

  C20 = cos(q(20));
  S20 = sin(q(20));
  C21 = cos(q(21));
  S21 = sin(q(21));

% = = Block_0_1_0_0_0_2 = = 
 
% Constraints and Constraints Jacobian 

%
  RO_1_88 = -S7*C8;
  RO_1_98 = C7*C8;
  RL_1_124 = s.dpt(1,9)*C8+s.dpt(3,9)*S8;
  RL_1_224 = RO_1_88*s.dpt(3,9)+s.dpt(1,9)*S7*S8;
  RL_1_324 = RO_1_98*s.dpt(3,9)-s.dpt(1,9)*C7*S8;
  JT_1_124_8 = -(s.dpt(1,9)*S8-s.dpt(3,9)*C8);
  JT_1_224_8 = RL_1_124*S7;
  JT_1_324_8 = -RL_1_124*C7;
%
  RL_3_126 = s.dpt(3,8)*S8;
  RL_3_226 = RO_1_88*s.dpt(3,8)+s.dpt(2,8)*C7;
  RL_3_326 = RO_1_98*s.dpt(3,8)+s.dpt(2,8)*S7;
  JT_3_126_8 = -(RL_3_226*S7-RL_3_326*C7);
  JT_3_226_8 = RL_3_126*S7;
  JT_3_326_8 = -RL_3_126*C7;

% = = Block_0_1_0_0_0_7 = = 
 
% Constraints and Constraints Jacobian 

%
  RO_0_818 = -S17*C18;
  RO_0_918 = C17*C18;
  RL_0_119 = Dz193*S18;
  RL_0_219 = Dz193*RO_0_818;
  RL_0_319 = Dz193*RO_0_918;
  RL_0_123 = s.dpt(3,16)*S18;
  RL_0_223 = RO_0_818*s.dpt(3,16);
  RL_0_323 = RO_0_918*s.dpt(3,16);
  JT_0_223_17 = -(RL_0_319+RL_0_323);
  JT_0_323_17 = RL_0_219+RL_0_223;
  JT_0_123_18 = C18*(q(19)+s.dpt(3,15))-RL_0_223*S17+RL_0_323*C17;
  JT_0_223_18 = S17*(RL_0_119+RL_0_123);
  JT_0_323_18 = -C17*(RL_0_119+RL_0_123);

% = = Block_0_1_0_0_0_8 = = 
 
% Constraints and Constraints Jacobian 

%
  RO_2_821 = -S20*C21;
  RO_2_921 = C20*C21;
  RL_2_122 = Dz223*S21;
  RL_2_222 = Dz223*RO_2_821;
  RL_2_322 = Dz223*RO_2_921;
  RL_2_125 = s.dpt(3,18)*S21;
  RL_2_225 = RO_2_821*s.dpt(3,18);
  RL_2_325 = RO_2_921*s.dpt(3,18);
  JT_2_225_20 = -(RL_2_322+RL_2_325);
  JT_2_325_20 = RL_2_222+RL_2_225;
  JT_2_125_21 = C21*(q(22)+s.dpt(3,17))-RL_2_225*S20+RL_2_325*C20;
  JT_2_225_21 = S20*(RL_2_122+RL_2_125);
  JT_2_325_21 = -C20*(RL_2_122+RL_2_125);

% = = Block_0_1_0_0_1_0 = = 
 
% Constraints and Constraints Jacobian 

%
  h_1 = RL_0_119+RL_0_123-RL_1_124+s.dpt(1,5);
  h_2 = RL_0_219+RL_0_223-RL_1_224;
  h_3 = RL_0_319+RL_0_323-RL_1_324;
%
  h_4 = RL_2_122+RL_2_125-RL_3_126;
  h_5 = RL_2_222+RL_2_225-RL_3_226+s.dpt(2,6);
  h_6 = RL_2_322+RL_2_325-RL_3_326;

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  h(1) = h_1;
  h(2) = h_2;
  h(3) = h_3;
  h(4) = h_4;
  h(5) = h_5;
  h(6) = h_6;
  Jac(1,8) = -JT_1_124_8;
  Jac(1,18) = JT_0_123_18;
  Jac(1,19) = S18;
  Jac(2,7) = RL_1_324;
  Jac(2,8) = -JT_1_224_8;
  Jac(2,17) = JT_0_223_17;
  Jac(2,18) = JT_0_223_18;
  Jac(2,19) = RO_0_818;
  Jac(3,7) = -RL_1_224;
  Jac(3,8) = -JT_1_324_8;
  Jac(3,17) = JT_0_323_17;
  Jac(3,18) = JT_0_323_18;
  Jac(3,19) = RO_0_918;
  Jac(4,8) = -JT_3_126_8;
  Jac(4,21) = JT_2_125_21;
  Jac(4,22) = S21;
  Jac(5,7) = RL_3_326;
  Jac(5,8) = -JT_3_226_8;
  Jac(5,20) = JT_2_225_20;
  Jac(5,21) = JT_2_225_21;
  Jac(5,22) = RO_2_821;
  Jac(6,7) = -RL_3_226;
  Jac(6,8) = -JT_3_326_8;
  Jac(6,20) = JT_2_325_20;
  Jac(6,21) = JT_2_325_21;
  Jac(6,22) = RO_2_921;

% ====== END Task 0 ====== 

  

