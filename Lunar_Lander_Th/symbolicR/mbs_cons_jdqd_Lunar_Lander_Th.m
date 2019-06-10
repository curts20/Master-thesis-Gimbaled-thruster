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
%	==> Function : F18 : Constraints Quadratic Velocity Terms (Jdqd)
%	==> Flops complexity : 277
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [Jdqd] = cons_jdqd(s,tsim,usrfun)

 Jdqd = zeros(6,1);

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
%
  RL_3_126 = s.dpt(3,8)*S8;
  RL_3_226 = RO_1_88*s.dpt(3,8)+s.dpt(2,8)*C7;
  RL_3_326 = RO_1_98*s.dpt(3,8)+s.dpt(2,8)*S7;

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

% = = Block_0_2_0_0_0_0 = = 
 
% Constraints Quadratic Terms 

%
  OM_0_218 = qd(18)*C17;
  OM_0_318 = qd(18)*S17;
  Ompqp_0_218 = -qd(17)*qd(18)*S17;
  Ompqp_0_318 = qd(17)*qd(18)*C17;
  OR_0_119 = qd(18)*C18*(q(19)+s.dpt(3,15));
  OR_0_219 = OM_0_318*RL_0_119-RL_0_319*qd(17);
  OR_0_319 = -(OM_0_218*RL_0_119-RL_0_219*qd(17));
  OR_0_123 = qd(18)*s.dpt(3,16)*C18;
  OR_0_223 = OM_0_318*RL_0_123-RL_0_323*qd(17);
  OR_0_323 = -(OM_0_218*RL_0_123-RL_0_223*qd(17));
%
  OM_1_28 = qd(8)*C7;
  OM_1_38 = qd(8)*S7;
  Ompqp_1_28 = -qd(7)*qd(8)*S7;
  Ompqp_1_38 = qd(7)*qd(8)*C7;
  OR_1_124 = OM_1_28*RL_1_324-OM_1_38*RL_1_224;
  OR_1_224 = OM_1_38*RL_1_124-RL_1_324*qd(7);
  OR_1_324 = -(OM_1_28*RL_1_124-RL_1_224*qd(7));
%
  OM_2_221 = qd(21)*C20;
  OM_2_321 = qd(21)*S20;
  Ompqp_2_221 = -qd(20)*qd(21)*S20;
  Ompqp_2_321 = qd(20)*qd(21)*C20;
  OR_2_122 = qd(21)*C21*(q(22)+s.dpt(3,17));
  OR_2_222 = OM_2_321*RL_2_122-RL_2_322*qd(20);
  OR_2_322 = -(OM_2_221*RL_2_122-RL_2_222*qd(20));
  OR_2_125 = qd(21)*s.dpt(3,18)*C21;
  OR_2_225 = OM_2_321*RL_2_125-RL_2_325*qd(20);
  OR_2_325 = -(OM_2_221*RL_2_125-RL_2_225*qd(20));
%
  OR_3_126 = OM_1_28*RL_3_326-OM_1_38*RL_3_226;
  OR_3_226 = OM_1_38*RL_3_126-RL_3_326*qd(7);
  OR_3_326 = -(OM_1_28*RL_3_126-RL_3_226*qd(7));

% = = Block_0_2_0_0_0_1 = = 
 
% Constraints Quadratic Terms 

%
  jdqd1 = OM_0_218*(OR_0_319+OR_0_323)-OM_0_318*OR_0_219-OM_0_318*OR_0_223-OM_1_28*OR_1_324+OM_1_38*OR_1_224+Ompqp_0_218*RL_0_319+Ompqp_0_218*...
 RL_0_323-Ompqp_0_318*RL_0_219-Ompqp_0_318*RL_0_223-Ompqp_1_28*RL_1_324+Ompqp_1_38*RL_1_224+(2.0)*qd(19)*(OM_0_218*RO_0_918-OM_0_318*RO_0_818);
  jdqd2 = OM_0_318*(OR_0_119+OR_0_123)-OM_1_38*OR_1_124-OR_0_319*qd(17)-OR_0_323*qd(17)+OR_1_324*qd(7)+Ompqp_0_318*RL_0_119+Ompqp_0_318*RL_0_123-...
 Ompqp_1_38*RL_1_124+(2.0)*qd(19)*(OM_0_318*S18-RO_0_918*qd(17));
  jdqd3 = -(OM_0_218*(OR_0_119+OR_0_123)-OM_1_28*OR_1_124-OR_0_219*qd(17)-OR_0_223*qd(17)+OR_1_224*qd(7)+Ompqp_0_218*RL_0_119+Ompqp_0_218*...
 RL_0_123-Ompqp_1_28*RL_1_124+(2.0)*qd(19)*(OM_0_218*S18-RO_0_818*qd(17)));
%
  jdqd4 = OM_2_221*(OR_2_322+OR_2_325)-OM_2_321*OR_2_222-OM_2_321*OR_2_225+Ompqp_2_221*RL_2_322+Ompqp_2_221*RL_2_325-Ompqp_2_321*RL_2_222-...
 Ompqp_2_321*RL_2_225+(2.0)*qd(22)*(OM_2_221*RO_2_921-OM_2_321*RO_2_821)-(OM_1_28*OR_3_326-OM_1_38*OR_3_226+Ompqp_1_28*RL_3_326-Ompqp_1_38*RL_3_226);
  jdqd5 = OM_2_321*(OR_2_122+OR_2_125)-OR_2_322*qd(20)-OR_2_325*qd(20)+Ompqp_2_321*RL_2_122+Ompqp_2_321*RL_2_125+(2.0)*qd(22)*(OM_2_321*S21-RO_2_921*...
 qd(20))-(OM_1_38*OR_3_126-OR_3_326*qd(7)+Ompqp_1_38*RL_3_126);
  jdqd6 = OM_1_28*OR_3_126-OM_2_221*(OR_2_122+OR_2_125)+OR_2_222*qd(20)+OR_2_225*qd(20)-OR_3_226*qd(7)+Ompqp_1_28*RL_3_126-Ompqp_2_221*RL_2_122-...
 Ompqp_2_221*RL_2_125-(2.0)*qd(22)*(OM_2_221*S21-RO_2_821*qd(20));

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  Jdqd(1) = jdqd1;
  Jdqd(2) = jdqd2;
  Jdqd(3) = jdqd3;
  Jdqd(4) = jdqd4;
  Jdqd(5) = jdqd5;
  Jdqd(6) = jdqd6;

% ====== END Task 0 ====== 

  

