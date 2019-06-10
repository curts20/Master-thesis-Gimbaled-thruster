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
%	==> Generation Date : Tue May  7 01:37:59 2019
%
%	==> Project name : Lunar_Lander_Th
%	==> using XML input file 
%
%	==> Number of joints : 22
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 1273
%
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.020 seconds
%
%-------------------------------------------------------------
%
function [sens] = sensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,22);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C4 = cos(q(4));
  S4 = sin(q(4));
  C5 = cos(q(5));
  S5 = sin(q(5));
  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C7 = cos(q(7));
  S7 = sin(q(7));
  C8 = cos(q(8));
  S8 = sin(q(8));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C9 = cos(q(9));
  S9 = sin(q(9));
  C10 = cos(q(10));
  S10 = sin(q(10));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C11 = cos(q(11));
  S11 = sin(q(11));
  C12 = cos(q(12));
  S12 = sin(q(12));

% = = Block_0_0_0_0_0_5 = = 
 
% Trigonometric Variables  

  C13 = cos(q(13));
  S13 = sin(q(13));
  C14 = cos(q(14));
  S14 = sin(q(14));

% = = Block_0_0_0_0_0_6 = = 
 
% Trigonometric Variables  

  C15 = cos(q(15));
  S15 = sin(q(15));
  C16 = cos(q(16));
  S16 = sin(q(16));

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_0_1 = = 
 
% Sensor Kinematics 


    ROcp0_25 = S4*S5;
    ROcp0_35 = -C4*S5;
    ROcp0_85 = -S4*C5;
    ROcp0_95 = C4*C5;
    ROcp0_16 = C5*C6;
    ROcp0_26 = ROcp0_25*C6+C4*S6;
    ROcp0_36 = ROcp0_35*C6+S4*S6;
    ROcp0_46 = -C5*S6;
    ROcp0_56 = -(ROcp0_25*S6-C4*C6);
    ROcp0_66 = -(ROcp0_35*S6-S4*C6);
    OMcp0_25 = qd(5)*C4;
    OMcp0_35 = qd(5)*S4;
    OMcp0_16 = qd(4)+qd(6)*S5;
    OMcp0_26 = OMcp0_25+ROcp0_85*qd(6);
    OMcp0_36 = OMcp0_35+ROcp0_95*qd(6);
    OPcp0_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp0_26 = ROcp0_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp0_35*S5-ROcp0_95*qd(4));
    OPcp0_36 = ROcp0_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp0_25*S5-ROcp0_85*qd(4));
    RLcp0_127 = s.dpt(3,7)*S5;
    RLcp0_227 = ROcp0_85*s.dpt(3,7);
    RLcp0_327 = ROcp0_95*s.dpt(3,7);
    POcp0_127 = RLcp0_127+q(1);
    POcp0_227 = RLcp0_227+q(2);
    POcp0_327 = RLcp0_327+q(3);
    JTcp0_127_5 = s.dpt(3,7)*C5;
    JTcp0_227_5 = RLcp0_127*S4;
    JTcp0_327_5 = -RLcp0_127*C4;
    ORcp0_127 = OMcp0_26*RLcp0_327-OMcp0_36*RLcp0_227;
    ORcp0_227 = -(OMcp0_16*RLcp0_327-OMcp0_36*RLcp0_127);
    ORcp0_327 = OMcp0_16*RLcp0_227-OMcp0_26*RLcp0_127;
    VIcp0_127 = ORcp0_127+qd(1);
    VIcp0_227 = ORcp0_227+qd(2);
    VIcp0_327 = ORcp0_327+qd(3);
    ACcp0_127 = qdd(1)+OMcp0_26*ORcp0_327-OMcp0_36*ORcp0_227+OPcp0_26*RLcp0_327-OPcp0_36*RLcp0_227;
    ACcp0_227 = qdd(2)-OMcp0_16*ORcp0_327+OMcp0_36*ORcp0_127-OPcp0_16*RLcp0_327+OPcp0_36*RLcp0_127;
    ACcp0_327 = qdd(3)+OMcp0_16*ORcp0_227-OMcp0_26*ORcp0_127+OPcp0_16*RLcp0_227-OPcp0_26*RLcp0_127;

% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp0_127;
    sens.P(2) = POcp0_227;
    sens.P(3) = POcp0_327;
    sens.R(1,1) = ROcp0_16;
    sens.R(1,2) = ROcp0_26;
    sens.R(1,3) = ROcp0_36;
    sens.R(2,1) = ROcp0_46;
    sens.R(2,2) = ROcp0_56;
    sens.R(2,3) = ROcp0_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp0_85;
    sens.R(3,3) = ROcp0_95;
    sens.V(1) = VIcp0_127;
    sens.V(2) = VIcp0_227;
    sens.V(3) = VIcp0_327;
    sens.OM(1) = OMcp0_16;
    sens.OM(2) = OMcp0_26;
    sens.OM(3) = OMcp0_36;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp0_127_5;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp0_327;
    sens.J(2,5) = JTcp0_227_5;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp0_227;
    sens.J(3,5) = JTcp0_327_5;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp0_85;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp0_95;
    sens.A(1) = ACcp0_127;
    sens.A(2) = ACcp0_227;
    sens.A(3) = ACcp0_327;
    sens.OMP(1) = OPcp0_16;
    sens.OMP(2) = OPcp0_26;
    sens.OMP(3) = OPcp0_36;
 
% 
case 2, 


% = = Block_1_0_0_2_0_1 = = 
 
% Sensor Kinematics 


    ROcp1_25 = S4*S5;
    ROcp1_35 = -C4*S5;
    ROcp1_85 = -S4*C5;
    ROcp1_95 = C4*C5;
    ROcp1_16 = C5*C6;
    ROcp1_26 = ROcp1_25*C6+C4*S6;
    ROcp1_36 = ROcp1_35*C6+S4*S6;
    ROcp1_46 = -C5*S6;
    ROcp1_56 = -(ROcp1_25*S6-C4*C6);
    ROcp1_66 = -(ROcp1_35*S6-S4*C6);
    OMcp1_25 = qd(5)*C4;
    OMcp1_35 = qd(5)*S4;
    OMcp1_16 = qd(4)+qd(6)*S5;
    OMcp1_26 = OMcp1_25+ROcp1_85*qd(6);
    OMcp1_36 = OMcp1_35+ROcp1_95*qd(6);

% = = Block_1_0_0_2_0_2 = = 
 
% Sensor Kinematics 


    ROcp1_47 = ROcp1_46*C7+S5*S7;
    ROcp1_57 = ROcp1_56*C7+ROcp1_85*S7;
    ROcp1_67 = ROcp1_66*C7+ROcp1_95*S7;
    ROcp1_77 = -(ROcp1_46*S7-S5*C7);
    ROcp1_87 = -(ROcp1_56*S7-ROcp1_85*C7);
    ROcp1_97 = -(ROcp1_66*S7-ROcp1_95*C7);
    ROcp1_18 = ROcp1_16*C8-ROcp1_77*S8;
    ROcp1_28 = ROcp1_26*C8-ROcp1_87*S8;
    ROcp1_38 = ROcp1_36*C8-ROcp1_97*S8;
    ROcp1_78 = ROcp1_16*S8+ROcp1_77*C8;
    ROcp1_88 = ROcp1_26*S8+ROcp1_87*C8;
    ROcp1_98 = ROcp1_36*S8+ROcp1_97*C8;
    OMcp1_17 = OMcp1_16+ROcp1_16*qd(7);
    OMcp1_27 = OMcp1_26+ROcp1_26*qd(7);
    OMcp1_37 = OMcp1_36+ROcp1_36*qd(7);
    OMcp1_18 = OMcp1_17+ROcp1_47*qd(8);
    OMcp1_28 = OMcp1_27+ROcp1_57*qd(8);
    OMcp1_38 = OMcp1_37+ROcp1_67*qd(8);
    OPcp1_18 = qdd(4)+ROcp1_16*qdd(7)+ROcp1_47*qdd(8)+qdd(6)*S5+qd(5)*qd(6)*C5+qd(7)*(OMcp1_26*ROcp1_36-OMcp1_36*ROcp1_26)+qd(8)*(OMcp1_27*ROcp1_67...
 -OMcp1_37*ROcp1_57);
    OPcp1_28 = ROcp1_26*qdd(7)+ROcp1_57*qdd(8)+ROcp1_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp1_35*S5-ROcp1_95*qd(4))-qd(7)*(OMcp1_16*ROcp1_36...
 -OMcp1_36*ROcp1_16)-qd(8)*(OMcp1_17*ROcp1_67-OMcp1_37*ROcp1_47);
    OPcp1_38 = ROcp1_36*qdd(7)+ROcp1_67*qdd(8)+ROcp1_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp1_25*S5-ROcp1_85*qd(4))+qd(7)*(OMcp1_16*ROcp1_26...
 -OMcp1_26*ROcp1_16)+qd(8)*(OMcp1_17*ROcp1_57-OMcp1_27*ROcp1_47);
    RLcp1_128 = ROcp1_78*s.dpt(3,10);
    RLcp1_228 = ROcp1_88*s.dpt(3,10);
    RLcp1_328 = ROcp1_98*s.dpt(3,10);
    POcp1_128 = RLcp1_128+q(1);
    POcp1_228 = RLcp1_228+q(2);
    POcp1_328 = RLcp1_328+q(3);
    ORcp1_128 = OMcp1_28*RLcp1_328-OMcp1_38*RLcp1_228;
    ORcp1_228 = -(OMcp1_18*RLcp1_328-OMcp1_38*RLcp1_128);
    ORcp1_328 = OMcp1_18*RLcp1_228-OMcp1_28*RLcp1_128;
    VIcp1_128 = ORcp1_128+qd(1);
    VIcp1_228 = ORcp1_228+qd(2);
    VIcp1_328 = ORcp1_328+qd(3);
    ACcp1_128 = qdd(1)+OMcp1_28*ORcp1_328-OMcp1_38*ORcp1_228+OPcp1_28*RLcp1_328-OPcp1_38*RLcp1_228;
    ACcp1_228 = qdd(2)-OMcp1_18*ORcp1_328+OMcp1_38*ORcp1_128-OPcp1_18*RLcp1_328+OPcp1_38*RLcp1_128;
    ACcp1_328 = qdd(3)+OMcp1_18*ORcp1_228-OMcp1_28*ORcp1_128+OPcp1_18*RLcp1_228-OPcp1_28*RLcp1_128;

% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp1_128;
    sens.P(2) = POcp1_228;
    sens.P(3) = POcp1_328;
    sens.R(1,1) = ROcp1_18;
    sens.R(1,2) = ROcp1_28;
    sens.R(1,3) = ROcp1_38;
    sens.R(2,1) = ROcp1_47;
    sens.R(2,2) = ROcp1_57;
    sens.R(2,3) = ROcp1_67;
    sens.R(3,1) = ROcp1_78;
    sens.R(3,2) = ROcp1_88;
    sens.R(3,3) = ROcp1_98;
    sens.V(1) = VIcp1_128;
    sens.V(2) = VIcp1_228;
    sens.V(3) = VIcp1_328;
    sens.OM(1) = OMcp1_18;
    sens.OM(2) = OMcp1_28;
    sens.OM(3) = OMcp1_38;
    sens.A(1) = ACcp1_128;
    sens.A(2) = ACcp1_228;
    sens.A(3) = ACcp1_328;
    sens.OMP(1) = OPcp1_18;
    sens.OMP(2) = OPcp1_28;
    sens.OMP(3) = OPcp1_38;
 
% 
case 3, 


% = = Block_1_0_0_3_0_1 = = 
 
% Sensor Kinematics 


    ROcp2_25 = S4*S5;
    ROcp2_35 = -C4*S5;
    ROcp2_85 = -S4*C5;
    ROcp2_95 = C4*C5;
    ROcp2_16 = C5*C6;
    ROcp2_26 = ROcp2_25*C6+C4*S6;
    ROcp2_36 = ROcp2_35*C6+S4*S6;
    ROcp2_46 = -C5*S6;
    ROcp2_56 = -(ROcp2_25*S6-C4*C6);
    ROcp2_66 = -(ROcp2_35*S6-S4*C6);
    OMcp2_25 = qd(5)*C4;
    OMcp2_35 = qd(5)*S4;
    OMcp2_16 = qd(4)+qd(6)*S5;
    OMcp2_26 = OMcp2_25+ROcp2_85*qd(6);
    OMcp2_36 = OMcp2_35+ROcp2_95*qd(6);
    OPcp2_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp2_26 = ROcp2_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp2_35*S5-ROcp2_95*qd(4));
    OPcp2_36 = ROcp2_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp2_25*S5-ROcp2_85*qd(4));

% = = Block_1_0_0_3_0_3 = = 
 
% Sensor Kinematics 


    ROcp2_19 = ROcp2_16*C9-S5*S9;
    ROcp2_29 = ROcp2_26*C9-ROcp2_85*S9;
    ROcp2_39 = ROcp2_36*C9-ROcp2_95*S9;
    ROcp2_79 = ROcp2_16*S9+S5*C9;
    ROcp2_89 = ROcp2_26*S9+ROcp2_85*C9;
    ROcp2_99 = ROcp2_36*S9+ROcp2_95*C9;
    ROcp2_410 = ROcp2_46*C10+ROcp2_79*S10;
    ROcp2_510 = ROcp2_56*C10+ROcp2_89*S10;
    ROcp2_610 = ROcp2_66*C10+ROcp2_99*S10;
    ROcp2_710 = -(ROcp2_46*S10-ROcp2_79*C10);
    ROcp2_810 = -(ROcp2_56*S10-ROcp2_89*C10);
    ROcp2_910 = -(ROcp2_66*S10-ROcp2_99*C10);
    RLcp2_19 = ROcp2_16*s.dpt(1,1);
    RLcp2_29 = ROcp2_26*s.dpt(1,1);
    RLcp2_39 = ROcp2_36*s.dpt(1,1);
    OMcp2_19 = OMcp2_16+ROcp2_46*qd(9);
    OMcp2_29 = OMcp2_26+ROcp2_56*qd(9);
    OMcp2_39 = OMcp2_36+ROcp2_66*qd(9);
    ORcp2_19 = OMcp2_26*RLcp2_39-OMcp2_36*RLcp2_29;
    ORcp2_29 = -(OMcp2_16*RLcp2_39-OMcp2_36*RLcp2_19);
    ORcp2_39 = OMcp2_16*RLcp2_29-OMcp2_26*RLcp2_19;
    OMcp2_110 = OMcp2_19+ROcp2_19*qd(10);
    OMcp2_210 = OMcp2_29+ROcp2_29*qd(10);
    OMcp2_310 = OMcp2_39+ROcp2_39*qd(10);
    OPcp2_110 = OPcp2_16+ROcp2_19*qdd(10)+ROcp2_46*qdd(9)+qd(10)*(OMcp2_29*ROcp2_39-OMcp2_39*ROcp2_29)+qd(9)*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56);
    OPcp2_210 = OPcp2_26+ROcp2_29*qdd(10)+ROcp2_56*qdd(9)-qd(10)*(OMcp2_19*ROcp2_39-OMcp2_39*ROcp2_19)-qd(9)*(OMcp2_16*ROcp2_66-OMcp2_36*ROcp2_46);
    OPcp2_310 = OPcp2_36+ROcp2_39*qdd(10)+ROcp2_66*qdd(9)+qd(10)*(OMcp2_19*ROcp2_29-OMcp2_29*ROcp2_19)+qd(9)*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46);
    RLcp2_129 = ROcp2_710*s.dpt(3,11);
    RLcp2_229 = ROcp2_810*s.dpt(3,11);
    RLcp2_329 = ROcp2_910*s.dpt(3,11);
    POcp2_129 = RLcp2_129+RLcp2_19+q(1);
    POcp2_229 = RLcp2_229+RLcp2_29+q(2);
    POcp2_329 = RLcp2_329+RLcp2_39+q(3);
    ORcp2_129 = OMcp2_210*RLcp2_329-OMcp2_310*RLcp2_229;
    ORcp2_229 = -(OMcp2_110*RLcp2_329-OMcp2_310*RLcp2_129);
    ORcp2_329 = OMcp2_110*RLcp2_229-OMcp2_210*RLcp2_129;
    VIcp2_129 = ORcp2_129+ORcp2_19+qd(1);
    VIcp2_229 = ORcp2_229+ORcp2_29+qd(2);
    VIcp2_329 = ORcp2_329+ORcp2_39+qd(3);
    ACcp2_129 = qdd(1)+OMcp2_210*ORcp2_329+OMcp2_26*ORcp2_39-OMcp2_310*ORcp2_229-OMcp2_36*ORcp2_29+OPcp2_210*RLcp2_329+OPcp2_26*RLcp2_39-OPcp2_310*...
 RLcp2_229-OPcp2_36*RLcp2_29;
    ACcp2_229 = qdd(2)-OMcp2_110*ORcp2_329-OMcp2_16*ORcp2_39+OMcp2_310*ORcp2_129+OMcp2_36*ORcp2_19-OPcp2_110*RLcp2_329-OPcp2_16*RLcp2_39+OPcp2_310*...
 RLcp2_129+OPcp2_36*RLcp2_19;
    ACcp2_329 = qdd(3)+OMcp2_110*ORcp2_229+OMcp2_16*ORcp2_29-OMcp2_210*ORcp2_129-OMcp2_26*ORcp2_19+OPcp2_110*RLcp2_229+OPcp2_16*RLcp2_29-OPcp2_210*...
 RLcp2_129-OPcp2_26*RLcp2_19;

% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp2_129;
    sens.P(2) = POcp2_229;
    sens.P(3) = POcp2_329;
    sens.R(1,1) = ROcp2_19;
    sens.R(1,2) = ROcp2_29;
    sens.R(1,3) = ROcp2_39;
    sens.R(2,1) = ROcp2_410;
    sens.R(2,2) = ROcp2_510;
    sens.R(2,3) = ROcp2_610;
    sens.R(3,1) = ROcp2_710;
    sens.R(3,2) = ROcp2_810;
    sens.R(3,3) = ROcp2_910;
    sens.V(1) = VIcp2_129;
    sens.V(2) = VIcp2_229;
    sens.V(3) = VIcp2_329;
    sens.OM(1) = OMcp2_110;
    sens.OM(2) = OMcp2_210;
    sens.OM(3) = OMcp2_310;
    sens.A(1) = ACcp2_129;
    sens.A(2) = ACcp2_229;
    sens.A(3) = ACcp2_329;
    sens.OMP(1) = OPcp2_110;
    sens.OMP(2) = OPcp2_210;
    sens.OMP(3) = OPcp2_310;
 
% 
case 4, 


% = = Block_1_0_0_4_0_1 = = 
 
% Sensor Kinematics 


    ROcp3_25 = S4*S5;
    ROcp3_35 = -C4*S5;
    ROcp3_85 = -S4*C5;
    ROcp3_95 = C4*C5;
    ROcp3_16 = C5*C6;
    ROcp3_26 = ROcp3_25*C6+C4*S6;
    ROcp3_36 = ROcp3_35*C6+S4*S6;
    ROcp3_46 = -C5*S6;
    ROcp3_56 = -(ROcp3_25*S6-C4*C6);
    ROcp3_66 = -(ROcp3_35*S6-S4*C6);
    OMcp3_25 = qd(5)*C4;
    OMcp3_35 = qd(5)*S4;
    OMcp3_16 = qd(4)+qd(6)*S5;
    OMcp3_26 = OMcp3_25+ROcp3_85*qd(6);
    OMcp3_36 = OMcp3_35+ROcp3_95*qd(6);
    OPcp3_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp3_26 = ROcp3_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp3_35*S5-ROcp3_95*qd(4));
    OPcp3_36 = ROcp3_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp3_25*S5-ROcp3_85*qd(4));

% = = Block_1_0_0_4_0_4 = = 
 
% Sensor Kinematics 


    ROcp3_111 = ROcp3_16*C11-S11*S5;
    ROcp3_211 = ROcp3_26*C11-ROcp3_85*S11;
    ROcp3_311 = ROcp3_36*C11-ROcp3_95*S11;
    ROcp3_711 = ROcp3_16*S11+C11*S5;
    ROcp3_811 = ROcp3_26*S11+ROcp3_85*C11;
    ROcp3_911 = ROcp3_36*S11+ROcp3_95*C11;
    ROcp3_412 = ROcp3_46*C12+ROcp3_711*S12;
    ROcp3_512 = ROcp3_56*C12+ROcp3_811*S12;
    ROcp3_612 = ROcp3_66*C12+ROcp3_911*S12;
    ROcp3_712 = -(ROcp3_46*S12-ROcp3_711*C12);
    ROcp3_812 = -(ROcp3_56*S12-ROcp3_811*C12);
    ROcp3_912 = -(ROcp3_66*S12-ROcp3_911*C12);
    RLcp3_111 = ROcp3_46*s.dpt(2,2);
    RLcp3_211 = ROcp3_56*s.dpt(2,2);
    RLcp3_311 = ROcp3_66*s.dpt(2,2);
    OMcp3_111 = OMcp3_16+ROcp3_46*qd(11);
    OMcp3_211 = OMcp3_26+ROcp3_56*qd(11);
    OMcp3_311 = OMcp3_36+ROcp3_66*qd(11);
    ORcp3_111 = OMcp3_26*RLcp3_311-OMcp3_36*RLcp3_211;
    ORcp3_211 = -(OMcp3_16*RLcp3_311-OMcp3_36*RLcp3_111);
    ORcp3_311 = OMcp3_16*RLcp3_211-OMcp3_26*RLcp3_111;
    OMcp3_112 = OMcp3_111+ROcp3_111*qd(12);
    OMcp3_212 = OMcp3_211+ROcp3_211*qd(12);
    OMcp3_312 = OMcp3_311+ROcp3_311*qd(12);
    OPcp3_112 = OPcp3_16+ROcp3_111*qdd(12)+ROcp3_46*qdd(11)+qd(11)*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)+qd(12)*(OMcp3_211*ROcp3_311-OMcp3_311*...
 ROcp3_211);
    OPcp3_212 = OPcp3_26+ROcp3_211*qdd(12)+ROcp3_56*qdd(11)-qd(11)*(OMcp3_16*ROcp3_66-OMcp3_36*ROcp3_46)-qd(12)*(OMcp3_111*ROcp3_311-OMcp3_311*...
 ROcp3_111);
    OPcp3_312 = OPcp3_36+ROcp3_311*qdd(12)+ROcp3_66*qdd(11)+qd(11)*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)+qd(12)*(OMcp3_111*ROcp3_211-OMcp3_211*...
 ROcp3_111);
    RLcp3_130 = ROcp3_712*s.dpt(3,12);
    RLcp3_230 = ROcp3_812*s.dpt(3,12);
    RLcp3_330 = ROcp3_912*s.dpt(3,12);
    POcp3_130 = RLcp3_111+RLcp3_130+q(1);
    POcp3_230 = RLcp3_211+RLcp3_230+q(2);
    POcp3_330 = RLcp3_311+RLcp3_330+q(3);
    ORcp3_130 = OMcp3_212*RLcp3_330-OMcp3_312*RLcp3_230;
    ORcp3_230 = -(OMcp3_112*RLcp3_330-OMcp3_312*RLcp3_130);
    ORcp3_330 = OMcp3_112*RLcp3_230-OMcp3_212*RLcp3_130;
    VIcp3_130 = ORcp3_111+ORcp3_130+qd(1);
    VIcp3_230 = ORcp3_211+ORcp3_230+qd(2);
    VIcp3_330 = ORcp3_311+ORcp3_330+qd(3);
    ACcp3_130 = qdd(1)+OMcp3_212*ORcp3_330+OMcp3_26*ORcp3_311-OMcp3_312*ORcp3_230-OMcp3_36*ORcp3_211+OPcp3_212*RLcp3_330+OPcp3_26*RLcp3_311-...
 OPcp3_312*RLcp3_230-OPcp3_36*RLcp3_211;
    ACcp3_230 = qdd(2)-OMcp3_112*ORcp3_330-OMcp3_16*ORcp3_311+OMcp3_312*ORcp3_130+OMcp3_36*ORcp3_111-OPcp3_112*RLcp3_330-OPcp3_16*RLcp3_311+...
 OPcp3_312*RLcp3_130+OPcp3_36*RLcp3_111;
    ACcp3_330 = qdd(3)+OMcp3_112*ORcp3_230+OMcp3_16*ORcp3_211-OMcp3_212*ORcp3_130-OMcp3_26*ORcp3_111+OPcp3_112*RLcp3_230+OPcp3_16*RLcp3_211-...
 OPcp3_212*RLcp3_130-OPcp3_26*RLcp3_111;

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp3_130;
    sens.P(2) = POcp3_230;
    sens.P(3) = POcp3_330;
    sens.R(1,1) = ROcp3_111;
    sens.R(1,2) = ROcp3_211;
    sens.R(1,3) = ROcp3_311;
    sens.R(2,1) = ROcp3_412;
    sens.R(2,2) = ROcp3_512;
    sens.R(2,3) = ROcp3_612;
    sens.R(3,1) = ROcp3_712;
    sens.R(3,2) = ROcp3_812;
    sens.R(3,3) = ROcp3_912;
    sens.V(1) = VIcp3_130;
    sens.V(2) = VIcp3_230;
    sens.V(3) = VIcp3_330;
    sens.OM(1) = OMcp3_112;
    sens.OM(2) = OMcp3_212;
    sens.OM(3) = OMcp3_312;
    sens.A(1) = ACcp3_130;
    sens.A(2) = ACcp3_230;
    sens.A(3) = ACcp3_330;
    sens.OMP(1) = OPcp3_112;
    sens.OMP(2) = OPcp3_212;
    sens.OMP(3) = OPcp3_312;
 
% 
case 5, 


% = = Block_1_0_0_5_0_1 = = 
 
% Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    ROcp4_16 = C5*C6;
    ROcp4_26 = ROcp4_25*C6+C4*S6;
    ROcp4_36 = ROcp4_35*C6+S4*S6;
    ROcp4_46 = -C5*S6;
    ROcp4_56 = -(ROcp4_25*S6-C4*C6);
    ROcp4_66 = -(ROcp4_35*S6-S4*C6);
    OMcp4_25 = qd(5)*C4;
    OMcp4_35 = qd(5)*S4;
    OMcp4_16 = qd(4)+qd(6)*S5;
    OMcp4_26 = OMcp4_25+ROcp4_85*qd(6);
    OMcp4_36 = OMcp4_35+ROcp4_95*qd(6);
    OPcp4_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp4_26 = ROcp4_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp4_35*S5-ROcp4_95*qd(4));
    OPcp4_36 = ROcp4_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp4_25*S5-ROcp4_85*qd(4));

% = = Block_1_0_0_5_0_5 = = 
 
% Sensor Kinematics 


    ROcp4_113 = ROcp4_16*C13-S13*S5;
    ROcp4_213 = ROcp4_26*C13-ROcp4_85*S13;
    ROcp4_313 = ROcp4_36*C13-ROcp4_95*S13;
    ROcp4_713 = ROcp4_16*S13+C13*S5;
    ROcp4_813 = ROcp4_26*S13+ROcp4_85*C13;
    ROcp4_913 = ROcp4_36*S13+ROcp4_95*C13;
    ROcp4_414 = ROcp4_46*C14+ROcp4_713*S14;
    ROcp4_514 = ROcp4_56*C14+ROcp4_813*S14;
    ROcp4_614 = ROcp4_66*C14+ROcp4_913*S14;
    ROcp4_714 = -(ROcp4_46*S14-ROcp4_713*C14);
    ROcp4_814 = -(ROcp4_56*S14-ROcp4_813*C14);
    ROcp4_914 = -(ROcp4_66*S14-ROcp4_913*C14);
    RLcp4_113 = ROcp4_16*s.dpt(1,3);
    RLcp4_213 = ROcp4_26*s.dpt(1,3);
    RLcp4_313 = ROcp4_36*s.dpt(1,3);
    OMcp4_113 = OMcp4_16+ROcp4_46*qd(13);
    OMcp4_213 = OMcp4_26+ROcp4_56*qd(13);
    OMcp4_313 = OMcp4_36+ROcp4_66*qd(13);
    ORcp4_113 = OMcp4_26*RLcp4_313-OMcp4_36*RLcp4_213;
    ORcp4_213 = -(OMcp4_16*RLcp4_313-OMcp4_36*RLcp4_113);
    ORcp4_313 = OMcp4_16*RLcp4_213-OMcp4_26*RLcp4_113;
    OMcp4_114 = OMcp4_113+ROcp4_113*qd(14);
    OMcp4_214 = OMcp4_213+ROcp4_213*qd(14);
    OMcp4_314 = OMcp4_313+ROcp4_313*qd(14);
    OPcp4_114 = OPcp4_16+ROcp4_113*qdd(14)+ROcp4_46*qdd(13)+qd(13)*(OMcp4_26*ROcp4_66-OMcp4_36*ROcp4_56)+qd(14)*(OMcp4_213*ROcp4_313-OMcp4_313*...
 ROcp4_213);
    OPcp4_214 = OPcp4_26+ROcp4_213*qdd(14)+ROcp4_56*qdd(13)-qd(13)*(OMcp4_16*ROcp4_66-OMcp4_36*ROcp4_46)-qd(14)*(OMcp4_113*ROcp4_313-OMcp4_313*...
 ROcp4_113);
    OPcp4_314 = OPcp4_36+ROcp4_313*qdd(14)+ROcp4_66*qdd(13)+qd(13)*(OMcp4_16*ROcp4_56-OMcp4_26*ROcp4_46)+qd(14)*(OMcp4_113*ROcp4_213-OMcp4_213*...
 ROcp4_113);
    RLcp4_131 = ROcp4_714*s.dpt(3,13);
    RLcp4_231 = ROcp4_814*s.dpt(3,13);
    RLcp4_331 = ROcp4_914*s.dpt(3,13);
    POcp4_131 = RLcp4_113+RLcp4_131+q(1);
    POcp4_231 = RLcp4_213+RLcp4_231+q(2);
    POcp4_331 = RLcp4_313+RLcp4_331+q(3);
    ORcp4_131 = OMcp4_214*RLcp4_331-OMcp4_314*RLcp4_231;
    ORcp4_231 = -(OMcp4_114*RLcp4_331-OMcp4_314*RLcp4_131);
    ORcp4_331 = OMcp4_114*RLcp4_231-OMcp4_214*RLcp4_131;
    VIcp4_131 = ORcp4_113+ORcp4_131+qd(1);
    VIcp4_231 = ORcp4_213+ORcp4_231+qd(2);
    VIcp4_331 = ORcp4_313+ORcp4_331+qd(3);
    ACcp4_131 = qdd(1)+OMcp4_214*ORcp4_331+OMcp4_26*ORcp4_313-OMcp4_314*ORcp4_231-OMcp4_36*ORcp4_213+OPcp4_214*RLcp4_331+OPcp4_26*RLcp4_313-...
 OPcp4_314*RLcp4_231-OPcp4_36*RLcp4_213;
    ACcp4_231 = qdd(2)-OMcp4_114*ORcp4_331-OMcp4_16*ORcp4_313+OMcp4_314*ORcp4_131+OMcp4_36*ORcp4_113-OPcp4_114*RLcp4_331-OPcp4_16*RLcp4_313+...
 OPcp4_314*RLcp4_131+OPcp4_36*RLcp4_113;
    ACcp4_331 = qdd(3)+OMcp4_114*ORcp4_231+OMcp4_16*ORcp4_213-OMcp4_214*ORcp4_131-OMcp4_26*ORcp4_113+OPcp4_114*RLcp4_231+OPcp4_16*RLcp4_213-...
 OPcp4_214*RLcp4_131-OPcp4_26*RLcp4_113;

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp4_131;
    sens.P(2) = POcp4_231;
    sens.P(3) = POcp4_331;
    sens.R(1,1) = ROcp4_113;
    sens.R(1,2) = ROcp4_213;
    sens.R(1,3) = ROcp4_313;
    sens.R(2,1) = ROcp4_414;
    sens.R(2,2) = ROcp4_514;
    sens.R(2,3) = ROcp4_614;
    sens.R(3,1) = ROcp4_714;
    sens.R(3,2) = ROcp4_814;
    sens.R(3,3) = ROcp4_914;
    sens.V(1) = VIcp4_131;
    sens.V(2) = VIcp4_231;
    sens.V(3) = VIcp4_331;
    sens.OM(1) = OMcp4_114;
    sens.OM(2) = OMcp4_214;
    sens.OM(3) = OMcp4_314;
    sens.A(1) = ACcp4_131;
    sens.A(2) = ACcp4_231;
    sens.A(3) = ACcp4_331;
    sens.OMP(1) = OPcp4_114;
    sens.OMP(2) = OPcp4_214;
    sens.OMP(3) = OPcp4_314;
 
% 
case 6, 


% = = Block_1_0_0_6_0_1 = = 
 
% Sensor Kinematics 


    ROcp5_25 = S4*S5;
    ROcp5_35 = -C4*S5;
    ROcp5_85 = -S4*C5;
    ROcp5_95 = C4*C5;
    ROcp5_16 = C5*C6;
    ROcp5_26 = ROcp5_25*C6+C4*S6;
    ROcp5_36 = ROcp5_35*C6+S4*S6;
    ROcp5_46 = -C5*S6;
    ROcp5_56 = -(ROcp5_25*S6-C4*C6);
    ROcp5_66 = -(ROcp5_35*S6-S4*C6);
    OMcp5_25 = qd(5)*C4;
    OMcp5_35 = qd(5)*S4;
    OMcp5_16 = qd(4)+qd(6)*S5;
    OMcp5_26 = OMcp5_25+ROcp5_85*qd(6);
    OMcp5_36 = OMcp5_35+ROcp5_95*qd(6);
    OPcp5_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp5_26 = ROcp5_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp5_35*S5-ROcp5_95*qd(4));
    OPcp5_36 = ROcp5_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp5_25*S5-ROcp5_85*qd(4));

% = = Block_1_0_0_6_0_6 = = 
 
% Sensor Kinematics 


    ROcp5_115 = ROcp5_16*C15-S15*S5;
    ROcp5_215 = ROcp5_26*C15-ROcp5_85*S15;
    ROcp5_315 = ROcp5_36*C15-ROcp5_95*S15;
    ROcp5_715 = ROcp5_16*S15+C15*S5;
    ROcp5_815 = ROcp5_26*S15+ROcp5_85*C15;
    ROcp5_915 = ROcp5_36*S15+ROcp5_95*C15;
    ROcp5_416 = ROcp5_46*C16+ROcp5_715*S16;
    ROcp5_516 = ROcp5_56*C16+ROcp5_815*S16;
    ROcp5_616 = ROcp5_66*C16+ROcp5_915*S16;
    ROcp5_716 = -(ROcp5_46*S16-ROcp5_715*C16);
    ROcp5_816 = -(ROcp5_56*S16-ROcp5_815*C16);
    ROcp5_916 = -(ROcp5_66*S16-ROcp5_915*C16);
    RLcp5_115 = ROcp5_46*s.dpt(2,4);
    RLcp5_215 = ROcp5_56*s.dpt(2,4);
    RLcp5_315 = ROcp5_66*s.dpt(2,4);
    OMcp5_115 = OMcp5_16+ROcp5_46*qd(15);
    OMcp5_215 = OMcp5_26+ROcp5_56*qd(15);
    OMcp5_315 = OMcp5_36+ROcp5_66*qd(15);
    ORcp5_115 = OMcp5_26*RLcp5_315-OMcp5_36*RLcp5_215;
    ORcp5_215 = -(OMcp5_16*RLcp5_315-OMcp5_36*RLcp5_115);
    ORcp5_315 = OMcp5_16*RLcp5_215-OMcp5_26*RLcp5_115;
    OMcp5_116 = OMcp5_115+ROcp5_115*qd(16);
    OMcp5_216 = OMcp5_215+ROcp5_215*qd(16);
    OMcp5_316 = OMcp5_315+ROcp5_315*qd(16);
    OPcp5_116 = OPcp5_16+ROcp5_115*qdd(16)+ROcp5_46*qdd(15)+qd(15)*(OMcp5_26*ROcp5_66-OMcp5_36*ROcp5_56)+qd(16)*(OMcp5_215*ROcp5_315-OMcp5_315*...
 ROcp5_215);
    OPcp5_216 = OPcp5_26+ROcp5_215*qdd(16)+ROcp5_56*qdd(15)-qd(15)*(OMcp5_16*ROcp5_66-OMcp5_36*ROcp5_46)-qd(16)*(OMcp5_115*ROcp5_315-OMcp5_315*...
 ROcp5_115);
    OPcp5_316 = OPcp5_36+ROcp5_315*qdd(16)+ROcp5_66*qdd(15)+qd(15)*(OMcp5_16*ROcp5_56-OMcp5_26*ROcp5_46)+qd(16)*(OMcp5_115*ROcp5_215-OMcp5_215*...
 ROcp5_115);
    RLcp5_132 = ROcp5_716*s.dpt(3,14);
    RLcp5_232 = ROcp5_816*s.dpt(3,14);
    RLcp5_332 = ROcp5_916*s.dpt(3,14);
    POcp5_132 = RLcp5_115+RLcp5_132+q(1);
    POcp5_232 = RLcp5_215+RLcp5_232+q(2);
    POcp5_332 = RLcp5_315+RLcp5_332+q(3);
    ORcp5_132 = OMcp5_216*RLcp5_332-OMcp5_316*RLcp5_232;
    ORcp5_232 = -(OMcp5_116*RLcp5_332-OMcp5_316*RLcp5_132);
    ORcp5_332 = OMcp5_116*RLcp5_232-OMcp5_216*RLcp5_132;
    VIcp5_132 = ORcp5_115+ORcp5_132+qd(1);
    VIcp5_232 = ORcp5_215+ORcp5_232+qd(2);
    VIcp5_332 = ORcp5_315+ORcp5_332+qd(3);
    ACcp5_132 = qdd(1)+OMcp5_216*ORcp5_332+OMcp5_26*ORcp5_315-OMcp5_316*ORcp5_232-OMcp5_36*ORcp5_215+OPcp5_216*RLcp5_332+OPcp5_26*RLcp5_315-...
 OPcp5_316*RLcp5_232-OPcp5_36*RLcp5_215;
    ACcp5_232 = qdd(2)-OMcp5_116*ORcp5_332-OMcp5_16*ORcp5_315+OMcp5_316*ORcp5_132+OMcp5_36*ORcp5_115-OPcp5_116*RLcp5_332-OPcp5_16*RLcp5_315+...
 OPcp5_316*RLcp5_132+OPcp5_36*RLcp5_115;
    ACcp5_332 = qdd(3)+OMcp5_116*ORcp5_232+OMcp5_16*ORcp5_215-OMcp5_216*ORcp5_132-OMcp5_26*ORcp5_115+OPcp5_116*RLcp5_232+OPcp5_16*RLcp5_215-...
 OPcp5_216*RLcp5_132-OPcp5_26*RLcp5_115;

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp5_132;
    sens.P(2) = POcp5_232;
    sens.P(3) = POcp5_332;
    sens.R(1,1) = ROcp5_115;
    sens.R(1,2) = ROcp5_215;
    sens.R(1,3) = ROcp5_315;
    sens.R(2,1) = ROcp5_416;
    sens.R(2,2) = ROcp5_516;
    sens.R(2,3) = ROcp5_616;
    sens.R(3,1) = ROcp5_716;
    sens.R(3,2) = ROcp5_816;
    sens.R(3,3) = ROcp5_916;
    sens.V(1) = VIcp5_132;
    sens.V(2) = VIcp5_232;
    sens.V(3) = VIcp5_332;
    sens.OM(1) = OMcp5_116;
    sens.OM(2) = OMcp5_216;
    sens.OM(3) = OMcp5_316;
    sens.A(1) = ACcp5_132;
    sens.A(2) = ACcp5_232;
    sens.A(3) = ACcp5_332;
    sens.OMP(1) = OPcp5_116;
    sens.OMP(2) = OPcp5_216;
    sens.OMP(3) = OPcp5_316;

end


% ====== END Task 1 ====== 

  

