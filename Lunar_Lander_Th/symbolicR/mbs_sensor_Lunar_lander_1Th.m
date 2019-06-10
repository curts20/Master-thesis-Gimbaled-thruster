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
%	==> Generation Date : Sun Nov 11 18:50:07 2018
%
%	==> Project name : Lunar_lander_1Th
%	==> using XML input file 
%
%	==> Number of joints : 16
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 1168
%
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.010 seconds
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
 sens.J = zeros(6,16);

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

% = = Block_1_0_0_1_0_2 = = 
 
% Sensor Kinematics 


    ROcp0_47 = ROcp0_46*C7+S5*S7;
    ROcp0_57 = ROcp0_56*C7+ROcp0_85*S7;
    ROcp0_67 = ROcp0_66*C7+ROcp0_95*S7;
    ROcp0_77 = -(ROcp0_46*S7-S5*C7);
    ROcp0_87 = -(ROcp0_56*S7-ROcp0_85*C7);
    ROcp0_97 = -(ROcp0_66*S7-ROcp0_95*C7);
    ROcp0_18 = ROcp0_16*C8-ROcp0_77*S8;
    ROcp0_28 = ROcp0_26*C8-ROcp0_87*S8;
    ROcp0_38 = ROcp0_36*C8-ROcp0_97*S8;
    ROcp0_78 = ROcp0_16*S8+ROcp0_77*C8;
    ROcp0_88 = ROcp0_26*S8+ROcp0_87*C8;
    ROcp0_98 = ROcp0_36*S8+ROcp0_97*C8;
    OMcp0_17 = OMcp0_16+ROcp0_16*qd(7);
    OMcp0_27 = OMcp0_26+ROcp0_26*qd(7);
    OMcp0_37 = OMcp0_36+ROcp0_36*qd(7);
    OMcp0_18 = OMcp0_17+ROcp0_47*qd(8);
    OMcp0_28 = OMcp0_27+ROcp0_57*qd(8);
    OMcp0_38 = OMcp0_37+ROcp0_67*qd(8);
    OPcp0_18 = qdd(4)+ROcp0_16*qdd(7)+ROcp0_47*qdd(8)+qdd(6)*S5+qd(5)*qd(6)*C5+qd(7)*(OMcp0_26*ROcp0_36-OMcp0_36*ROcp0_26)+qd(8)*(OMcp0_27*ROcp0_67...
 -OMcp0_37*ROcp0_57);
    OPcp0_28 = ROcp0_26*qdd(7)+ROcp0_57*qdd(8)+ROcp0_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp0_35*S5-ROcp0_95*qd(4))-qd(7)*(OMcp0_16*ROcp0_36...
 -OMcp0_36*ROcp0_16)-qd(8)*(OMcp0_17*ROcp0_67-OMcp0_37*ROcp0_47);
    OPcp0_38 = ROcp0_36*qdd(7)+ROcp0_67*qdd(8)+ROcp0_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp0_25*S5-ROcp0_85*qd(4))+qd(7)*(OMcp0_16*ROcp0_26...
 -OMcp0_26*ROcp0_16)+qd(8)*(OMcp0_17*ROcp0_57-OMcp0_27*ROcp0_47);
    RLcp0_137 = ROcp0_78*s.dpt(3,17);
    RLcp0_237 = ROcp0_88*s.dpt(3,17);
    RLcp0_337 = ROcp0_98*s.dpt(3,17);
    POcp0_137 = RLcp0_137+q(1);
    POcp0_237 = RLcp0_237+q(2);
    POcp0_337 = RLcp0_337+q(3);
    ORcp0_137 = OMcp0_28*RLcp0_337-OMcp0_38*RLcp0_237;
    ORcp0_237 = -(OMcp0_18*RLcp0_337-OMcp0_38*RLcp0_137);
    ORcp0_337 = OMcp0_18*RLcp0_237-OMcp0_28*RLcp0_137;
    VIcp0_137 = ORcp0_137+qd(1);
    VIcp0_237 = ORcp0_237+qd(2);
    VIcp0_337 = ORcp0_337+qd(3);
    ACcp0_137 = qdd(1)+OMcp0_28*ORcp0_337-OMcp0_38*ORcp0_237+OPcp0_28*RLcp0_337-OPcp0_38*RLcp0_237;
    ACcp0_237 = qdd(2)-OMcp0_18*ORcp0_337+OMcp0_38*ORcp0_137-OPcp0_18*RLcp0_337+OPcp0_38*RLcp0_137;
    ACcp0_337 = qdd(3)+OMcp0_18*ORcp0_237-OMcp0_28*ORcp0_137+OPcp0_18*RLcp0_237-OPcp0_28*RLcp0_137;

% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp0_137;
    sens.P(2) = POcp0_237;
    sens.P(3) = POcp0_337;
    sens.R(1,1) = ROcp0_18;
    sens.R(1,2) = ROcp0_28;
    sens.R(1,3) = ROcp0_38;
    sens.R(2,1) = ROcp0_47;
    sens.R(2,2) = ROcp0_57;
    sens.R(2,3) = ROcp0_67;
    sens.R(3,1) = ROcp0_78;
    sens.R(3,2) = ROcp0_88;
    sens.R(3,3) = ROcp0_98;
    sens.V(1) = VIcp0_137;
    sens.V(2) = VIcp0_237;
    sens.V(3) = VIcp0_337;
    sens.OM(1) = OMcp0_18;
    sens.OM(2) = OMcp0_28;
    sens.OM(3) = OMcp0_38;
    sens.A(1) = ACcp0_137;
    sens.A(2) = ACcp0_237;
    sens.A(3) = ACcp0_337;
    sens.OMP(1) = OPcp0_18;
    sens.OMP(2) = OPcp0_28;
    sens.OMP(3) = OPcp0_38;
 
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
    OPcp1_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp1_26 = ROcp1_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp1_35*S5-ROcp1_95*qd(4));
    OPcp1_36 = ROcp1_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp1_25*S5-ROcp1_85*qd(4));

% = = Block_1_0_0_2_0_3 = = 
 
% Sensor Kinematics 


    ROcp1_19 = ROcp1_16*C9-S5*S9;
    ROcp1_29 = ROcp1_26*C9-ROcp1_85*S9;
    ROcp1_39 = ROcp1_36*C9-ROcp1_95*S9;
    ROcp1_79 = ROcp1_16*S9+S5*C9;
    ROcp1_89 = ROcp1_26*S9+ROcp1_85*C9;
    ROcp1_99 = ROcp1_36*S9+ROcp1_95*C9;
    ROcp1_410 = ROcp1_46*C10+ROcp1_79*S10;
    ROcp1_510 = ROcp1_56*C10+ROcp1_89*S10;
    ROcp1_610 = ROcp1_66*C10+ROcp1_99*S10;
    ROcp1_710 = -(ROcp1_46*S10-ROcp1_79*C10);
    ROcp1_810 = -(ROcp1_56*S10-ROcp1_89*C10);
    ROcp1_910 = -(ROcp1_66*S10-ROcp1_99*C10);
    RLcp1_19 = ROcp1_16*s.dpt(1,1);
    RLcp1_29 = ROcp1_26*s.dpt(1,1);
    RLcp1_39 = ROcp1_36*s.dpt(1,1);
    OMcp1_19 = OMcp1_16+ROcp1_46*qd(9);
    OMcp1_29 = OMcp1_26+ROcp1_56*qd(9);
    OMcp1_39 = OMcp1_36+ROcp1_66*qd(9);
    ORcp1_19 = OMcp1_26*RLcp1_39-OMcp1_36*RLcp1_29;
    ORcp1_29 = -(OMcp1_16*RLcp1_39-OMcp1_36*RLcp1_19);
    ORcp1_39 = OMcp1_16*RLcp1_29-OMcp1_26*RLcp1_19;
    OMcp1_110 = OMcp1_19+ROcp1_19*qd(10);
    OMcp1_210 = OMcp1_29+ROcp1_29*qd(10);
    OMcp1_310 = OMcp1_39+ROcp1_39*qd(10);
    OPcp1_110 = OPcp1_16+ROcp1_19*qdd(10)+ROcp1_46*qdd(9)+qd(10)*(OMcp1_29*ROcp1_39-OMcp1_39*ROcp1_29)+qd(9)*(OMcp1_26*ROcp1_66-OMcp1_36*ROcp1_56);
    OPcp1_210 = OPcp1_26+ROcp1_29*qdd(10)+ROcp1_56*qdd(9)-qd(10)*(OMcp1_19*ROcp1_39-OMcp1_39*ROcp1_19)-qd(9)*(OMcp1_16*ROcp1_66-OMcp1_36*ROcp1_46);
    OPcp1_310 = OPcp1_36+ROcp1_39*qdd(10)+ROcp1_66*qdd(9)+qd(10)*(OMcp1_19*ROcp1_29-OMcp1_29*ROcp1_19)+qd(9)*(OMcp1_16*ROcp1_56-OMcp1_26*ROcp1_46);
    RLcp1_138 = ROcp1_710*s.dpt(3,18);
    RLcp1_238 = ROcp1_810*s.dpt(3,18);
    RLcp1_338 = ROcp1_910*s.dpt(3,18);
    POcp1_138 = RLcp1_138+RLcp1_19+q(1);
    POcp1_238 = RLcp1_238+RLcp1_29+q(2);
    POcp1_338 = RLcp1_338+RLcp1_39+q(3);
    ORcp1_138 = OMcp1_210*RLcp1_338-OMcp1_310*RLcp1_238;
    ORcp1_238 = -(OMcp1_110*RLcp1_338-OMcp1_310*RLcp1_138);
    ORcp1_338 = OMcp1_110*RLcp1_238-OMcp1_210*RLcp1_138;
    VIcp1_138 = ORcp1_138+ORcp1_19+qd(1);
    VIcp1_238 = ORcp1_238+ORcp1_29+qd(2);
    VIcp1_338 = ORcp1_338+ORcp1_39+qd(3);
    ACcp1_138 = qdd(1)+OMcp1_210*ORcp1_338+OMcp1_26*ORcp1_39-OMcp1_310*ORcp1_238-OMcp1_36*ORcp1_29+OPcp1_210*RLcp1_338+OPcp1_26*RLcp1_39-OPcp1_310*...
 RLcp1_238-OPcp1_36*RLcp1_29;
    ACcp1_238 = qdd(2)-OMcp1_110*ORcp1_338-OMcp1_16*ORcp1_39+OMcp1_310*ORcp1_138+OMcp1_36*ORcp1_19-OPcp1_110*RLcp1_338-OPcp1_16*RLcp1_39+OPcp1_310*...
 RLcp1_138+OPcp1_36*RLcp1_19;
    ACcp1_338 = qdd(3)+OMcp1_110*ORcp1_238+OMcp1_16*ORcp1_29-OMcp1_210*ORcp1_138-OMcp1_26*ORcp1_19+OPcp1_110*RLcp1_238+OPcp1_16*RLcp1_29-OPcp1_210*...
 RLcp1_138-OPcp1_26*RLcp1_19;

% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp1_138;
    sens.P(2) = POcp1_238;
    sens.P(3) = POcp1_338;
    sens.R(1,1) = ROcp1_19;
    sens.R(1,2) = ROcp1_29;
    sens.R(1,3) = ROcp1_39;
    sens.R(2,1) = ROcp1_410;
    sens.R(2,2) = ROcp1_510;
    sens.R(2,3) = ROcp1_610;
    sens.R(3,1) = ROcp1_710;
    sens.R(3,2) = ROcp1_810;
    sens.R(3,3) = ROcp1_910;
    sens.V(1) = VIcp1_138;
    sens.V(2) = VIcp1_238;
    sens.V(3) = VIcp1_338;
    sens.OM(1) = OMcp1_110;
    sens.OM(2) = OMcp1_210;
    sens.OM(3) = OMcp1_310;
    sens.A(1) = ACcp1_138;
    sens.A(2) = ACcp1_238;
    sens.A(3) = ACcp1_338;
    sens.OMP(1) = OPcp1_110;
    sens.OMP(2) = OPcp1_210;
    sens.OMP(3) = OPcp1_310;
 
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

% = = Block_1_0_0_3_0_4 = = 
 
% Sensor Kinematics 


    ROcp2_111 = ROcp2_16*C11-S11*S5;
    ROcp2_211 = ROcp2_26*C11-ROcp2_85*S11;
    ROcp2_311 = ROcp2_36*C11-ROcp2_95*S11;
    ROcp2_711 = ROcp2_16*S11+C11*S5;
    ROcp2_811 = ROcp2_26*S11+ROcp2_85*C11;
    ROcp2_911 = ROcp2_36*S11+ROcp2_95*C11;
    ROcp2_412 = ROcp2_46*C12+ROcp2_711*S12;
    ROcp2_512 = ROcp2_56*C12+ROcp2_811*S12;
    ROcp2_612 = ROcp2_66*C12+ROcp2_911*S12;
    ROcp2_712 = -(ROcp2_46*S12-ROcp2_711*C12);
    ROcp2_812 = -(ROcp2_56*S12-ROcp2_811*C12);
    ROcp2_912 = -(ROcp2_66*S12-ROcp2_911*C12);
    RLcp2_111 = ROcp2_46*s.dpt(2,2);
    RLcp2_211 = ROcp2_56*s.dpt(2,2);
    RLcp2_311 = ROcp2_66*s.dpt(2,2);
    OMcp2_111 = OMcp2_16+ROcp2_46*qd(11);
    OMcp2_211 = OMcp2_26+ROcp2_56*qd(11);
    OMcp2_311 = OMcp2_36+ROcp2_66*qd(11);
    ORcp2_111 = OMcp2_26*RLcp2_311-OMcp2_36*RLcp2_211;
    ORcp2_211 = -(OMcp2_16*RLcp2_311-OMcp2_36*RLcp2_111);
    ORcp2_311 = OMcp2_16*RLcp2_211-OMcp2_26*RLcp2_111;
    OMcp2_112 = OMcp2_111+ROcp2_111*qd(12);
    OMcp2_212 = OMcp2_211+ROcp2_211*qd(12);
    OMcp2_312 = OMcp2_311+ROcp2_311*qd(12);
    OPcp2_112 = OPcp2_16+ROcp2_111*qdd(12)+ROcp2_46*qdd(11)+qd(11)*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56)+qd(12)*(OMcp2_211*ROcp2_311-OMcp2_311*...
 ROcp2_211);
    OPcp2_212 = OPcp2_26+ROcp2_211*qdd(12)+ROcp2_56*qdd(11)-qd(11)*(OMcp2_16*ROcp2_66-OMcp2_36*ROcp2_46)-qd(12)*(OMcp2_111*ROcp2_311-OMcp2_311*...
 ROcp2_111);
    OPcp2_312 = OPcp2_36+ROcp2_311*qdd(12)+ROcp2_66*qdd(11)+qd(11)*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46)+qd(12)*(OMcp2_111*ROcp2_211-OMcp2_211*...
 ROcp2_111);
    RLcp2_139 = ROcp2_712*s.dpt(3,20);
    RLcp2_239 = ROcp2_812*s.dpt(3,20);
    RLcp2_339 = ROcp2_912*s.dpt(3,20);
    POcp2_139 = RLcp2_111+RLcp2_139+q(1);
    POcp2_239 = RLcp2_211+RLcp2_239+q(2);
    POcp2_339 = RLcp2_311+RLcp2_339+q(3);
    ORcp2_139 = OMcp2_212*RLcp2_339-OMcp2_312*RLcp2_239;
    ORcp2_239 = -(OMcp2_112*RLcp2_339-OMcp2_312*RLcp2_139);
    ORcp2_339 = OMcp2_112*RLcp2_239-OMcp2_212*RLcp2_139;
    VIcp2_139 = ORcp2_111+ORcp2_139+qd(1);
    VIcp2_239 = ORcp2_211+ORcp2_239+qd(2);
    VIcp2_339 = ORcp2_311+ORcp2_339+qd(3);
    ACcp2_139 = qdd(1)+OMcp2_212*ORcp2_339+OMcp2_26*ORcp2_311-OMcp2_312*ORcp2_239-OMcp2_36*ORcp2_211+OPcp2_212*RLcp2_339+OPcp2_26*RLcp2_311-...
 OPcp2_312*RLcp2_239-OPcp2_36*RLcp2_211;
    ACcp2_239 = qdd(2)-OMcp2_112*ORcp2_339-OMcp2_16*ORcp2_311+OMcp2_312*ORcp2_139+OMcp2_36*ORcp2_111-OPcp2_112*RLcp2_339-OPcp2_16*RLcp2_311+...
 OPcp2_312*RLcp2_139+OPcp2_36*RLcp2_111;
    ACcp2_339 = qdd(3)+OMcp2_112*ORcp2_239+OMcp2_16*ORcp2_211-OMcp2_212*ORcp2_139-OMcp2_26*ORcp2_111+OPcp2_112*RLcp2_239+OPcp2_16*RLcp2_211-...
 OPcp2_212*RLcp2_139-OPcp2_26*RLcp2_111;

% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp2_139;
    sens.P(2) = POcp2_239;
    sens.P(3) = POcp2_339;
    sens.R(1,1) = ROcp2_111;
    sens.R(1,2) = ROcp2_211;
    sens.R(1,3) = ROcp2_311;
    sens.R(2,1) = ROcp2_412;
    sens.R(2,2) = ROcp2_512;
    sens.R(2,3) = ROcp2_612;
    sens.R(3,1) = ROcp2_712;
    sens.R(3,2) = ROcp2_812;
    sens.R(3,3) = ROcp2_912;
    sens.V(1) = VIcp2_139;
    sens.V(2) = VIcp2_239;
    sens.V(3) = VIcp2_339;
    sens.OM(1) = OMcp2_112;
    sens.OM(2) = OMcp2_212;
    sens.OM(3) = OMcp2_312;
    sens.A(1) = ACcp2_139;
    sens.A(2) = ACcp2_239;
    sens.A(3) = ACcp2_339;
    sens.OMP(1) = OPcp2_112;
    sens.OMP(2) = OPcp2_212;
    sens.OMP(3) = OPcp2_312;
 
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

% = = Block_1_0_0_4_0_5 = = 
 
% Sensor Kinematics 


    ROcp3_113 = ROcp3_16*C13-S13*S5;
    ROcp3_213 = ROcp3_26*C13-ROcp3_85*S13;
    ROcp3_313 = ROcp3_36*C13-ROcp3_95*S13;
    ROcp3_713 = ROcp3_16*S13+C13*S5;
    ROcp3_813 = ROcp3_26*S13+ROcp3_85*C13;
    ROcp3_913 = ROcp3_36*S13+ROcp3_95*C13;
    ROcp3_414 = ROcp3_46*C14+ROcp3_713*S14;
    ROcp3_514 = ROcp3_56*C14+ROcp3_813*S14;
    ROcp3_614 = ROcp3_66*C14+ROcp3_913*S14;
    ROcp3_714 = -(ROcp3_46*S14-ROcp3_713*C14);
    ROcp3_814 = -(ROcp3_56*S14-ROcp3_813*C14);
    ROcp3_914 = -(ROcp3_66*S14-ROcp3_913*C14);
    RLcp3_113 = ROcp3_16*s.dpt(1,3);
    RLcp3_213 = ROcp3_26*s.dpt(1,3);
    RLcp3_313 = ROcp3_36*s.dpt(1,3);
    OMcp3_113 = OMcp3_16+ROcp3_46*qd(13);
    OMcp3_213 = OMcp3_26+ROcp3_56*qd(13);
    OMcp3_313 = OMcp3_36+ROcp3_66*qd(13);
    ORcp3_113 = OMcp3_26*RLcp3_313-OMcp3_36*RLcp3_213;
    ORcp3_213 = -(OMcp3_16*RLcp3_313-OMcp3_36*RLcp3_113);
    ORcp3_313 = OMcp3_16*RLcp3_213-OMcp3_26*RLcp3_113;
    OMcp3_114 = OMcp3_113+ROcp3_113*qd(14);
    OMcp3_214 = OMcp3_213+ROcp3_213*qd(14);
    OMcp3_314 = OMcp3_313+ROcp3_313*qd(14);
    OPcp3_114 = OPcp3_16+ROcp3_113*qdd(14)+ROcp3_46*qdd(13)+qd(13)*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)+qd(14)*(OMcp3_213*ROcp3_313-OMcp3_313*...
 ROcp3_213);
    OPcp3_214 = OPcp3_26+ROcp3_213*qdd(14)+ROcp3_56*qdd(13)-qd(13)*(OMcp3_16*ROcp3_66-OMcp3_36*ROcp3_46)-qd(14)*(OMcp3_113*ROcp3_313-OMcp3_313*...
 ROcp3_113);
    OPcp3_314 = OPcp3_36+ROcp3_313*qdd(14)+ROcp3_66*qdd(13)+qd(13)*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)+qd(14)*(OMcp3_113*ROcp3_213-OMcp3_213*...
 ROcp3_113);
    RLcp3_140 = ROcp3_714*s.dpt(3,22);
    RLcp3_240 = ROcp3_814*s.dpt(3,22);
    RLcp3_340 = ROcp3_914*s.dpt(3,22);
    POcp3_140 = RLcp3_113+RLcp3_140+q(1);
    POcp3_240 = RLcp3_213+RLcp3_240+q(2);
    POcp3_340 = RLcp3_313+RLcp3_340+q(3);
    ORcp3_140 = OMcp3_214*RLcp3_340-OMcp3_314*RLcp3_240;
    ORcp3_240 = -(OMcp3_114*RLcp3_340-OMcp3_314*RLcp3_140);
    ORcp3_340 = OMcp3_114*RLcp3_240-OMcp3_214*RLcp3_140;
    VIcp3_140 = ORcp3_113+ORcp3_140+qd(1);
    VIcp3_240 = ORcp3_213+ORcp3_240+qd(2);
    VIcp3_340 = ORcp3_313+ORcp3_340+qd(3);
    ACcp3_140 = qdd(1)+OMcp3_214*ORcp3_340+OMcp3_26*ORcp3_313-OMcp3_314*ORcp3_240-OMcp3_36*ORcp3_213+OPcp3_214*RLcp3_340+OPcp3_26*RLcp3_313-...
 OPcp3_314*RLcp3_240-OPcp3_36*RLcp3_213;
    ACcp3_240 = qdd(2)-OMcp3_114*ORcp3_340-OMcp3_16*ORcp3_313+OMcp3_314*ORcp3_140+OMcp3_36*ORcp3_113-OPcp3_114*RLcp3_340-OPcp3_16*RLcp3_313+...
 OPcp3_314*RLcp3_140+OPcp3_36*RLcp3_113;
    ACcp3_340 = qdd(3)+OMcp3_114*ORcp3_240+OMcp3_16*ORcp3_213-OMcp3_214*ORcp3_140-OMcp3_26*ORcp3_113+OPcp3_114*RLcp3_240+OPcp3_16*RLcp3_213-...
 OPcp3_214*RLcp3_140-OPcp3_26*RLcp3_113;

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp3_140;
    sens.P(2) = POcp3_240;
    sens.P(3) = POcp3_340;
    sens.R(1,1) = ROcp3_113;
    sens.R(1,2) = ROcp3_213;
    sens.R(1,3) = ROcp3_313;
    sens.R(2,1) = ROcp3_414;
    sens.R(2,2) = ROcp3_514;
    sens.R(2,3) = ROcp3_614;
    sens.R(3,1) = ROcp3_714;
    sens.R(3,2) = ROcp3_814;
    sens.R(3,3) = ROcp3_914;
    sens.V(1) = VIcp3_140;
    sens.V(2) = VIcp3_240;
    sens.V(3) = VIcp3_340;
    sens.OM(1) = OMcp3_114;
    sens.OM(2) = OMcp3_214;
    sens.OM(3) = OMcp3_314;
    sens.A(1) = ACcp3_140;
    sens.A(2) = ACcp3_240;
    sens.A(3) = ACcp3_340;
    sens.OMP(1) = OPcp3_114;
    sens.OMP(2) = OPcp3_214;
    sens.OMP(3) = OPcp3_314;
 
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

% = = Block_1_0_0_5_0_6 = = 
 
% Sensor Kinematics 


    ROcp4_115 = ROcp4_16*C15-S15*S5;
    ROcp4_215 = ROcp4_26*C15-ROcp4_85*S15;
    ROcp4_315 = ROcp4_36*C15-ROcp4_95*S15;
    ROcp4_715 = ROcp4_16*S15+C15*S5;
    ROcp4_815 = ROcp4_26*S15+ROcp4_85*C15;
    ROcp4_915 = ROcp4_36*S15+ROcp4_95*C15;
    ROcp4_416 = ROcp4_46*C16+ROcp4_715*S16;
    ROcp4_516 = ROcp4_56*C16+ROcp4_815*S16;
    ROcp4_616 = ROcp4_66*C16+ROcp4_915*S16;
    ROcp4_716 = -(ROcp4_46*S16-ROcp4_715*C16);
    ROcp4_816 = -(ROcp4_56*S16-ROcp4_815*C16);
    ROcp4_916 = -(ROcp4_66*S16-ROcp4_915*C16);
    RLcp4_115 = ROcp4_46*s.dpt(2,4);
    RLcp4_215 = ROcp4_56*s.dpt(2,4);
    RLcp4_315 = ROcp4_66*s.dpt(2,4);
    OMcp4_115 = OMcp4_16+ROcp4_46*qd(15);
    OMcp4_215 = OMcp4_26+ROcp4_56*qd(15);
    OMcp4_315 = OMcp4_36+ROcp4_66*qd(15);
    ORcp4_115 = OMcp4_26*RLcp4_315-OMcp4_36*RLcp4_215;
    ORcp4_215 = -(OMcp4_16*RLcp4_315-OMcp4_36*RLcp4_115);
    ORcp4_315 = OMcp4_16*RLcp4_215-OMcp4_26*RLcp4_115;
    OMcp4_116 = OMcp4_115+ROcp4_115*qd(16);
    OMcp4_216 = OMcp4_215+ROcp4_215*qd(16);
    OMcp4_316 = OMcp4_315+ROcp4_315*qd(16);
    OPcp4_116 = OPcp4_16+ROcp4_115*qdd(16)+ROcp4_46*qdd(15)+qd(15)*(OMcp4_26*ROcp4_66-OMcp4_36*ROcp4_56)+qd(16)*(OMcp4_215*ROcp4_315-OMcp4_315*...
 ROcp4_215);
    OPcp4_216 = OPcp4_26+ROcp4_215*qdd(16)+ROcp4_56*qdd(15)-qd(15)*(OMcp4_16*ROcp4_66-OMcp4_36*ROcp4_46)-qd(16)*(OMcp4_115*ROcp4_315-OMcp4_315*...
 ROcp4_115);
    OPcp4_316 = OPcp4_36+ROcp4_315*qdd(16)+ROcp4_66*qdd(15)+qd(15)*(OMcp4_16*ROcp4_56-OMcp4_26*ROcp4_46)+qd(16)*(OMcp4_115*ROcp4_215-OMcp4_215*...
 ROcp4_115);
    RLcp4_141 = ROcp4_716*s.dpt(3,24);
    RLcp4_241 = ROcp4_816*s.dpt(3,24);
    RLcp4_341 = ROcp4_916*s.dpt(3,24);
    POcp4_141 = RLcp4_115+RLcp4_141+q(1);
    POcp4_241 = RLcp4_215+RLcp4_241+q(2);
    POcp4_341 = RLcp4_315+RLcp4_341+q(3);
    ORcp4_141 = OMcp4_216*RLcp4_341-OMcp4_316*RLcp4_241;
    ORcp4_241 = -(OMcp4_116*RLcp4_341-OMcp4_316*RLcp4_141);
    ORcp4_341 = OMcp4_116*RLcp4_241-OMcp4_216*RLcp4_141;
    VIcp4_141 = ORcp4_115+ORcp4_141+qd(1);
    VIcp4_241 = ORcp4_215+ORcp4_241+qd(2);
    VIcp4_341 = ORcp4_315+ORcp4_341+qd(3);
    ACcp4_141 = qdd(1)+OMcp4_216*ORcp4_341+OMcp4_26*ORcp4_315-OMcp4_316*ORcp4_241-OMcp4_36*ORcp4_215+OPcp4_216*RLcp4_341+OPcp4_26*RLcp4_315-...
 OPcp4_316*RLcp4_241-OPcp4_36*RLcp4_215;
    ACcp4_241 = qdd(2)-OMcp4_116*ORcp4_341-OMcp4_16*ORcp4_315+OMcp4_316*ORcp4_141+OMcp4_36*ORcp4_115-OPcp4_116*RLcp4_341-OPcp4_16*RLcp4_315+...
 OPcp4_316*RLcp4_141+OPcp4_36*RLcp4_115;
    ACcp4_341 = qdd(3)+OMcp4_116*ORcp4_241+OMcp4_16*ORcp4_215-OMcp4_216*ORcp4_141-OMcp4_26*ORcp4_115+OPcp4_116*RLcp4_241+OPcp4_16*RLcp4_215-...
 OPcp4_216*RLcp4_141-OPcp4_26*RLcp4_115;

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp4_141;
    sens.P(2) = POcp4_241;
    sens.P(3) = POcp4_341;
    sens.R(1,1) = ROcp4_115;
    sens.R(1,2) = ROcp4_215;
    sens.R(1,3) = ROcp4_315;
    sens.R(2,1) = ROcp4_416;
    sens.R(2,2) = ROcp4_516;
    sens.R(2,3) = ROcp4_616;
    sens.R(3,1) = ROcp4_716;
    sens.R(3,2) = ROcp4_816;
    sens.R(3,3) = ROcp4_916;
    sens.V(1) = VIcp4_141;
    sens.V(2) = VIcp4_241;
    sens.V(3) = VIcp4_341;
    sens.OM(1) = OMcp4_116;
    sens.OM(2) = OMcp4_216;
    sens.OM(3) = OMcp4_316;
    sens.A(1) = ACcp4_141;
    sens.A(2) = ACcp4_241;
    sens.A(3) = ACcp4_341;
    sens.OMP(1) = OPcp4_116;
    sens.OMP(2) = OPcp4_216;
    sens.OMP(3) = OPcp4_316;

end


% ====== END Task 1 ====== 

  

