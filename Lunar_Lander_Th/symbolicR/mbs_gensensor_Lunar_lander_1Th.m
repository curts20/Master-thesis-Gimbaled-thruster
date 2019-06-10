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
%	==> Flops complexity : 1717
%
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.040 seconds
%
%-------------------------------------------------------------
%
function [sens] = gensensor(s,tsim,usrfun,isens)

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


% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.A(1) = qdd(1);
 
% 
case 2, 


% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
 
% 
case 3, 


% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
 
% 
case 4, 


% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C4;
    sens.R(2,3) = S4;
    sens.R(3,2) = -S4;
    sens.R(3,3) = C4;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = qd(4);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = qdd(4);
 
% 
case 5, 


% = = Block_1_0_0_5_0_1 = = 
 
% Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    OMcp4_25 = qd(5)*C4;
    OMcp4_35 = qd(5)*S4;
    OPcp4_25 = qdd(5)*C4-qd(4)*qd(5)*S4;
    OPcp4_35 = qdd(5)*S4+qd(4)*qd(5)*C4;

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = C5;
    sens.R(1,2) = ROcp4_25;
    sens.R(1,3) = ROcp4_35;
    sens.R(2,2) = C4;
    sens.R(2,3) = S4;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp4_85;
    sens.R(3,3) = ROcp4_95;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = qd(4);
    sens.OM(2) = OMcp4_25;
    sens.OM(3) = OMcp4_35;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = qdd(4);
    sens.OMP(2) = OPcp4_25;
    sens.OMP(3) = OPcp4_35;
 
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

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = ROcp5_16;
    sens.R(1,2) = ROcp5_26;
    sens.R(1,3) = ROcp5_36;
    sens.R(2,1) = ROcp5_46;
    sens.R(2,2) = ROcp5_56;
    sens.R(2,3) = ROcp5_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp5_85;
    sens.R(3,3) = ROcp5_95;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = OMcp5_16;
    sens.OM(2) = OMcp5_26;
    sens.OM(3) = OMcp5_36;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = OPcp5_16;
    sens.OMP(2) = OPcp5_26;
    sens.OMP(3) = OPcp5_36;
 
% 
case 7, 


% = = Block_1_0_0_7_0_1 = = 
 
% Sensor Kinematics 


    ROcp6_25 = S4*S5;
    ROcp6_35 = -C4*S5;
    ROcp6_85 = -S4*C5;
    ROcp6_95 = C4*C5;
    ROcp6_16 = C5*C6;
    ROcp6_26 = ROcp6_25*C6+C4*S6;
    ROcp6_36 = ROcp6_35*C6+S4*S6;
    ROcp6_46 = -C5*S6;
    ROcp6_56 = -(ROcp6_25*S6-C4*C6);
    ROcp6_66 = -(ROcp6_35*S6-S4*C6);
    OMcp6_25 = qd(5)*C4;
    OMcp6_35 = qd(5)*S4;
    OMcp6_16 = qd(4)+qd(6)*S5;
    OMcp6_26 = OMcp6_25+ROcp6_85*qd(6);
    OMcp6_36 = OMcp6_35+ROcp6_95*qd(6);

% = = Block_1_0_0_7_0_2 = = 
 
% Sensor Kinematics 


    ROcp6_47 = ROcp6_46*C7+S5*S7;
    ROcp6_57 = ROcp6_56*C7+ROcp6_85*S7;
    ROcp6_67 = ROcp6_66*C7+ROcp6_95*S7;
    ROcp6_77 = -(ROcp6_46*S7-S5*C7);
    ROcp6_87 = -(ROcp6_56*S7-ROcp6_85*C7);
    ROcp6_97 = -(ROcp6_66*S7-ROcp6_95*C7);
    OMcp6_17 = OMcp6_16+ROcp6_16*qd(7);
    OMcp6_27 = OMcp6_26+ROcp6_26*qd(7);
    OMcp6_37 = OMcp6_36+ROcp6_36*qd(7);
    OPcp6_17 = qdd(4)+ROcp6_16*qdd(7)+qdd(6)*S5+qd(5)*qd(6)*C5+qd(7)*(OMcp6_26*ROcp6_36-OMcp6_36*ROcp6_26);
    OPcp6_27 = ROcp6_26*qdd(7)+ROcp6_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp6_35*S5-ROcp6_95*qd(4))-qd(7)*(OMcp6_16*ROcp6_36-OMcp6_36*...
 ROcp6_16);
    OPcp6_37 = ROcp6_36*qdd(7)+ROcp6_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp6_25*S5-ROcp6_85*qd(4))+qd(7)*(OMcp6_16*ROcp6_26-OMcp6_26*...
 ROcp6_16);

% = = Block_1_0_0_7_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = ROcp6_16;
    sens.R(1,2) = ROcp6_26;
    sens.R(1,3) = ROcp6_36;
    sens.R(2,1) = ROcp6_47;
    sens.R(2,2) = ROcp6_57;
    sens.R(2,3) = ROcp6_67;
    sens.R(3,1) = ROcp6_77;
    sens.R(3,2) = ROcp6_87;
    sens.R(3,3) = ROcp6_97;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = OMcp6_17;
    sens.OM(2) = OMcp6_27;
    sens.OM(3) = OMcp6_37;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = OPcp6_17;
    sens.OMP(2) = OPcp6_27;
    sens.OMP(3) = OPcp6_37;
 
% 
case 8, 


% = = Block_1_0_0_8_0_1 = = 
 
% Sensor Kinematics 


    ROcp7_25 = S4*S5;
    ROcp7_35 = -C4*S5;
    ROcp7_85 = -S4*C5;
    ROcp7_95 = C4*C5;
    ROcp7_16 = C5*C6;
    ROcp7_26 = ROcp7_25*C6+C4*S6;
    ROcp7_36 = ROcp7_35*C6+S4*S6;
    ROcp7_46 = -C5*S6;
    ROcp7_56 = -(ROcp7_25*S6-C4*C6);
    ROcp7_66 = -(ROcp7_35*S6-S4*C6);
    OMcp7_25 = qd(5)*C4;
    OMcp7_35 = qd(5)*S4;
    OMcp7_16 = qd(4)+qd(6)*S5;
    OMcp7_26 = OMcp7_25+ROcp7_85*qd(6);
    OMcp7_36 = OMcp7_35+ROcp7_95*qd(6);

% = = Block_1_0_0_8_0_2 = = 
 
% Sensor Kinematics 


    ROcp7_47 = ROcp7_46*C7+S5*S7;
    ROcp7_57 = ROcp7_56*C7+ROcp7_85*S7;
    ROcp7_67 = ROcp7_66*C7+ROcp7_95*S7;
    ROcp7_77 = -(ROcp7_46*S7-S5*C7);
    ROcp7_87 = -(ROcp7_56*S7-ROcp7_85*C7);
    ROcp7_97 = -(ROcp7_66*S7-ROcp7_95*C7);
    ROcp7_18 = ROcp7_16*C8-ROcp7_77*S8;
    ROcp7_28 = ROcp7_26*C8-ROcp7_87*S8;
    ROcp7_38 = ROcp7_36*C8-ROcp7_97*S8;
    ROcp7_78 = ROcp7_16*S8+ROcp7_77*C8;
    ROcp7_88 = ROcp7_26*S8+ROcp7_87*C8;
    ROcp7_98 = ROcp7_36*S8+ROcp7_97*C8;
    OMcp7_17 = OMcp7_16+ROcp7_16*qd(7);
    OMcp7_27 = OMcp7_26+ROcp7_26*qd(7);
    OMcp7_37 = OMcp7_36+ROcp7_36*qd(7);
    OMcp7_18 = OMcp7_17+ROcp7_47*qd(8);
    OMcp7_28 = OMcp7_27+ROcp7_57*qd(8);
    OMcp7_38 = OMcp7_37+ROcp7_67*qd(8);
    OPcp7_18 = qdd(4)+ROcp7_16*qdd(7)+ROcp7_47*qdd(8)+qdd(6)*S5+qd(5)*qd(6)*C5+qd(7)*(OMcp7_26*ROcp7_36-OMcp7_36*ROcp7_26)+qd(8)*(OMcp7_27*ROcp7_67...
 -OMcp7_37*ROcp7_57);
    OPcp7_28 = ROcp7_26*qdd(7)+ROcp7_57*qdd(8)+ROcp7_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp7_35*S5-ROcp7_95*qd(4))-qd(7)*(OMcp7_16*ROcp7_36...
 -OMcp7_36*ROcp7_16)-qd(8)*(OMcp7_17*ROcp7_67-OMcp7_37*ROcp7_47);
    OPcp7_38 = ROcp7_36*qdd(7)+ROcp7_67*qdd(8)+ROcp7_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp7_25*S5-ROcp7_85*qd(4))+qd(7)*(OMcp7_16*ROcp7_26...
 -OMcp7_26*ROcp7_16)+qd(8)*(OMcp7_17*ROcp7_57-OMcp7_27*ROcp7_47);

% = = Block_1_0_0_8_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = ROcp7_18;
    sens.R(1,2) = ROcp7_28;
    sens.R(1,3) = ROcp7_38;
    sens.R(2,1) = ROcp7_47;
    sens.R(2,2) = ROcp7_57;
    sens.R(2,3) = ROcp7_67;
    sens.R(3,1) = ROcp7_78;
    sens.R(3,2) = ROcp7_88;
    sens.R(3,3) = ROcp7_98;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = OMcp7_18;
    sens.OM(2) = OMcp7_28;
    sens.OM(3) = OMcp7_38;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = OPcp7_18;
    sens.OMP(2) = OPcp7_28;
    sens.OMP(3) = OPcp7_38;
 
% 
case 9, 


% = = Block_1_0_0_9_0_1 = = 
 
% Sensor Kinematics 


    ROcp8_25 = S4*S5;
    ROcp8_35 = -C4*S5;
    ROcp8_85 = -S4*C5;
    ROcp8_95 = C4*C5;
    ROcp8_16 = C5*C6;
    ROcp8_26 = ROcp8_25*C6+C4*S6;
    ROcp8_36 = ROcp8_35*C6+S4*S6;
    ROcp8_46 = -C5*S6;
    ROcp8_56 = -(ROcp8_25*S6-C4*C6);
    ROcp8_66 = -(ROcp8_35*S6-S4*C6);
    OMcp8_25 = qd(5)*C4;
    OMcp8_35 = qd(5)*S4;
    OMcp8_16 = qd(4)+qd(6)*S5;
    OMcp8_26 = OMcp8_25+ROcp8_85*qd(6);
    OMcp8_36 = OMcp8_35+ROcp8_95*qd(6);
    OPcp8_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp8_26 = ROcp8_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp8_35*S5-ROcp8_95*qd(4));
    OPcp8_36 = ROcp8_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp8_25*S5-ROcp8_85*qd(4));

% = = Block_1_0_0_9_0_3 = = 
 
% Sensor Kinematics 


    ROcp8_19 = ROcp8_16*C9-S5*S9;
    ROcp8_29 = ROcp8_26*C9-ROcp8_85*S9;
    ROcp8_39 = ROcp8_36*C9-ROcp8_95*S9;
    ROcp8_79 = ROcp8_16*S9+S5*C9;
    ROcp8_89 = ROcp8_26*S9+ROcp8_85*C9;
    ROcp8_99 = ROcp8_36*S9+ROcp8_95*C9;
    RLcp8_19 = ROcp8_16*s.dpt(1,1);
    RLcp8_29 = ROcp8_26*s.dpt(1,1);
    RLcp8_39 = ROcp8_36*s.dpt(1,1);
    POcp8_19 = RLcp8_19+q(1);
    POcp8_29 = RLcp8_29+q(2);
    POcp8_39 = RLcp8_39+q(3);
    OMcp8_19 = OMcp8_16+ROcp8_46*qd(9);
    OMcp8_29 = OMcp8_26+ROcp8_56*qd(9);
    OMcp8_39 = OMcp8_36+ROcp8_66*qd(9);
    ORcp8_19 = OMcp8_26*RLcp8_39-OMcp8_36*RLcp8_29;
    ORcp8_29 = -(OMcp8_16*RLcp8_39-OMcp8_36*RLcp8_19);
    ORcp8_39 = OMcp8_16*RLcp8_29-OMcp8_26*RLcp8_19;
    VIcp8_19 = ORcp8_19+qd(1);
    VIcp8_29 = ORcp8_29+qd(2);
    VIcp8_39 = ORcp8_39+qd(3);
    OPcp8_19 = OPcp8_16+ROcp8_46*qdd(9)+qd(9)*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56);
    OPcp8_29 = OPcp8_26+ROcp8_56*qdd(9)-qd(9)*(OMcp8_16*ROcp8_66-OMcp8_36*ROcp8_46);
    OPcp8_39 = OPcp8_36+ROcp8_66*qdd(9)+qd(9)*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46);
    ACcp8_19 = qdd(1)+OMcp8_26*ORcp8_39-OMcp8_36*ORcp8_29+OPcp8_26*RLcp8_39-OPcp8_36*RLcp8_29;
    ACcp8_29 = qdd(2)-OMcp8_16*ORcp8_39+OMcp8_36*ORcp8_19-OPcp8_16*RLcp8_39+OPcp8_36*RLcp8_19;
    ACcp8_39 = qdd(3)+OMcp8_16*ORcp8_29-OMcp8_26*ORcp8_19+OPcp8_16*RLcp8_29-OPcp8_26*RLcp8_19;

% = = Block_1_0_0_9_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp8_19;
    sens.P(2) = POcp8_29;
    sens.P(3) = POcp8_39;
    sens.R(1,1) = ROcp8_19;
    sens.R(1,2) = ROcp8_29;
    sens.R(1,3) = ROcp8_39;
    sens.R(2,1) = ROcp8_46;
    sens.R(2,2) = ROcp8_56;
    sens.R(2,3) = ROcp8_66;
    sens.R(3,1) = ROcp8_79;
    sens.R(3,2) = ROcp8_89;
    sens.R(3,3) = ROcp8_99;
    sens.V(1) = VIcp8_19;
    sens.V(2) = VIcp8_29;
    sens.V(3) = VIcp8_39;
    sens.OM(1) = OMcp8_19;
    sens.OM(2) = OMcp8_29;
    sens.OM(3) = OMcp8_39;
    sens.A(1) = ACcp8_19;
    sens.A(2) = ACcp8_29;
    sens.A(3) = ACcp8_39;
    sens.OMP(1) = OPcp8_19;
    sens.OMP(2) = OPcp8_29;
    sens.OMP(3) = OPcp8_39;
 
% 
case 10, 


% = = Block_1_0_0_10_0_1 = = 
 
% Sensor Kinematics 


    ROcp9_25 = S4*S5;
    ROcp9_35 = -C4*S5;
    ROcp9_85 = -S4*C5;
    ROcp9_95 = C4*C5;
    ROcp9_16 = C5*C6;
    ROcp9_26 = ROcp9_25*C6+C4*S6;
    ROcp9_36 = ROcp9_35*C6+S4*S6;
    ROcp9_46 = -C5*S6;
    ROcp9_56 = -(ROcp9_25*S6-C4*C6);
    ROcp9_66 = -(ROcp9_35*S6-S4*C6);
    OMcp9_25 = qd(5)*C4;
    OMcp9_35 = qd(5)*S4;
    OMcp9_16 = qd(4)+qd(6)*S5;
    OMcp9_26 = OMcp9_25+ROcp9_85*qd(6);
    OMcp9_36 = OMcp9_35+ROcp9_95*qd(6);
    OPcp9_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp9_26 = ROcp9_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp9_35*S5-ROcp9_95*qd(4));
    OPcp9_36 = ROcp9_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp9_25*S5-ROcp9_85*qd(4));

% = = Block_1_0_0_10_0_3 = = 
 
% Sensor Kinematics 


    ROcp9_19 = ROcp9_16*C9-S5*S9;
    ROcp9_29 = ROcp9_26*C9-ROcp9_85*S9;
    ROcp9_39 = ROcp9_36*C9-ROcp9_95*S9;
    ROcp9_79 = ROcp9_16*S9+S5*C9;
    ROcp9_89 = ROcp9_26*S9+ROcp9_85*C9;
    ROcp9_99 = ROcp9_36*S9+ROcp9_95*C9;
    ROcp9_410 = ROcp9_46*C10+ROcp9_79*S10;
    ROcp9_510 = ROcp9_56*C10+ROcp9_89*S10;
    ROcp9_610 = ROcp9_66*C10+ROcp9_99*S10;
    ROcp9_710 = -(ROcp9_46*S10-ROcp9_79*C10);
    ROcp9_810 = -(ROcp9_56*S10-ROcp9_89*C10);
    ROcp9_910 = -(ROcp9_66*S10-ROcp9_99*C10);
    RLcp9_19 = ROcp9_16*s.dpt(1,1);
    RLcp9_29 = ROcp9_26*s.dpt(1,1);
    RLcp9_39 = ROcp9_36*s.dpt(1,1);
    POcp9_19 = RLcp9_19+q(1);
    POcp9_29 = RLcp9_29+q(2);
    POcp9_39 = RLcp9_39+q(3);
    OMcp9_19 = OMcp9_16+ROcp9_46*qd(9);
    OMcp9_29 = OMcp9_26+ROcp9_56*qd(9);
    OMcp9_39 = OMcp9_36+ROcp9_66*qd(9);
    ORcp9_19 = OMcp9_26*RLcp9_39-OMcp9_36*RLcp9_29;
    ORcp9_29 = -(OMcp9_16*RLcp9_39-OMcp9_36*RLcp9_19);
    ORcp9_39 = OMcp9_16*RLcp9_29-OMcp9_26*RLcp9_19;
    VIcp9_19 = ORcp9_19+qd(1);
    VIcp9_29 = ORcp9_29+qd(2);
    VIcp9_39 = ORcp9_39+qd(3);
    ACcp9_19 = qdd(1)+OMcp9_26*ORcp9_39-OMcp9_36*ORcp9_29+OPcp9_26*RLcp9_39-OPcp9_36*RLcp9_29;
    ACcp9_29 = qdd(2)-OMcp9_16*ORcp9_39+OMcp9_36*ORcp9_19-OPcp9_16*RLcp9_39+OPcp9_36*RLcp9_19;
    ACcp9_39 = qdd(3)+OMcp9_16*ORcp9_29-OMcp9_26*ORcp9_19+OPcp9_16*RLcp9_29-OPcp9_26*RLcp9_19;
    OMcp9_110 = OMcp9_19+ROcp9_19*qd(10);
    OMcp9_210 = OMcp9_29+ROcp9_29*qd(10);
    OMcp9_310 = OMcp9_39+ROcp9_39*qd(10);
    OPcp9_110 = OPcp9_16+ROcp9_19*qdd(10)+ROcp9_46*qdd(9)+qd(10)*(OMcp9_29*ROcp9_39-OMcp9_39*ROcp9_29)+qd(9)*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56);
    OPcp9_210 = OPcp9_26+ROcp9_29*qdd(10)+ROcp9_56*qdd(9)-qd(10)*(OMcp9_19*ROcp9_39-OMcp9_39*ROcp9_19)-qd(9)*(OMcp9_16*ROcp9_66-OMcp9_36*ROcp9_46);
    OPcp9_310 = OPcp9_36+ROcp9_39*qdd(10)+ROcp9_66*qdd(9)+qd(10)*(OMcp9_19*ROcp9_29-OMcp9_29*ROcp9_19)+qd(9)*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46);

% = = Block_1_0_0_10_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp9_19;
    sens.P(2) = POcp9_29;
    sens.P(3) = POcp9_39;
    sens.R(1,1) = ROcp9_19;
    sens.R(1,2) = ROcp9_29;
    sens.R(1,3) = ROcp9_39;
    sens.R(2,1) = ROcp9_410;
    sens.R(2,2) = ROcp9_510;
    sens.R(2,3) = ROcp9_610;
    sens.R(3,1) = ROcp9_710;
    sens.R(3,2) = ROcp9_810;
    sens.R(3,3) = ROcp9_910;
    sens.V(1) = VIcp9_19;
    sens.V(2) = VIcp9_29;
    sens.V(3) = VIcp9_39;
    sens.OM(1) = OMcp9_110;
    sens.OM(2) = OMcp9_210;
    sens.OM(3) = OMcp9_310;
    sens.A(1) = ACcp9_19;
    sens.A(2) = ACcp9_29;
    sens.A(3) = ACcp9_39;
    sens.OMP(1) = OPcp9_110;
    sens.OMP(2) = OPcp9_210;
    sens.OMP(3) = OPcp9_310;
 
% 
case 11, 


% = = Block_1_0_0_11_0_1 = = 
 
% Sensor Kinematics 


    ROcp10_25 = S4*S5;
    ROcp10_35 = -C4*S5;
    ROcp10_85 = -S4*C5;
    ROcp10_95 = C4*C5;
    ROcp10_16 = C5*C6;
    ROcp10_26 = ROcp10_25*C6+C4*S6;
    ROcp10_36 = ROcp10_35*C6+S4*S6;
    ROcp10_46 = -C5*S6;
    ROcp10_56 = -(ROcp10_25*S6-C4*C6);
    ROcp10_66 = -(ROcp10_35*S6-S4*C6);
    OMcp10_25 = qd(5)*C4;
    OMcp10_35 = qd(5)*S4;
    OMcp10_16 = qd(4)+qd(6)*S5;
    OMcp10_26 = OMcp10_25+ROcp10_85*qd(6);
    OMcp10_36 = OMcp10_35+ROcp10_95*qd(6);
    OPcp10_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp10_26 = ROcp10_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp10_35*S5-ROcp10_95*qd(4));
    OPcp10_36 = ROcp10_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp10_25*S5-ROcp10_85*qd(4));

% = = Block_1_0_0_11_0_4 = = 
 
% Sensor Kinematics 


    ROcp10_111 = ROcp10_16*C11-S11*S5;
    ROcp10_211 = ROcp10_26*C11-ROcp10_85*S11;
    ROcp10_311 = ROcp10_36*C11-ROcp10_95*S11;
    ROcp10_711 = ROcp10_16*S11+C11*S5;
    ROcp10_811 = ROcp10_26*S11+ROcp10_85*C11;
    ROcp10_911 = ROcp10_36*S11+ROcp10_95*C11;
    RLcp10_111 = ROcp10_46*s.dpt(2,2);
    RLcp10_211 = ROcp10_56*s.dpt(2,2);
    RLcp10_311 = ROcp10_66*s.dpt(2,2);
    POcp10_111 = RLcp10_111+q(1);
    POcp10_211 = RLcp10_211+q(2);
    POcp10_311 = RLcp10_311+q(3);
    OMcp10_111 = OMcp10_16+ROcp10_46*qd(11);
    OMcp10_211 = OMcp10_26+ROcp10_56*qd(11);
    OMcp10_311 = OMcp10_36+ROcp10_66*qd(11);
    ORcp10_111 = OMcp10_26*RLcp10_311-OMcp10_36*RLcp10_211;
    ORcp10_211 = -(OMcp10_16*RLcp10_311-OMcp10_36*RLcp10_111);
    ORcp10_311 = OMcp10_16*RLcp10_211-OMcp10_26*RLcp10_111;
    VIcp10_111 = ORcp10_111+qd(1);
    VIcp10_211 = ORcp10_211+qd(2);
    VIcp10_311 = ORcp10_311+qd(3);
    OPcp10_111 = OPcp10_16+ROcp10_46*qdd(11)+qd(11)*(OMcp10_26*ROcp10_66-OMcp10_36*ROcp10_56);
    OPcp10_211 = OPcp10_26+ROcp10_56*qdd(11)-qd(11)*(OMcp10_16*ROcp10_66-OMcp10_36*ROcp10_46);
    OPcp10_311 = OPcp10_36+ROcp10_66*qdd(11)+qd(11)*(OMcp10_16*ROcp10_56-OMcp10_26*ROcp10_46);
    ACcp10_111 = qdd(1)+OMcp10_26*ORcp10_311-OMcp10_36*ORcp10_211+OPcp10_26*RLcp10_311-OPcp10_36*RLcp10_211;
    ACcp10_211 = qdd(2)-OMcp10_16*ORcp10_311+OMcp10_36*ORcp10_111-OPcp10_16*RLcp10_311+OPcp10_36*RLcp10_111;
    ACcp10_311 = qdd(3)+OMcp10_16*ORcp10_211-OMcp10_26*ORcp10_111+OPcp10_16*RLcp10_211-OPcp10_26*RLcp10_111;

% = = Block_1_0_0_11_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp10_111;
    sens.P(2) = POcp10_211;
    sens.P(3) = POcp10_311;
    sens.R(1,1) = ROcp10_111;
    sens.R(1,2) = ROcp10_211;
    sens.R(1,3) = ROcp10_311;
    sens.R(2,1) = ROcp10_46;
    sens.R(2,2) = ROcp10_56;
    sens.R(2,3) = ROcp10_66;
    sens.R(3,1) = ROcp10_711;
    sens.R(3,2) = ROcp10_811;
    sens.R(3,3) = ROcp10_911;
    sens.V(1) = VIcp10_111;
    sens.V(2) = VIcp10_211;
    sens.V(3) = VIcp10_311;
    sens.OM(1) = OMcp10_111;
    sens.OM(2) = OMcp10_211;
    sens.OM(3) = OMcp10_311;
    sens.A(1) = ACcp10_111;
    sens.A(2) = ACcp10_211;
    sens.A(3) = ACcp10_311;
    sens.OMP(1) = OPcp10_111;
    sens.OMP(2) = OPcp10_211;
    sens.OMP(3) = OPcp10_311;
 
% 
case 12, 


% = = Block_1_0_0_12_0_1 = = 
 
% Sensor Kinematics 


    ROcp11_25 = S4*S5;
    ROcp11_35 = -C4*S5;
    ROcp11_85 = -S4*C5;
    ROcp11_95 = C4*C5;
    ROcp11_16 = C5*C6;
    ROcp11_26 = ROcp11_25*C6+C4*S6;
    ROcp11_36 = ROcp11_35*C6+S4*S6;
    ROcp11_46 = -C5*S6;
    ROcp11_56 = -(ROcp11_25*S6-C4*C6);
    ROcp11_66 = -(ROcp11_35*S6-S4*C6);
    OMcp11_25 = qd(5)*C4;
    OMcp11_35 = qd(5)*S4;
    OMcp11_16 = qd(4)+qd(6)*S5;
    OMcp11_26 = OMcp11_25+ROcp11_85*qd(6);
    OMcp11_36 = OMcp11_35+ROcp11_95*qd(6);
    OPcp11_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp11_26 = ROcp11_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp11_35*S5-ROcp11_95*qd(4));
    OPcp11_36 = ROcp11_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp11_25*S5-ROcp11_85*qd(4));

% = = Block_1_0_0_12_0_4 = = 
 
% Sensor Kinematics 


    ROcp11_111 = ROcp11_16*C11-S11*S5;
    ROcp11_211 = ROcp11_26*C11-ROcp11_85*S11;
    ROcp11_311 = ROcp11_36*C11-ROcp11_95*S11;
    ROcp11_711 = ROcp11_16*S11+C11*S5;
    ROcp11_811 = ROcp11_26*S11+ROcp11_85*C11;
    ROcp11_911 = ROcp11_36*S11+ROcp11_95*C11;
    ROcp11_412 = ROcp11_46*C12+ROcp11_711*S12;
    ROcp11_512 = ROcp11_56*C12+ROcp11_811*S12;
    ROcp11_612 = ROcp11_66*C12+ROcp11_911*S12;
    ROcp11_712 = -(ROcp11_46*S12-ROcp11_711*C12);
    ROcp11_812 = -(ROcp11_56*S12-ROcp11_811*C12);
    ROcp11_912 = -(ROcp11_66*S12-ROcp11_911*C12);
    RLcp11_111 = ROcp11_46*s.dpt(2,2);
    RLcp11_211 = ROcp11_56*s.dpt(2,2);
    RLcp11_311 = ROcp11_66*s.dpt(2,2);
    POcp11_111 = RLcp11_111+q(1);
    POcp11_211 = RLcp11_211+q(2);
    POcp11_311 = RLcp11_311+q(3);
    OMcp11_111 = OMcp11_16+ROcp11_46*qd(11);
    OMcp11_211 = OMcp11_26+ROcp11_56*qd(11);
    OMcp11_311 = OMcp11_36+ROcp11_66*qd(11);
    ORcp11_111 = OMcp11_26*RLcp11_311-OMcp11_36*RLcp11_211;
    ORcp11_211 = -(OMcp11_16*RLcp11_311-OMcp11_36*RLcp11_111);
    ORcp11_311 = OMcp11_16*RLcp11_211-OMcp11_26*RLcp11_111;
    VIcp11_111 = ORcp11_111+qd(1);
    VIcp11_211 = ORcp11_211+qd(2);
    VIcp11_311 = ORcp11_311+qd(3);
    ACcp11_111 = qdd(1)+OMcp11_26*ORcp11_311-OMcp11_36*ORcp11_211+OPcp11_26*RLcp11_311-OPcp11_36*RLcp11_211;
    ACcp11_211 = qdd(2)-OMcp11_16*ORcp11_311+OMcp11_36*ORcp11_111-OPcp11_16*RLcp11_311+OPcp11_36*RLcp11_111;
    ACcp11_311 = qdd(3)+OMcp11_16*ORcp11_211-OMcp11_26*ORcp11_111+OPcp11_16*RLcp11_211-OPcp11_26*RLcp11_111;
    OMcp11_112 = OMcp11_111+ROcp11_111*qd(12);
    OMcp11_212 = OMcp11_211+ROcp11_211*qd(12);
    OMcp11_312 = OMcp11_311+ROcp11_311*qd(12);
    OPcp11_112 = OPcp11_16+ROcp11_111*qdd(12)+ROcp11_46*qdd(11)+qd(11)*(OMcp11_26*ROcp11_66-OMcp11_36*ROcp11_56)+qd(12)*(OMcp11_211*ROcp11_311-...
 OMcp11_311*ROcp11_211);
    OPcp11_212 = OPcp11_26+ROcp11_211*qdd(12)+ROcp11_56*qdd(11)-qd(11)*(OMcp11_16*ROcp11_66-OMcp11_36*ROcp11_46)-qd(12)*(OMcp11_111*ROcp11_311-...
 OMcp11_311*ROcp11_111);
    OPcp11_312 = OPcp11_36+ROcp11_311*qdd(12)+ROcp11_66*qdd(11)+qd(11)*(OMcp11_16*ROcp11_56-OMcp11_26*ROcp11_46)+qd(12)*(OMcp11_111*ROcp11_211-...
 OMcp11_211*ROcp11_111);

% = = Block_1_0_0_12_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp11_111;
    sens.P(2) = POcp11_211;
    sens.P(3) = POcp11_311;
    sens.R(1,1) = ROcp11_111;
    sens.R(1,2) = ROcp11_211;
    sens.R(1,3) = ROcp11_311;
    sens.R(2,1) = ROcp11_412;
    sens.R(2,2) = ROcp11_512;
    sens.R(2,3) = ROcp11_612;
    sens.R(3,1) = ROcp11_712;
    sens.R(3,2) = ROcp11_812;
    sens.R(3,3) = ROcp11_912;
    sens.V(1) = VIcp11_111;
    sens.V(2) = VIcp11_211;
    sens.V(3) = VIcp11_311;
    sens.OM(1) = OMcp11_112;
    sens.OM(2) = OMcp11_212;
    sens.OM(3) = OMcp11_312;
    sens.A(1) = ACcp11_111;
    sens.A(2) = ACcp11_211;
    sens.A(3) = ACcp11_311;
    sens.OMP(1) = OPcp11_112;
    sens.OMP(2) = OPcp11_212;
    sens.OMP(3) = OPcp11_312;
 
% 
case 13, 


% = = Block_1_0_0_13_0_1 = = 
 
% Sensor Kinematics 


    ROcp12_25 = S4*S5;
    ROcp12_35 = -C4*S5;
    ROcp12_85 = -S4*C5;
    ROcp12_95 = C4*C5;
    ROcp12_16 = C5*C6;
    ROcp12_26 = ROcp12_25*C6+C4*S6;
    ROcp12_36 = ROcp12_35*C6+S4*S6;
    ROcp12_46 = -C5*S6;
    ROcp12_56 = -(ROcp12_25*S6-C4*C6);
    ROcp12_66 = -(ROcp12_35*S6-S4*C6);
    OMcp12_25 = qd(5)*C4;
    OMcp12_35 = qd(5)*S4;
    OMcp12_16 = qd(4)+qd(6)*S5;
    OMcp12_26 = OMcp12_25+ROcp12_85*qd(6);
    OMcp12_36 = OMcp12_35+ROcp12_95*qd(6);
    OPcp12_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp12_26 = ROcp12_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp12_35*S5-ROcp12_95*qd(4));
    OPcp12_36 = ROcp12_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp12_25*S5-ROcp12_85*qd(4));

% = = Block_1_0_0_13_0_5 = = 
 
% Sensor Kinematics 


    ROcp12_113 = ROcp12_16*C13-S13*S5;
    ROcp12_213 = ROcp12_26*C13-ROcp12_85*S13;
    ROcp12_313 = ROcp12_36*C13-ROcp12_95*S13;
    ROcp12_713 = ROcp12_16*S13+C13*S5;
    ROcp12_813 = ROcp12_26*S13+ROcp12_85*C13;
    ROcp12_913 = ROcp12_36*S13+ROcp12_95*C13;
    RLcp12_113 = ROcp12_16*s.dpt(1,3);
    RLcp12_213 = ROcp12_26*s.dpt(1,3);
    RLcp12_313 = ROcp12_36*s.dpt(1,3);
    POcp12_113 = RLcp12_113+q(1);
    POcp12_213 = RLcp12_213+q(2);
    POcp12_313 = RLcp12_313+q(3);
    OMcp12_113 = OMcp12_16+ROcp12_46*qd(13);
    OMcp12_213 = OMcp12_26+ROcp12_56*qd(13);
    OMcp12_313 = OMcp12_36+ROcp12_66*qd(13);
    ORcp12_113 = OMcp12_26*RLcp12_313-OMcp12_36*RLcp12_213;
    ORcp12_213 = -(OMcp12_16*RLcp12_313-OMcp12_36*RLcp12_113);
    ORcp12_313 = OMcp12_16*RLcp12_213-OMcp12_26*RLcp12_113;
    VIcp12_113 = ORcp12_113+qd(1);
    VIcp12_213 = ORcp12_213+qd(2);
    VIcp12_313 = ORcp12_313+qd(3);
    OPcp12_113 = OPcp12_16+ROcp12_46*qdd(13)+qd(13)*(OMcp12_26*ROcp12_66-OMcp12_36*ROcp12_56);
    OPcp12_213 = OPcp12_26+ROcp12_56*qdd(13)-qd(13)*(OMcp12_16*ROcp12_66-OMcp12_36*ROcp12_46);
    OPcp12_313 = OPcp12_36+ROcp12_66*qdd(13)+qd(13)*(OMcp12_16*ROcp12_56-OMcp12_26*ROcp12_46);
    ACcp12_113 = qdd(1)+OMcp12_26*ORcp12_313-OMcp12_36*ORcp12_213+OPcp12_26*RLcp12_313-OPcp12_36*RLcp12_213;
    ACcp12_213 = qdd(2)-OMcp12_16*ORcp12_313+OMcp12_36*ORcp12_113-OPcp12_16*RLcp12_313+OPcp12_36*RLcp12_113;
    ACcp12_313 = qdd(3)+OMcp12_16*ORcp12_213-OMcp12_26*ORcp12_113+OPcp12_16*RLcp12_213-OPcp12_26*RLcp12_113;

% = = Block_1_0_0_13_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp12_113;
    sens.P(2) = POcp12_213;
    sens.P(3) = POcp12_313;
    sens.R(1,1) = ROcp12_113;
    sens.R(1,2) = ROcp12_213;
    sens.R(1,3) = ROcp12_313;
    sens.R(2,1) = ROcp12_46;
    sens.R(2,2) = ROcp12_56;
    sens.R(2,3) = ROcp12_66;
    sens.R(3,1) = ROcp12_713;
    sens.R(3,2) = ROcp12_813;
    sens.R(3,3) = ROcp12_913;
    sens.V(1) = VIcp12_113;
    sens.V(2) = VIcp12_213;
    sens.V(3) = VIcp12_313;
    sens.OM(1) = OMcp12_113;
    sens.OM(2) = OMcp12_213;
    sens.OM(3) = OMcp12_313;
    sens.A(1) = ACcp12_113;
    sens.A(2) = ACcp12_213;
    sens.A(3) = ACcp12_313;
    sens.OMP(1) = OPcp12_113;
    sens.OMP(2) = OPcp12_213;
    sens.OMP(3) = OPcp12_313;
 
% 
case 14, 


% = = Block_1_0_0_14_0_1 = = 
 
% Sensor Kinematics 


    ROcp13_25 = S4*S5;
    ROcp13_35 = -C4*S5;
    ROcp13_85 = -S4*C5;
    ROcp13_95 = C4*C5;
    ROcp13_16 = C5*C6;
    ROcp13_26 = ROcp13_25*C6+C4*S6;
    ROcp13_36 = ROcp13_35*C6+S4*S6;
    ROcp13_46 = -C5*S6;
    ROcp13_56 = -(ROcp13_25*S6-C4*C6);
    ROcp13_66 = -(ROcp13_35*S6-S4*C6);
    OMcp13_25 = qd(5)*C4;
    OMcp13_35 = qd(5)*S4;
    OMcp13_16 = qd(4)+qd(6)*S5;
    OMcp13_26 = OMcp13_25+ROcp13_85*qd(6);
    OMcp13_36 = OMcp13_35+ROcp13_95*qd(6);
    OPcp13_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp13_26 = ROcp13_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp13_35*S5-ROcp13_95*qd(4));
    OPcp13_36 = ROcp13_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp13_25*S5-ROcp13_85*qd(4));

% = = Block_1_0_0_14_0_5 = = 
 
% Sensor Kinematics 


    ROcp13_113 = ROcp13_16*C13-S13*S5;
    ROcp13_213 = ROcp13_26*C13-ROcp13_85*S13;
    ROcp13_313 = ROcp13_36*C13-ROcp13_95*S13;
    ROcp13_713 = ROcp13_16*S13+C13*S5;
    ROcp13_813 = ROcp13_26*S13+ROcp13_85*C13;
    ROcp13_913 = ROcp13_36*S13+ROcp13_95*C13;
    ROcp13_414 = ROcp13_46*C14+ROcp13_713*S14;
    ROcp13_514 = ROcp13_56*C14+ROcp13_813*S14;
    ROcp13_614 = ROcp13_66*C14+ROcp13_913*S14;
    ROcp13_714 = -(ROcp13_46*S14-ROcp13_713*C14);
    ROcp13_814 = -(ROcp13_56*S14-ROcp13_813*C14);
    ROcp13_914 = -(ROcp13_66*S14-ROcp13_913*C14);
    RLcp13_113 = ROcp13_16*s.dpt(1,3);
    RLcp13_213 = ROcp13_26*s.dpt(1,3);
    RLcp13_313 = ROcp13_36*s.dpt(1,3);
    POcp13_113 = RLcp13_113+q(1);
    POcp13_213 = RLcp13_213+q(2);
    POcp13_313 = RLcp13_313+q(3);
    OMcp13_113 = OMcp13_16+ROcp13_46*qd(13);
    OMcp13_213 = OMcp13_26+ROcp13_56*qd(13);
    OMcp13_313 = OMcp13_36+ROcp13_66*qd(13);
    ORcp13_113 = OMcp13_26*RLcp13_313-OMcp13_36*RLcp13_213;
    ORcp13_213 = -(OMcp13_16*RLcp13_313-OMcp13_36*RLcp13_113);
    ORcp13_313 = OMcp13_16*RLcp13_213-OMcp13_26*RLcp13_113;
    VIcp13_113 = ORcp13_113+qd(1);
    VIcp13_213 = ORcp13_213+qd(2);
    VIcp13_313 = ORcp13_313+qd(3);
    ACcp13_113 = qdd(1)+OMcp13_26*ORcp13_313-OMcp13_36*ORcp13_213+OPcp13_26*RLcp13_313-OPcp13_36*RLcp13_213;
    ACcp13_213 = qdd(2)-OMcp13_16*ORcp13_313+OMcp13_36*ORcp13_113-OPcp13_16*RLcp13_313+OPcp13_36*RLcp13_113;
    ACcp13_313 = qdd(3)+OMcp13_16*ORcp13_213-OMcp13_26*ORcp13_113+OPcp13_16*RLcp13_213-OPcp13_26*RLcp13_113;
    OMcp13_114 = OMcp13_113+ROcp13_113*qd(14);
    OMcp13_214 = OMcp13_213+ROcp13_213*qd(14);
    OMcp13_314 = OMcp13_313+ROcp13_313*qd(14);
    OPcp13_114 = OPcp13_16+ROcp13_113*qdd(14)+ROcp13_46*qdd(13)+qd(13)*(OMcp13_26*ROcp13_66-OMcp13_36*ROcp13_56)+qd(14)*(OMcp13_213*ROcp13_313-...
 OMcp13_313*ROcp13_213);
    OPcp13_214 = OPcp13_26+ROcp13_213*qdd(14)+ROcp13_56*qdd(13)-qd(13)*(OMcp13_16*ROcp13_66-OMcp13_36*ROcp13_46)-qd(14)*(OMcp13_113*ROcp13_313-...
 OMcp13_313*ROcp13_113);
    OPcp13_314 = OPcp13_36+ROcp13_313*qdd(14)+ROcp13_66*qdd(13)+qd(13)*(OMcp13_16*ROcp13_56-OMcp13_26*ROcp13_46)+qd(14)*(OMcp13_113*ROcp13_213-...
 OMcp13_213*ROcp13_113);

% = = Block_1_0_0_14_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp13_113;
    sens.P(2) = POcp13_213;
    sens.P(3) = POcp13_313;
    sens.R(1,1) = ROcp13_113;
    sens.R(1,2) = ROcp13_213;
    sens.R(1,3) = ROcp13_313;
    sens.R(2,1) = ROcp13_414;
    sens.R(2,2) = ROcp13_514;
    sens.R(2,3) = ROcp13_614;
    sens.R(3,1) = ROcp13_714;
    sens.R(3,2) = ROcp13_814;
    sens.R(3,3) = ROcp13_914;
    sens.V(1) = VIcp13_113;
    sens.V(2) = VIcp13_213;
    sens.V(3) = VIcp13_313;
    sens.OM(1) = OMcp13_114;
    sens.OM(2) = OMcp13_214;
    sens.OM(3) = OMcp13_314;
    sens.A(1) = ACcp13_113;
    sens.A(2) = ACcp13_213;
    sens.A(3) = ACcp13_313;
    sens.OMP(1) = OPcp13_114;
    sens.OMP(2) = OPcp13_214;
    sens.OMP(3) = OPcp13_314;
 
% 
case 15, 


% = = Block_1_0_0_15_0_1 = = 
 
% Sensor Kinematics 


    ROcp14_25 = S4*S5;
    ROcp14_35 = -C4*S5;
    ROcp14_85 = -S4*C5;
    ROcp14_95 = C4*C5;
    ROcp14_16 = C5*C6;
    ROcp14_26 = ROcp14_25*C6+C4*S6;
    ROcp14_36 = ROcp14_35*C6+S4*S6;
    ROcp14_46 = -C5*S6;
    ROcp14_56 = -(ROcp14_25*S6-C4*C6);
    ROcp14_66 = -(ROcp14_35*S6-S4*C6);
    OMcp14_25 = qd(5)*C4;
    OMcp14_35 = qd(5)*S4;
    OMcp14_16 = qd(4)+qd(6)*S5;
    OMcp14_26 = OMcp14_25+ROcp14_85*qd(6);
    OMcp14_36 = OMcp14_35+ROcp14_95*qd(6);
    OPcp14_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp14_26 = ROcp14_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp14_35*S5-ROcp14_95*qd(4));
    OPcp14_36 = ROcp14_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp14_25*S5-ROcp14_85*qd(4));

% = = Block_1_0_0_15_0_6 = = 
 
% Sensor Kinematics 


    ROcp14_115 = ROcp14_16*C15-S15*S5;
    ROcp14_215 = ROcp14_26*C15-ROcp14_85*S15;
    ROcp14_315 = ROcp14_36*C15-ROcp14_95*S15;
    ROcp14_715 = ROcp14_16*S15+C15*S5;
    ROcp14_815 = ROcp14_26*S15+ROcp14_85*C15;
    ROcp14_915 = ROcp14_36*S15+ROcp14_95*C15;
    RLcp14_115 = ROcp14_46*s.dpt(2,4);
    RLcp14_215 = ROcp14_56*s.dpt(2,4);
    RLcp14_315 = ROcp14_66*s.dpt(2,4);
    POcp14_115 = RLcp14_115+q(1);
    POcp14_215 = RLcp14_215+q(2);
    POcp14_315 = RLcp14_315+q(3);
    OMcp14_115 = OMcp14_16+ROcp14_46*qd(15);
    OMcp14_215 = OMcp14_26+ROcp14_56*qd(15);
    OMcp14_315 = OMcp14_36+ROcp14_66*qd(15);
    ORcp14_115 = OMcp14_26*RLcp14_315-OMcp14_36*RLcp14_215;
    ORcp14_215 = -(OMcp14_16*RLcp14_315-OMcp14_36*RLcp14_115);
    ORcp14_315 = OMcp14_16*RLcp14_215-OMcp14_26*RLcp14_115;
    VIcp14_115 = ORcp14_115+qd(1);
    VIcp14_215 = ORcp14_215+qd(2);
    VIcp14_315 = ORcp14_315+qd(3);
    OPcp14_115 = OPcp14_16+ROcp14_46*qdd(15)+qd(15)*(OMcp14_26*ROcp14_66-OMcp14_36*ROcp14_56);
    OPcp14_215 = OPcp14_26+ROcp14_56*qdd(15)-qd(15)*(OMcp14_16*ROcp14_66-OMcp14_36*ROcp14_46);
    OPcp14_315 = OPcp14_36+ROcp14_66*qdd(15)+qd(15)*(OMcp14_16*ROcp14_56-OMcp14_26*ROcp14_46);
    ACcp14_115 = qdd(1)+OMcp14_26*ORcp14_315-OMcp14_36*ORcp14_215+OPcp14_26*RLcp14_315-OPcp14_36*RLcp14_215;
    ACcp14_215 = qdd(2)-OMcp14_16*ORcp14_315+OMcp14_36*ORcp14_115-OPcp14_16*RLcp14_315+OPcp14_36*RLcp14_115;
    ACcp14_315 = qdd(3)+OMcp14_16*ORcp14_215-OMcp14_26*ORcp14_115+OPcp14_16*RLcp14_215-OPcp14_26*RLcp14_115;

% = = Block_1_0_0_15_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp14_115;
    sens.P(2) = POcp14_215;
    sens.P(3) = POcp14_315;
    sens.R(1,1) = ROcp14_115;
    sens.R(1,2) = ROcp14_215;
    sens.R(1,3) = ROcp14_315;
    sens.R(2,1) = ROcp14_46;
    sens.R(2,2) = ROcp14_56;
    sens.R(2,3) = ROcp14_66;
    sens.R(3,1) = ROcp14_715;
    sens.R(3,2) = ROcp14_815;
    sens.R(3,3) = ROcp14_915;
    sens.V(1) = VIcp14_115;
    sens.V(2) = VIcp14_215;
    sens.V(3) = VIcp14_315;
    sens.OM(1) = OMcp14_115;
    sens.OM(2) = OMcp14_215;
    sens.OM(3) = OMcp14_315;
    sens.A(1) = ACcp14_115;
    sens.A(2) = ACcp14_215;
    sens.A(3) = ACcp14_315;
    sens.OMP(1) = OPcp14_115;
    sens.OMP(2) = OPcp14_215;
    sens.OMP(3) = OPcp14_315;
 
% 
case 16, 


% = = Block_1_0_0_16_0_1 = = 
 
% Sensor Kinematics 


    ROcp15_25 = S4*S5;
    ROcp15_35 = -C4*S5;
    ROcp15_85 = -S4*C5;
    ROcp15_95 = C4*C5;
    ROcp15_16 = C5*C6;
    ROcp15_26 = ROcp15_25*C6+C4*S6;
    ROcp15_36 = ROcp15_35*C6+S4*S6;
    ROcp15_46 = -C5*S6;
    ROcp15_56 = -(ROcp15_25*S6-C4*C6);
    ROcp15_66 = -(ROcp15_35*S6-S4*C6);
    OMcp15_25 = qd(5)*C4;
    OMcp15_35 = qd(5)*S4;
    OMcp15_16 = qd(4)+qd(6)*S5;
    OMcp15_26 = OMcp15_25+ROcp15_85*qd(6);
    OMcp15_36 = OMcp15_35+ROcp15_95*qd(6);
    OPcp15_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp15_26 = ROcp15_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp15_35*S5-ROcp15_95*qd(4));
    OPcp15_36 = ROcp15_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp15_25*S5-ROcp15_85*qd(4));

% = = Block_1_0_0_16_0_6 = = 
 
% Sensor Kinematics 


    ROcp15_115 = ROcp15_16*C15-S15*S5;
    ROcp15_215 = ROcp15_26*C15-ROcp15_85*S15;
    ROcp15_315 = ROcp15_36*C15-ROcp15_95*S15;
    ROcp15_715 = ROcp15_16*S15+C15*S5;
    ROcp15_815 = ROcp15_26*S15+ROcp15_85*C15;
    ROcp15_915 = ROcp15_36*S15+ROcp15_95*C15;
    ROcp15_416 = ROcp15_46*C16+ROcp15_715*S16;
    ROcp15_516 = ROcp15_56*C16+ROcp15_815*S16;
    ROcp15_616 = ROcp15_66*C16+ROcp15_915*S16;
    ROcp15_716 = -(ROcp15_46*S16-ROcp15_715*C16);
    ROcp15_816 = -(ROcp15_56*S16-ROcp15_815*C16);
    ROcp15_916 = -(ROcp15_66*S16-ROcp15_915*C16);
    RLcp15_115 = ROcp15_46*s.dpt(2,4);
    RLcp15_215 = ROcp15_56*s.dpt(2,4);
    RLcp15_315 = ROcp15_66*s.dpt(2,4);
    POcp15_115 = RLcp15_115+q(1);
    POcp15_215 = RLcp15_215+q(2);
    POcp15_315 = RLcp15_315+q(3);
    OMcp15_115 = OMcp15_16+ROcp15_46*qd(15);
    OMcp15_215 = OMcp15_26+ROcp15_56*qd(15);
    OMcp15_315 = OMcp15_36+ROcp15_66*qd(15);
    ORcp15_115 = OMcp15_26*RLcp15_315-OMcp15_36*RLcp15_215;
    ORcp15_215 = -(OMcp15_16*RLcp15_315-OMcp15_36*RLcp15_115);
    ORcp15_315 = OMcp15_16*RLcp15_215-OMcp15_26*RLcp15_115;
    VIcp15_115 = ORcp15_115+qd(1);
    VIcp15_215 = ORcp15_215+qd(2);
    VIcp15_315 = ORcp15_315+qd(3);
    ACcp15_115 = qdd(1)+OMcp15_26*ORcp15_315-OMcp15_36*ORcp15_215+OPcp15_26*RLcp15_315-OPcp15_36*RLcp15_215;
    ACcp15_215 = qdd(2)-OMcp15_16*ORcp15_315+OMcp15_36*ORcp15_115-OPcp15_16*RLcp15_315+OPcp15_36*RLcp15_115;
    ACcp15_315 = qdd(3)+OMcp15_16*ORcp15_215-OMcp15_26*ORcp15_115+OPcp15_16*RLcp15_215-OPcp15_26*RLcp15_115;
    OMcp15_116 = OMcp15_115+ROcp15_115*qd(16);
    OMcp15_216 = OMcp15_215+ROcp15_215*qd(16);
    OMcp15_316 = OMcp15_315+ROcp15_315*qd(16);
    OPcp15_116 = OPcp15_16+ROcp15_115*qdd(16)+ROcp15_46*qdd(15)+qd(15)*(OMcp15_26*ROcp15_66-OMcp15_36*ROcp15_56)+qd(16)*(OMcp15_215*ROcp15_315-...
 OMcp15_315*ROcp15_215);
    OPcp15_216 = OPcp15_26+ROcp15_215*qdd(16)+ROcp15_56*qdd(15)-qd(15)*(OMcp15_16*ROcp15_66-OMcp15_36*ROcp15_46)-qd(16)*(OMcp15_115*ROcp15_315-...
 OMcp15_315*ROcp15_115);
    OPcp15_316 = OPcp15_36+ROcp15_315*qdd(16)+ROcp15_66*qdd(15)+qd(15)*(OMcp15_16*ROcp15_56-OMcp15_26*ROcp15_46)+qd(16)*(OMcp15_115*ROcp15_215-...
 OMcp15_215*ROcp15_115);

% = = Block_1_0_0_16_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp15_115;
    sens.P(2) = POcp15_215;
    sens.P(3) = POcp15_315;
    sens.R(1,1) = ROcp15_115;
    sens.R(1,2) = ROcp15_215;
    sens.R(1,3) = ROcp15_315;
    sens.R(2,1) = ROcp15_416;
    sens.R(2,2) = ROcp15_516;
    sens.R(2,3) = ROcp15_616;
    sens.R(3,1) = ROcp15_716;
    sens.R(3,2) = ROcp15_816;
    sens.R(3,3) = ROcp15_916;
    sens.V(1) = VIcp15_115;
    sens.V(2) = VIcp15_215;
    sens.V(3) = VIcp15_315;
    sens.OM(1) = OMcp15_116;
    sens.OM(2) = OMcp15_216;
    sens.OM(3) = OMcp15_316;
    sens.A(1) = ACcp15_115;
    sens.A(2) = ACcp15_215;
    sens.A(3) = ACcp15_315;
    sens.OMP(1) = OPcp15_116;
    sens.OMP(2) = OPcp15_216;
    sens.OMP(3) = OPcp15_316;

end


% ====== END Task 1 ====== 

  

