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
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 2947
%
%	==> Generation Time :  0.040 seconds
%	==> Post-Processing :  0.060 seconds
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
 
% 
case 17, 


% = = Block_1_0_0_17_0_1 = = 
 
% Sensor Kinematics 


    ROcp16_25 = S4*S5;
    ROcp16_35 = -C4*S5;
    ROcp16_85 = -S4*C5;
    ROcp16_95 = C4*C5;
    ROcp16_16 = C5*C6;
    ROcp16_26 = ROcp16_25*C6+C4*S6;
    ROcp16_36 = ROcp16_35*C6+S4*S6;
    ROcp16_46 = -C5*S6;
    ROcp16_56 = -(ROcp16_25*S6-C4*C6);
    ROcp16_66 = -(ROcp16_35*S6-S4*C6);
    OMcp16_25 = qd(5)*C4;
    OMcp16_35 = qd(5)*S4;
    OMcp16_16 = qd(4)+qd(6)*S5;
    OMcp16_26 = OMcp16_25+ROcp16_85*qd(6);
    OMcp16_36 = OMcp16_35+ROcp16_95*qd(6);
    OPcp16_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp16_26 = ROcp16_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp16_35*S5-ROcp16_95*qd(4));
    OPcp16_36 = ROcp16_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp16_25*S5-ROcp16_85*qd(4));

% = = Block_1_0_0_17_0_7 = = 
 
% Sensor Kinematics 


    ROcp16_417 = ROcp16_46*C17+S17*S5;
    ROcp16_517 = ROcp16_56*C17+ROcp16_85*S17;
    ROcp16_617 = ROcp16_66*C17+ROcp16_95*S17;
    ROcp16_717 = -(ROcp16_46*S17-C17*S5);
    ROcp16_817 = -(ROcp16_56*S17-ROcp16_85*C17);
    ROcp16_917 = -(ROcp16_66*S17-ROcp16_95*C17);
    RLcp16_117 = ROcp16_16*s.dpt(1,5);
    RLcp16_217 = ROcp16_26*s.dpt(1,5);
    RLcp16_317 = ROcp16_36*s.dpt(1,5);
    POcp16_117 = RLcp16_117+q(1);
    POcp16_217 = RLcp16_217+q(2);
    POcp16_317 = RLcp16_317+q(3);
    OMcp16_117 = OMcp16_16+ROcp16_16*qd(17);
    OMcp16_217 = OMcp16_26+ROcp16_26*qd(17);
    OMcp16_317 = OMcp16_36+ROcp16_36*qd(17);
    ORcp16_117 = OMcp16_26*RLcp16_317-OMcp16_36*RLcp16_217;
    ORcp16_217 = -(OMcp16_16*RLcp16_317-OMcp16_36*RLcp16_117);
    ORcp16_317 = OMcp16_16*RLcp16_217-OMcp16_26*RLcp16_117;
    VIcp16_117 = ORcp16_117+qd(1);
    VIcp16_217 = ORcp16_217+qd(2);
    VIcp16_317 = ORcp16_317+qd(3);
    OPcp16_117 = OPcp16_16+ROcp16_16*qdd(17)+qd(17)*(OMcp16_26*ROcp16_36-OMcp16_36*ROcp16_26);
    OPcp16_217 = OPcp16_26+ROcp16_26*qdd(17)-qd(17)*(OMcp16_16*ROcp16_36-OMcp16_36*ROcp16_16);
    OPcp16_317 = OPcp16_36+ROcp16_36*qdd(17)+qd(17)*(OMcp16_16*ROcp16_26-OMcp16_26*ROcp16_16);
    ACcp16_117 = qdd(1)+OMcp16_26*ORcp16_317-OMcp16_36*ORcp16_217+OPcp16_26*RLcp16_317-OPcp16_36*RLcp16_217;
    ACcp16_217 = qdd(2)-OMcp16_16*ORcp16_317+OMcp16_36*ORcp16_117-OPcp16_16*RLcp16_317+OPcp16_36*RLcp16_117;
    ACcp16_317 = qdd(3)+OMcp16_16*ORcp16_217-OMcp16_26*ORcp16_117+OPcp16_16*RLcp16_217-OPcp16_26*RLcp16_117;

% = = Block_1_0_0_17_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp16_117;
    sens.P(2) = POcp16_217;
    sens.P(3) = POcp16_317;
    sens.R(1,1) = ROcp16_16;
    sens.R(1,2) = ROcp16_26;
    sens.R(1,3) = ROcp16_36;
    sens.R(2,1) = ROcp16_417;
    sens.R(2,2) = ROcp16_517;
    sens.R(2,3) = ROcp16_617;
    sens.R(3,1) = ROcp16_717;
    sens.R(3,2) = ROcp16_817;
    sens.R(3,3) = ROcp16_917;
    sens.V(1) = VIcp16_117;
    sens.V(2) = VIcp16_217;
    sens.V(3) = VIcp16_317;
    sens.OM(1) = OMcp16_117;
    sens.OM(2) = OMcp16_217;
    sens.OM(3) = OMcp16_317;
    sens.A(1) = ACcp16_117;
    sens.A(2) = ACcp16_217;
    sens.A(3) = ACcp16_317;
    sens.OMP(1) = OPcp16_117;
    sens.OMP(2) = OPcp16_217;
    sens.OMP(3) = OPcp16_317;
 
% 
case 18, 


% = = Block_1_0_0_18_0_1 = = 
 
% Sensor Kinematics 


    ROcp17_25 = S4*S5;
    ROcp17_35 = -C4*S5;
    ROcp17_85 = -S4*C5;
    ROcp17_95 = C4*C5;
    ROcp17_16 = C5*C6;
    ROcp17_26 = ROcp17_25*C6+C4*S6;
    ROcp17_36 = ROcp17_35*C6+S4*S6;
    ROcp17_46 = -C5*S6;
    ROcp17_56 = -(ROcp17_25*S6-C4*C6);
    ROcp17_66 = -(ROcp17_35*S6-S4*C6);
    OMcp17_25 = qd(5)*C4;
    OMcp17_35 = qd(5)*S4;
    OMcp17_16 = qd(4)+qd(6)*S5;
    OMcp17_26 = OMcp17_25+ROcp17_85*qd(6);
    OMcp17_36 = OMcp17_35+ROcp17_95*qd(6);
    OPcp17_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp17_26 = ROcp17_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp17_35*S5-ROcp17_95*qd(4));
    OPcp17_36 = ROcp17_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp17_25*S5-ROcp17_85*qd(4));

% = = Block_1_0_0_18_0_7 = = 
 
% Sensor Kinematics 


    ROcp17_417 = ROcp17_46*C17+S17*S5;
    ROcp17_517 = ROcp17_56*C17+ROcp17_85*S17;
    ROcp17_617 = ROcp17_66*C17+ROcp17_95*S17;
    ROcp17_717 = -(ROcp17_46*S17-C17*S5);
    ROcp17_817 = -(ROcp17_56*S17-ROcp17_85*C17);
    ROcp17_917 = -(ROcp17_66*S17-ROcp17_95*C17);
    ROcp17_118 = ROcp17_16*C18-ROcp17_717*S18;
    ROcp17_218 = ROcp17_26*C18-ROcp17_817*S18;
    ROcp17_318 = ROcp17_36*C18-ROcp17_917*S18;
    ROcp17_718 = ROcp17_16*S18+ROcp17_717*C18;
    ROcp17_818 = ROcp17_26*S18+ROcp17_817*C18;
    ROcp17_918 = ROcp17_36*S18+ROcp17_917*C18;
    RLcp17_117 = ROcp17_16*s.dpt(1,5);
    RLcp17_217 = ROcp17_26*s.dpt(1,5);
    RLcp17_317 = ROcp17_36*s.dpt(1,5);
    POcp17_117 = RLcp17_117+q(1);
    POcp17_217 = RLcp17_217+q(2);
    POcp17_317 = RLcp17_317+q(3);
    OMcp17_117 = OMcp17_16+ROcp17_16*qd(17);
    OMcp17_217 = OMcp17_26+ROcp17_26*qd(17);
    OMcp17_317 = OMcp17_36+ROcp17_36*qd(17);
    ORcp17_117 = OMcp17_26*RLcp17_317-OMcp17_36*RLcp17_217;
    ORcp17_217 = -(OMcp17_16*RLcp17_317-OMcp17_36*RLcp17_117);
    ORcp17_317 = OMcp17_16*RLcp17_217-OMcp17_26*RLcp17_117;
    VIcp17_117 = ORcp17_117+qd(1);
    VIcp17_217 = ORcp17_217+qd(2);
    VIcp17_317 = ORcp17_317+qd(3);
    ACcp17_117 = qdd(1)+OMcp17_26*ORcp17_317-OMcp17_36*ORcp17_217+OPcp17_26*RLcp17_317-OPcp17_36*RLcp17_217;
    ACcp17_217 = qdd(2)-OMcp17_16*ORcp17_317+OMcp17_36*ORcp17_117-OPcp17_16*RLcp17_317+OPcp17_36*RLcp17_117;
    ACcp17_317 = qdd(3)+OMcp17_16*ORcp17_217-OMcp17_26*ORcp17_117+OPcp17_16*RLcp17_217-OPcp17_26*RLcp17_117;
    OMcp17_118 = OMcp17_117+ROcp17_417*qd(18);
    OMcp17_218 = OMcp17_217+ROcp17_517*qd(18);
    OMcp17_318 = OMcp17_317+ROcp17_617*qd(18);
    OPcp17_118 = OPcp17_16+ROcp17_16*qdd(17)+ROcp17_417*qdd(18)+qd(17)*(OMcp17_26*ROcp17_36-OMcp17_36*ROcp17_26)+qd(18)*(OMcp17_217*ROcp17_617-...
 OMcp17_317*ROcp17_517);
    OPcp17_218 = OPcp17_26+ROcp17_26*qdd(17)+ROcp17_517*qdd(18)-qd(17)*(OMcp17_16*ROcp17_36-OMcp17_36*ROcp17_16)-qd(18)*(OMcp17_117*ROcp17_617-...
 OMcp17_317*ROcp17_417);
    OPcp17_318 = OPcp17_36+ROcp17_36*qdd(17)+ROcp17_617*qdd(18)+qd(17)*(OMcp17_16*ROcp17_26-OMcp17_26*ROcp17_16)+qd(18)*(OMcp17_117*ROcp17_517-...
 OMcp17_217*ROcp17_417);

% = = Block_1_0_0_18_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp17_117;
    sens.P(2) = POcp17_217;
    sens.P(3) = POcp17_317;
    sens.R(1,1) = ROcp17_118;
    sens.R(1,2) = ROcp17_218;
    sens.R(1,3) = ROcp17_318;
    sens.R(2,1) = ROcp17_417;
    sens.R(2,2) = ROcp17_517;
    sens.R(2,3) = ROcp17_617;
    sens.R(3,1) = ROcp17_718;
    sens.R(3,2) = ROcp17_818;
    sens.R(3,3) = ROcp17_918;
    sens.V(1) = VIcp17_117;
    sens.V(2) = VIcp17_217;
    sens.V(3) = VIcp17_317;
    sens.OM(1) = OMcp17_118;
    sens.OM(2) = OMcp17_218;
    sens.OM(3) = OMcp17_318;
    sens.A(1) = ACcp17_117;
    sens.A(2) = ACcp17_217;
    sens.A(3) = ACcp17_317;
    sens.OMP(1) = OPcp17_118;
    sens.OMP(2) = OPcp17_218;
    sens.OMP(3) = OPcp17_318;
 
% 
case 19, 


% = = Block_1_0_0_19_0_1 = = 
 
% Sensor Kinematics 


    ROcp18_25 = S4*S5;
    ROcp18_35 = -C4*S5;
    ROcp18_85 = -S4*C5;
    ROcp18_95 = C4*C5;
    ROcp18_16 = C5*C6;
    ROcp18_26 = ROcp18_25*C6+C4*S6;
    ROcp18_36 = ROcp18_35*C6+S4*S6;
    ROcp18_46 = -C5*S6;
    ROcp18_56 = -(ROcp18_25*S6-C4*C6);
    ROcp18_66 = -(ROcp18_35*S6-S4*C6);
    OMcp18_25 = qd(5)*C4;
    OMcp18_35 = qd(5)*S4;
    OMcp18_16 = qd(4)+qd(6)*S5;
    OMcp18_26 = OMcp18_25+ROcp18_85*qd(6);
    OMcp18_36 = OMcp18_35+ROcp18_95*qd(6);
    OPcp18_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp18_26 = ROcp18_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp18_35*S5-ROcp18_95*qd(4));
    OPcp18_36 = ROcp18_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp18_25*S5-ROcp18_85*qd(4));

% = = Block_1_0_0_19_0_7 = = 
 
% Sensor Kinematics 


    ROcp18_417 = ROcp18_46*C17+S17*S5;
    ROcp18_517 = ROcp18_56*C17+ROcp18_85*S17;
    ROcp18_617 = ROcp18_66*C17+ROcp18_95*S17;
    ROcp18_717 = -(ROcp18_46*S17-C17*S5);
    ROcp18_817 = -(ROcp18_56*S17-ROcp18_85*C17);
    ROcp18_917 = -(ROcp18_66*S17-ROcp18_95*C17);
    ROcp18_118 = ROcp18_16*C18-ROcp18_717*S18;
    ROcp18_218 = ROcp18_26*C18-ROcp18_817*S18;
    ROcp18_318 = ROcp18_36*C18-ROcp18_917*S18;
    ROcp18_718 = ROcp18_16*S18+ROcp18_717*C18;
    ROcp18_818 = ROcp18_26*S18+ROcp18_817*C18;
    ROcp18_918 = ROcp18_36*S18+ROcp18_917*C18;
    RLcp18_117 = ROcp18_16*s.dpt(1,5);
    RLcp18_217 = ROcp18_26*s.dpt(1,5);
    RLcp18_317 = ROcp18_36*s.dpt(1,5);
    OMcp18_117 = OMcp18_16+ROcp18_16*qd(17);
    OMcp18_217 = OMcp18_26+ROcp18_26*qd(17);
    OMcp18_317 = OMcp18_36+ROcp18_36*qd(17);
    ORcp18_117 = OMcp18_26*RLcp18_317-OMcp18_36*RLcp18_217;
    ORcp18_217 = -(OMcp18_16*RLcp18_317-OMcp18_36*RLcp18_117);
    ORcp18_317 = OMcp18_16*RLcp18_217-OMcp18_26*RLcp18_117;
    OMcp18_118 = OMcp18_117+ROcp18_417*qd(18);
    OMcp18_218 = OMcp18_217+ROcp18_517*qd(18);
    OMcp18_318 = OMcp18_317+ROcp18_617*qd(18);
    OPcp18_118 = OPcp18_16+ROcp18_16*qdd(17)+ROcp18_417*qdd(18)+qd(17)*(OMcp18_26*ROcp18_36-OMcp18_36*ROcp18_26)+qd(18)*(OMcp18_217*ROcp18_617-...
 OMcp18_317*ROcp18_517);
    OPcp18_218 = OPcp18_26+ROcp18_26*qdd(17)+ROcp18_517*qdd(18)-qd(17)*(OMcp18_16*ROcp18_36-OMcp18_36*ROcp18_16)-qd(18)*(OMcp18_117*ROcp18_617-...
 OMcp18_317*ROcp18_417);
    OPcp18_318 = OPcp18_36+ROcp18_36*qdd(17)+ROcp18_617*qdd(18)+qd(17)*(OMcp18_16*ROcp18_26-OMcp18_26*ROcp18_16)+qd(18)*(OMcp18_117*ROcp18_517-...
 OMcp18_217*ROcp18_417);
    RLcp18_119 = Dz193*ROcp18_718;
    RLcp18_219 = Dz193*ROcp18_818;
    RLcp18_319 = Dz193*ROcp18_918;
    POcp18_119 = RLcp18_117+RLcp18_119+q(1);
    POcp18_219 = RLcp18_217+RLcp18_219+q(2);
    POcp18_319 = RLcp18_317+RLcp18_319+q(3);
    ORcp18_119 = OMcp18_218*RLcp18_319-OMcp18_318*RLcp18_219;
    ORcp18_219 = -(OMcp18_118*RLcp18_319-OMcp18_318*RLcp18_119);
    ORcp18_319 = OMcp18_118*RLcp18_219-OMcp18_218*RLcp18_119;
    VIcp18_119 = ORcp18_117+ORcp18_119+qd(1)+ROcp18_718*qd(19);
    VIcp18_219 = ORcp18_217+ORcp18_219+qd(2)+ROcp18_818*qd(19);
    VIcp18_319 = ORcp18_317+ORcp18_319+qd(3)+ROcp18_918*qd(19);
    ACcp18_119 = qdd(1)+OMcp18_218*ORcp18_319+OMcp18_26*ORcp18_317-OMcp18_318*ORcp18_219-OMcp18_36*ORcp18_217+OPcp18_218*RLcp18_319+OPcp18_26*...
 RLcp18_317-OPcp18_318*RLcp18_219-OPcp18_36*RLcp18_217+ROcp18_718*qdd(19)+(2.0)*qd(19)*(OMcp18_218*ROcp18_918-OMcp18_318*ROcp18_818);
    ACcp18_219 = qdd(2)-OMcp18_118*ORcp18_319-OMcp18_16*ORcp18_317+OMcp18_318*ORcp18_119+OMcp18_36*ORcp18_117-OPcp18_118*RLcp18_319-OPcp18_16*...
 RLcp18_317+OPcp18_318*RLcp18_119+OPcp18_36*RLcp18_117+ROcp18_818*qdd(19)-(2.0)*qd(19)*(OMcp18_118*ROcp18_918-OMcp18_318*ROcp18_718);
    ACcp18_319 = qdd(3)+OMcp18_118*ORcp18_219+OMcp18_16*ORcp18_217-OMcp18_218*ORcp18_119-OMcp18_26*ORcp18_117+OPcp18_118*RLcp18_219+OPcp18_16*...
 RLcp18_217-OPcp18_218*RLcp18_119-OPcp18_26*RLcp18_117+ROcp18_918*qdd(19)+(2.0)*qd(19)*(OMcp18_118*ROcp18_818-OMcp18_218*ROcp18_718);

% = = Block_1_0_0_19_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp18_119;
    sens.P(2) = POcp18_219;
    sens.P(3) = POcp18_319;
    sens.R(1,1) = ROcp18_118;
    sens.R(1,2) = ROcp18_218;
    sens.R(1,3) = ROcp18_318;
    sens.R(2,1) = ROcp18_417;
    sens.R(2,2) = ROcp18_517;
    sens.R(2,3) = ROcp18_617;
    sens.R(3,1) = ROcp18_718;
    sens.R(3,2) = ROcp18_818;
    sens.R(3,3) = ROcp18_918;
    sens.V(1) = VIcp18_119;
    sens.V(2) = VIcp18_219;
    sens.V(3) = VIcp18_319;
    sens.OM(1) = OMcp18_118;
    sens.OM(2) = OMcp18_218;
    sens.OM(3) = OMcp18_318;
    sens.A(1) = ACcp18_119;
    sens.A(2) = ACcp18_219;
    sens.A(3) = ACcp18_319;
    sens.OMP(1) = OPcp18_118;
    sens.OMP(2) = OPcp18_218;
    sens.OMP(3) = OPcp18_318;
 
% 
case 20, 


% = = Block_1_0_0_20_0_1 = = 
 
% Sensor Kinematics 


    ROcp19_25 = S4*S5;
    ROcp19_35 = -C4*S5;
    ROcp19_85 = -S4*C5;
    ROcp19_95 = C4*C5;
    ROcp19_16 = C5*C6;
    ROcp19_26 = ROcp19_25*C6+C4*S6;
    ROcp19_36 = ROcp19_35*C6+S4*S6;
    ROcp19_46 = -C5*S6;
    ROcp19_56 = -(ROcp19_25*S6-C4*C6);
    ROcp19_66 = -(ROcp19_35*S6-S4*C6);
    OMcp19_25 = qd(5)*C4;
    OMcp19_35 = qd(5)*S4;
    OMcp19_16 = qd(4)+qd(6)*S5;
    OMcp19_26 = OMcp19_25+ROcp19_85*qd(6);
    OMcp19_36 = OMcp19_35+ROcp19_95*qd(6);
    OPcp19_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp19_26 = ROcp19_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp19_35*S5-ROcp19_95*qd(4));
    OPcp19_36 = ROcp19_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp19_25*S5-ROcp19_85*qd(4));

% = = Block_1_0_0_20_0_8 = = 
 
% Sensor Kinematics 


    ROcp19_420 = ROcp19_46*C20+S20*S5;
    ROcp19_520 = ROcp19_56*C20+ROcp19_85*S20;
    ROcp19_620 = ROcp19_66*C20+ROcp19_95*S20;
    ROcp19_720 = -(ROcp19_46*S20-C20*S5);
    ROcp19_820 = -(ROcp19_56*S20-ROcp19_85*C20);
    ROcp19_920 = -(ROcp19_66*S20-ROcp19_95*C20);
    RLcp19_120 = ROcp19_46*s.dpt(2,6);
    RLcp19_220 = ROcp19_56*s.dpt(2,6);
    RLcp19_320 = ROcp19_66*s.dpt(2,6);
    POcp19_120 = RLcp19_120+q(1);
    POcp19_220 = RLcp19_220+q(2);
    POcp19_320 = RLcp19_320+q(3);
    OMcp19_120 = OMcp19_16+ROcp19_16*qd(20);
    OMcp19_220 = OMcp19_26+ROcp19_26*qd(20);
    OMcp19_320 = OMcp19_36+ROcp19_36*qd(20);
    ORcp19_120 = OMcp19_26*RLcp19_320-OMcp19_36*RLcp19_220;
    ORcp19_220 = -(OMcp19_16*RLcp19_320-OMcp19_36*RLcp19_120);
    ORcp19_320 = OMcp19_16*RLcp19_220-OMcp19_26*RLcp19_120;
    VIcp19_120 = ORcp19_120+qd(1);
    VIcp19_220 = ORcp19_220+qd(2);
    VIcp19_320 = ORcp19_320+qd(3);
    OPcp19_120 = OPcp19_16+ROcp19_16*qdd(20)+qd(20)*(OMcp19_26*ROcp19_36-OMcp19_36*ROcp19_26);
    OPcp19_220 = OPcp19_26+ROcp19_26*qdd(20)-qd(20)*(OMcp19_16*ROcp19_36-OMcp19_36*ROcp19_16);
    OPcp19_320 = OPcp19_36+ROcp19_36*qdd(20)+qd(20)*(OMcp19_16*ROcp19_26-OMcp19_26*ROcp19_16);
    ACcp19_120 = qdd(1)+OMcp19_26*ORcp19_320-OMcp19_36*ORcp19_220+OPcp19_26*RLcp19_320-OPcp19_36*RLcp19_220;
    ACcp19_220 = qdd(2)-OMcp19_16*ORcp19_320+OMcp19_36*ORcp19_120-OPcp19_16*RLcp19_320+OPcp19_36*RLcp19_120;
    ACcp19_320 = qdd(3)+OMcp19_16*ORcp19_220-OMcp19_26*ORcp19_120+OPcp19_16*RLcp19_220-OPcp19_26*RLcp19_120;

% = = Block_1_0_0_20_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp19_120;
    sens.P(2) = POcp19_220;
    sens.P(3) = POcp19_320;
    sens.R(1,1) = ROcp19_16;
    sens.R(1,2) = ROcp19_26;
    sens.R(1,3) = ROcp19_36;
    sens.R(2,1) = ROcp19_420;
    sens.R(2,2) = ROcp19_520;
    sens.R(2,3) = ROcp19_620;
    sens.R(3,1) = ROcp19_720;
    sens.R(3,2) = ROcp19_820;
    sens.R(3,3) = ROcp19_920;
    sens.V(1) = VIcp19_120;
    sens.V(2) = VIcp19_220;
    sens.V(3) = VIcp19_320;
    sens.OM(1) = OMcp19_120;
    sens.OM(2) = OMcp19_220;
    sens.OM(3) = OMcp19_320;
    sens.A(1) = ACcp19_120;
    sens.A(2) = ACcp19_220;
    sens.A(3) = ACcp19_320;
    sens.OMP(1) = OPcp19_120;
    sens.OMP(2) = OPcp19_220;
    sens.OMP(3) = OPcp19_320;
 
% 
case 21, 


% = = Block_1_0_0_21_0_1 = = 
 
% Sensor Kinematics 


    ROcp20_25 = S4*S5;
    ROcp20_35 = -C4*S5;
    ROcp20_85 = -S4*C5;
    ROcp20_95 = C4*C5;
    ROcp20_16 = C5*C6;
    ROcp20_26 = ROcp20_25*C6+C4*S6;
    ROcp20_36 = ROcp20_35*C6+S4*S6;
    ROcp20_46 = -C5*S6;
    ROcp20_56 = -(ROcp20_25*S6-C4*C6);
    ROcp20_66 = -(ROcp20_35*S6-S4*C6);
    OMcp20_25 = qd(5)*C4;
    OMcp20_35 = qd(5)*S4;
    OMcp20_16 = qd(4)+qd(6)*S5;
    OMcp20_26 = OMcp20_25+ROcp20_85*qd(6);
    OMcp20_36 = OMcp20_35+ROcp20_95*qd(6);
    OPcp20_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp20_26 = ROcp20_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp20_35*S5-ROcp20_95*qd(4));
    OPcp20_36 = ROcp20_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp20_25*S5-ROcp20_85*qd(4));

% = = Block_1_0_0_21_0_8 = = 
 
% Sensor Kinematics 


    ROcp20_420 = ROcp20_46*C20+S20*S5;
    ROcp20_520 = ROcp20_56*C20+ROcp20_85*S20;
    ROcp20_620 = ROcp20_66*C20+ROcp20_95*S20;
    ROcp20_720 = -(ROcp20_46*S20-C20*S5);
    ROcp20_820 = -(ROcp20_56*S20-ROcp20_85*C20);
    ROcp20_920 = -(ROcp20_66*S20-ROcp20_95*C20);
    ROcp20_121 = ROcp20_16*C21-ROcp20_720*S21;
    ROcp20_221 = ROcp20_26*C21-ROcp20_820*S21;
    ROcp20_321 = ROcp20_36*C21-ROcp20_920*S21;
    ROcp20_721 = ROcp20_16*S21+ROcp20_720*C21;
    ROcp20_821 = ROcp20_26*S21+ROcp20_820*C21;
    ROcp20_921 = ROcp20_36*S21+ROcp20_920*C21;
    RLcp20_120 = ROcp20_46*s.dpt(2,6);
    RLcp20_220 = ROcp20_56*s.dpt(2,6);
    RLcp20_320 = ROcp20_66*s.dpt(2,6);
    POcp20_120 = RLcp20_120+q(1);
    POcp20_220 = RLcp20_220+q(2);
    POcp20_320 = RLcp20_320+q(3);
    OMcp20_120 = OMcp20_16+ROcp20_16*qd(20);
    OMcp20_220 = OMcp20_26+ROcp20_26*qd(20);
    OMcp20_320 = OMcp20_36+ROcp20_36*qd(20);
    ORcp20_120 = OMcp20_26*RLcp20_320-OMcp20_36*RLcp20_220;
    ORcp20_220 = -(OMcp20_16*RLcp20_320-OMcp20_36*RLcp20_120);
    ORcp20_320 = OMcp20_16*RLcp20_220-OMcp20_26*RLcp20_120;
    VIcp20_120 = ORcp20_120+qd(1);
    VIcp20_220 = ORcp20_220+qd(2);
    VIcp20_320 = ORcp20_320+qd(3);
    ACcp20_120 = qdd(1)+OMcp20_26*ORcp20_320-OMcp20_36*ORcp20_220+OPcp20_26*RLcp20_320-OPcp20_36*RLcp20_220;
    ACcp20_220 = qdd(2)-OMcp20_16*ORcp20_320+OMcp20_36*ORcp20_120-OPcp20_16*RLcp20_320+OPcp20_36*RLcp20_120;
    ACcp20_320 = qdd(3)+OMcp20_16*ORcp20_220-OMcp20_26*ORcp20_120+OPcp20_16*RLcp20_220-OPcp20_26*RLcp20_120;
    OMcp20_121 = OMcp20_120+ROcp20_420*qd(21);
    OMcp20_221 = OMcp20_220+ROcp20_520*qd(21);
    OMcp20_321 = OMcp20_320+ROcp20_620*qd(21);
    OPcp20_121 = OPcp20_16+ROcp20_16*qdd(20)+ROcp20_420*qdd(21)+qd(20)*(OMcp20_26*ROcp20_36-OMcp20_36*ROcp20_26)+qd(21)*(OMcp20_220*ROcp20_620-...
 OMcp20_320*ROcp20_520);
    OPcp20_221 = OPcp20_26+ROcp20_26*qdd(20)+ROcp20_520*qdd(21)-qd(20)*(OMcp20_16*ROcp20_36-OMcp20_36*ROcp20_16)-qd(21)*(OMcp20_120*ROcp20_620-...
 OMcp20_320*ROcp20_420);
    OPcp20_321 = OPcp20_36+ROcp20_36*qdd(20)+ROcp20_620*qdd(21)+qd(20)*(OMcp20_16*ROcp20_26-OMcp20_26*ROcp20_16)+qd(21)*(OMcp20_120*ROcp20_520-...
 OMcp20_220*ROcp20_420);

% = = Block_1_0_0_21_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp20_120;
    sens.P(2) = POcp20_220;
    sens.P(3) = POcp20_320;
    sens.R(1,1) = ROcp20_121;
    sens.R(1,2) = ROcp20_221;
    sens.R(1,3) = ROcp20_321;
    sens.R(2,1) = ROcp20_420;
    sens.R(2,2) = ROcp20_520;
    sens.R(2,3) = ROcp20_620;
    sens.R(3,1) = ROcp20_721;
    sens.R(3,2) = ROcp20_821;
    sens.R(3,3) = ROcp20_921;
    sens.V(1) = VIcp20_120;
    sens.V(2) = VIcp20_220;
    sens.V(3) = VIcp20_320;
    sens.OM(1) = OMcp20_121;
    sens.OM(2) = OMcp20_221;
    sens.OM(3) = OMcp20_321;
    sens.A(1) = ACcp20_120;
    sens.A(2) = ACcp20_220;
    sens.A(3) = ACcp20_320;
    sens.OMP(1) = OPcp20_121;
    sens.OMP(2) = OPcp20_221;
    sens.OMP(3) = OPcp20_321;
 
% 
case 22, 


% = = Block_1_0_0_22_0_1 = = 
 
% Sensor Kinematics 


    ROcp21_25 = S4*S5;
    ROcp21_35 = -C4*S5;
    ROcp21_85 = -S4*C5;
    ROcp21_95 = C4*C5;
    ROcp21_16 = C5*C6;
    ROcp21_26 = ROcp21_25*C6+C4*S6;
    ROcp21_36 = ROcp21_35*C6+S4*S6;
    ROcp21_46 = -C5*S6;
    ROcp21_56 = -(ROcp21_25*S6-C4*C6);
    ROcp21_66 = -(ROcp21_35*S6-S4*C6);
    OMcp21_25 = qd(5)*C4;
    OMcp21_35 = qd(5)*S4;
    OMcp21_16 = qd(4)+qd(6)*S5;
    OMcp21_26 = OMcp21_25+ROcp21_85*qd(6);
    OMcp21_36 = OMcp21_35+ROcp21_95*qd(6);
    OPcp21_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp21_26 = ROcp21_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp21_35*S5-ROcp21_95*qd(4));
    OPcp21_36 = ROcp21_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp21_25*S5-ROcp21_85*qd(4));

% = = Block_1_0_0_22_0_8 = = 
 
% Sensor Kinematics 


    ROcp21_420 = ROcp21_46*C20+S20*S5;
    ROcp21_520 = ROcp21_56*C20+ROcp21_85*S20;
    ROcp21_620 = ROcp21_66*C20+ROcp21_95*S20;
    ROcp21_720 = -(ROcp21_46*S20-C20*S5);
    ROcp21_820 = -(ROcp21_56*S20-ROcp21_85*C20);
    ROcp21_920 = -(ROcp21_66*S20-ROcp21_95*C20);
    ROcp21_121 = ROcp21_16*C21-ROcp21_720*S21;
    ROcp21_221 = ROcp21_26*C21-ROcp21_820*S21;
    ROcp21_321 = ROcp21_36*C21-ROcp21_920*S21;
    ROcp21_721 = ROcp21_16*S21+ROcp21_720*C21;
    ROcp21_821 = ROcp21_26*S21+ROcp21_820*C21;
    ROcp21_921 = ROcp21_36*S21+ROcp21_920*C21;
    RLcp21_120 = ROcp21_46*s.dpt(2,6);
    RLcp21_220 = ROcp21_56*s.dpt(2,6);
    RLcp21_320 = ROcp21_66*s.dpt(2,6);
    OMcp21_120 = OMcp21_16+ROcp21_16*qd(20);
    OMcp21_220 = OMcp21_26+ROcp21_26*qd(20);
    OMcp21_320 = OMcp21_36+ROcp21_36*qd(20);
    ORcp21_120 = OMcp21_26*RLcp21_320-OMcp21_36*RLcp21_220;
    ORcp21_220 = -(OMcp21_16*RLcp21_320-OMcp21_36*RLcp21_120);
    ORcp21_320 = OMcp21_16*RLcp21_220-OMcp21_26*RLcp21_120;
    OMcp21_121 = OMcp21_120+ROcp21_420*qd(21);
    OMcp21_221 = OMcp21_220+ROcp21_520*qd(21);
    OMcp21_321 = OMcp21_320+ROcp21_620*qd(21);
    OPcp21_121 = OPcp21_16+ROcp21_16*qdd(20)+ROcp21_420*qdd(21)+qd(20)*(OMcp21_26*ROcp21_36-OMcp21_36*ROcp21_26)+qd(21)*(OMcp21_220*ROcp21_620-...
 OMcp21_320*ROcp21_520);
    OPcp21_221 = OPcp21_26+ROcp21_26*qdd(20)+ROcp21_520*qdd(21)-qd(20)*(OMcp21_16*ROcp21_36-OMcp21_36*ROcp21_16)-qd(21)*(OMcp21_120*ROcp21_620-...
 OMcp21_320*ROcp21_420);
    OPcp21_321 = OPcp21_36+ROcp21_36*qdd(20)+ROcp21_620*qdd(21)+qd(20)*(OMcp21_16*ROcp21_26-OMcp21_26*ROcp21_16)+qd(21)*(OMcp21_120*ROcp21_520-...
 OMcp21_220*ROcp21_420);
    RLcp21_122 = Dz223*ROcp21_721;
    RLcp21_222 = Dz223*ROcp21_821;
    RLcp21_322 = Dz223*ROcp21_921;
    POcp21_122 = RLcp21_120+RLcp21_122+q(1);
    POcp21_222 = RLcp21_220+RLcp21_222+q(2);
    POcp21_322 = RLcp21_320+RLcp21_322+q(3);
    ORcp21_122 = OMcp21_221*RLcp21_322-OMcp21_321*RLcp21_222;
    ORcp21_222 = -(OMcp21_121*RLcp21_322-OMcp21_321*RLcp21_122);
    ORcp21_322 = OMcp21_121*RLcp21_222-OMcp21_221*RLcp21_122;
    VIcp21_122 = ORcp21_120+ORcp21_122+qd(1)+ROcp21_721*qd(22);
    VIcp21_222 = ORcp21_220+ORcp21_222+qd(2)+ROcp21_821*qd(22);
    VIcp21_322 = ORcp21_320+ORcp21_322+qd(3)+ROcp21_921*qd(22);
    ACcp21_122 = qdd(1)+OMcp21_221*ORcp21_322+OMcp21_26*ORcp21_320-OMcp21_321*ORcp21_222-OMcp21_36*ORcp21_220+OPcp21_221*RLcp21_322+OPcp21_26*...
 RLcp21_320-OPcp21_321*RLcp21_222-OPcp21_36*RLcp21_220+ROcp21_721*qdd(22)+(2.0)*qd(22)*(OMcp21_221*ROcp21_921-OMcp21_321*ROcp21_821);
    ACcp21_222 = qdd(2)-OMcp21_121*ORcp21_322-OMcp21_16*ORcp21_320+OMcp21_321*ORcp21_122+OMcp21_36*ORcp21_120-OPcp21_121*RLcp21_322-OPcp21_16*...
 RLcp21_320+OPcp21_321*RLcp21_122+OPcp21_36*RLcp21_120+ROcp21_821*qdd(22)-(2.0)*qd(22)*(OMcp21_121*ROcp21_921-OMcp21_321*ROcp21_721);
    ACcp21_322 = qdd(3)+OMcp21_121*ORcp21_222+OMcp21_16*ORcp21_220-OMcp21_221*ORcp21_122-OMcp21_26*ORcp21_120+OPcp21_121*RLcp21_222+OPcp21_16*...
 RLcp21_220-OPcp21_221*RLcp21_122-OPcp21_26*RLcp21_120+ROcp21_921*qdd(22)+(2.0)*qd(22)*(OMcp21_121*ROcp21_821-OMcp21_221*ROcp21_721);

% = = Block_1_0_0_22_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp21_122;
    sens.P(2) = POcp21_222;
    sens.P(3) = POcp21_322;
    sens.R(1,1) = ROcp21_121;
    sens.R(1,2) = ROcp21_221;
    sens.R(1,3) = ROcp21_321;
    sens.R(2,1) = ROcp21_420;
    sens.R(2,2) = ROcp21_520;
    sens.R(2,3) = ROcp21_620;
    sens.R(3,1) = ROcp21_721;
    sens.R(3,2) = ROcp21_821;
    sens.R(3,3) = ROcp21_921;
    sens.V(1) = VIcp21_122;
    sens.V(2) = VIcp21_222;
    sens.V(3) = VIcp21_322;
    sens.OM(1) = OMcp21_121;
    sens.OM(2) = OMcp21_221;
    sens.OM(3) = OMcp21_321;
    sens.A(1) = ACcp21_122;
    sens.A(2) = ACcp21_222;
    sens.A(3) = ACcp21_322;
    sens.OMP(1) = OPcp21_121;
    sens.OMP(2) = OPcp21_221;
    sens.OMP(3) = OPcp21_321;

end


% ====== END Task 1 ====== 

  

