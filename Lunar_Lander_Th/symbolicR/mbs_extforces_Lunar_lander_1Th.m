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
%	==> Function : F19 : External Forces
%	==> Flops complexity : 1328
%
%	==> Generation Time :  0.010 seconds
%	==> Post-Processing :  0.030 seconds
%
%-------------------------------------------------------------
%
function [frc,trq] = extforces(s,tsim,usrfun)

 frc = zeros(3,16);
 trq = zeros(3,16);

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

% = = Block_0_0_1_1_0_1 = = 
 
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
  OMcp0_26 = OMcp0_25+qd(6)*ROcp0_85;
  OMcp0_36 = OMcp0_35+qd(6)*ROcp0_95;

% = = Block_0_0_1_1_0_2 = = 
 
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
  OMcp0_17 = OMcp0_16+qd(7)*ROcp0_16;
  OMcp0_27 = OMcp0_26+qd(7)*ROcp0_26;
  OMcp0_37 = OMcp0_36+qd(7)*ROcp0_36;
  OMcp0_18 = OMcp0_17+qd(8)*ROcp0_47;
  OMcp0_28 = OMcp0_27+qd(8)*ROcp0_57;
  OMcp0_38 = OMcp0_37+qd(8)*ROcp0_67;
  OPcp0_18 = qdd(4)+qd(5)*qd(6)*C5+qd(7)*(OMcp0_26*ROcp0_36-OMcp0_36*ROcp0_26)+qd(8)*(OMcp0_27*ROcp0_67-OMcp0_37*ROcp0_57)+qdd(6)*S5+qdd(7)*...
 ROcp0_16+qdd(8)*ROcp0_47;
  OPcp0_28 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp0_95-OMcp0_35*S5)+qd(7)*(OMcp0_16*ROcp0_36-OMcp0_36*ROcp0_16)+qd(8)*(OMcp0_17*ROcp0_67-OMcp0_37*...
 ROcp0_47)-qdd(5)*C4-qdd(6)*ROcp0_85-qdd(7)*ROcp0_26-qdd(8)*ROcp0_57);
  OPcp0_38 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp0_85-OMcp0_25*S5)+qd(7)*(OMcp0_16*ROcp0_26-OMcp0_26*ROcp0_16)+qd(8)*(OMcp0_17*ROcp0_57-OMcp0_27*...
 ROcp0_47)+qdd(5)*S4+qdd(6)*ROcp0_95+qdd(7)*ROcp0_36+qdd(8)*ROcp0_67;
  RLcp0_137 = ROcp0_78*s.dpt(3,17);
  RLcp0_237 = ROcp0_88*s.dpt(3,17);
  RLcp0_337 = ROcp0_98*s.dpt(3,17);
  ORcp0_137 = OMcp0_28*RLcp0_337-OMcp0_38*RLcp0_237;
  ORcp0_237 = -(OMcp0_18*RLcp0_337-OMcp0_38*RLcp0_137);
  ORcp0_337 = OMcp0_18*RLcp0_237-OMcp0_28*RLcp0_137;
  PxF1(1) = q(1)+RLcp0_137;
  PxF1(2) = q(2)+RLcp0_237;
  PxF1(3) = q(3)+RLcp0_337;
  RxF1(1,1) = ROcp0_18;
  RxF1(1,2) = ROcp0_28;
  RxF1(1,3) = ROcp0_38;
  RxF1(2,1) = ROcp0_47;
  RxF1(2,2) = ROcp0_57;
  RxF1(2,3) = ROcp0_67;
  RxF1(3,1) = ROcp0_78;
  RxF1(3,2) = ROcp0_88;
  RxF1(3,3) = ROcp0_98;
  VxF1(1) = qd(1)+ORcp0_137;
  VxF1(2) = qd(2)+ORcp0_237;
  VxF1(3) = qd(3)+ORcp0_337;
  OMxF1(1) = OMcp0_18;
  OMxF1(2) = OMcp0_28;
  OMxF1(3) = OMcp0_38;
  AxF1(1) = qdd(1)+OMcp0_28*ORcp0_337-OMcp0_38*ORcp0_237+OPcp0_28*RLcp0_337-OPcp0_38*RLcp0_237;
  AxF1(2) = qdd(2)-OMcp0_18*ORcp0_337+OMcp0_38*ORcp0_137-OPcp0_18*RLcp0_337+OPcp0_38*RLcp0_137;
  AxF1(3) = qdd(3)+OMcp0_18*ORcp0_237-OMcp0_28*ORcp0_137+OPcp0_18*RLcp0_237-OPcp0_28*RLcp0_137;
  OMPxF1(1) = OPcp0_18;
  OMPxF1(2) = OPcp0_28;
  OMPxF1(3) = OPcp0_38;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc11 = ROcp0_18*SWr1(1)+ROcp0_28*SWr1(2)+ROcp0_38*SWr1(3);
  xfrc21 = ROcp0_47*SWr1(1)+ROcp0_57*SWr1(2)+ROcp0_67*SWr1(3);
  xfrc31 = ROcp0_78*SWr1(1)+ROcp0_88*SWr1(2)+ROcp0_98*SWr1(3);
  frc(1,8) = s.frc(1,8)+xfrc11;
  frc(2,8) = s.frc(2,8)+xfrc21;
  frc(3,8) = s.frc(3,8)+xfrc31;
  xtrq11 = ROcp0_18*SWr1(4)+ROcp0_28*SWr1(5)+ROcp0_38*SWr1(6);
  xtrq21 = ROcp0_47*SWr1(4)+ROcp0_57*SWr1(5)+ROcp0_67*SWr1(6);
  xtrq31 = ROcp0_78*SWr1(4)+ROcp0_88*SWr1(5)+ROcp0_98*SWr1(6);
  trq(1,8) = s.trq(1,8)+xtrq11-xfrc21*(SWr1(9)-s.l(3,8))+xfrc31*SWr1(8);
  trq(2,8) = s.trq(2,8)+xtrq21+xfrc11*(SWr1(9)-s.l(3,8))-xfrc31*SWr1(7);
  trq(3,8) = s.trq(3,8)+xtrq31-xfrc11*SWr1(8)+xfrc21*SWr1(7);

% = = Block_0_0_1_2_0_1 = = 
 
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
  OMcp1_26 = OMcp1_25+qd(6)*ROcp1_85;
  OMcp1_36 = OMcp1_35+qd(6)*ROcp1_95;
  OPcp1_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp1_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp1_95-OMcp1_35*S5)-qdd(5)*C4-qdd(6)*ROcp1_85);
  OPcp1_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp1_85-OMcp1_25*S5)+qdd(5)*S4+qdd(6)*ROcp1_95;

% = = Block_0_0_1_2_0_3 = = 
 
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
  OMcp1_19 = OMcp1_16+qd(9)*ROcp1_46;
  OMcp1_29 = OMcp1_26+qd(9)*ROcp1_56;
  OMcp1_39 = OMcp1_36+qd(9)*ROcp1_66;
  ORcp1_19 = OMcp1_26*RLcp1_39-OMcp1_36*RLcp1_29;
  ORcp1_29 = -(OMcp1_16*RLcp1_39-OMcp1_36*RLcp1_19);
  ORcp1_39 = OMcp1_16*RLcp1_29-OMcp1_26*RLcp1_19;
  OMcp1_110 = OMcp1_19+qd(10)*ROcp1_19;
  OMcp1_210 = OMcp1_29+qd(10)*ROcp1_29;
  OMcp1_310 = OMcp1_39+qd(10)*ROcp1_39;
  OPcp1_110 = OPcp1_16+qd(10)*(OMcp1_29*ROcp1_39-OMcp1_39*ROcp1_29)+qd(9)*(OMcp1_26*ROcp1_66-OMcp1_36*ROcp1_56)+qdd(10)*ROcp1_19+qdd(9)*ROcp1_46;
  OPcp1_210 = OPcp1_26-qd(10)*(OMcp1_19*ROcp1_39-OMcp1_39*ROcp1_19)-qd(9)*(OMcp1_16*ROcp1_66-OMcp1_36*ROcp1_46)+qdd(10)*ROcp1_29+qdd(9)*ROcp1_56;
  OPcp1_310 = OPcp1_36+qd(10)*(OMcp1_19*ROcp1_29-OMcp1_29*ROcp1_19)+qd(9)*(OMcp1_16*ROcp1_56-OMcp1_26*ROcp1_46)+qdd(10)*ROcp1_39+qdd(9)*ROcp1_66;
  RLcp1_138 = ROcp1_710*s.dpt(3,18);
  RLcp1_238 = ROcp1_810*s.dpt(3,18);
  RLcp1_338 = ROcp1_910*s.dpt(3,18);
  ORcp1_138 = OMcp1_210*RLcp1_338-OMcp1_310*RLcp1_238;
  ORcp1_238 = -(OMcp1_110*RLcp1_338-OMcp1_310*RLcp1_138);
  ORcp1_338 = OMcp1_110*RLcp1_238-OMcp1_210*RLcp1_138;
  PxF2(1) = q(1)+RLcp1_138+RLcp1_19;
  PxF2(2) = q(2)+RLcp1_238+RLcp1_29;
  PxF2(3) = q(3)+RLcp1_338+RLcp1_39;
  RxF2(1,1) = ROcp1_19;
  RxF2(1,2) = ROcp1_29;
  RxF2(1,3) = ROcp1_39;
  RxF2(2,1) = ROcp1_410;
  RxF2(2,2) = ROcp1_510;
  RxF2(2,3) = ROcp1_610;
  RxF2(3,1) = ROcp1_710;
  RxF2(3,2) = ROcp1_810;
  RxF2(3,3) = ROcp1_910;
  VxF2(1) = qd(1)+ORcp1_138+ORcp1_19;
  VxF2(2) = qd(2)+ORcp1_238+ORcp1_29;
  VxF2(3) = qd(3)+ORcp1_338+ORcp1_39;
  OMxF2(1) = OMcp1_110;
  OMxF2(2) = OMcp1_210;
  OMxF2(3) = OMcp1_310;
  AxF2(1) = qdd(1)+OMcp1_210*ORcp1_338+OMcp1_26*ORcp1_39-OMcp1_310*ORcp1_238-OMcp1_36*ORcp1_29+OPcp1_210*RLcp1_338+OPcp1_26*RLcp1_39-OPcp1_310*...
 RLcp1_238-OPcp1_36*RLcp1_29;
  AxF2(2) = qdd(2)-OMcp1_110*ORcp1_338-OMcp1_16*ORcp1_39+OMcp1_310*ORcp1_138+OMcp1_36*ORcp1_19-OPcp1_110*RLcp1_338-OPcp1_16*RLcp1_39+OPcp1_310*...
 RLcp1_138+OPcp1_36*RLcp1_19;
  AxF2(3) = qdd(3)+OMcp1_110*ORcp1_238+OMcp1_16*ORcp1_29-OMcp1_210*ORcp1_138-OMcp1_26*ORcp1_19+OPcp1_110*RLcp1_238+OPcp1_16*RLcp1_29-OPcp1_210*...
 RLcp1_138-OPcp1_26*RLcp1_19;
  OMPxF2(1) = OPcp1_110;
  OMPxF2(2) = OPcp1_210;
  OMPxF2(3) = OPcp1_310;
 
% Sensor Forces Computation 

  SWr2 = usrfun.fext(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc12 = ROcp1_19*SWr2(1)+ROcp1_29*SWr2(2)+ROcp1_39*SWr2(3);
  xfrc22 = ROcp1_410*SWr2(1)+ROcp1_510*SWr2(2)+ROcp1_610*SWr2(3);
  xfrc32 = ROcp1_710*SWr2(1)+ROcp1_810*SWr2(2)+ROcp1_910*SWr2(3);
  frc(1,10) = s.frc(1,10)+xfrc12;
  frc(2,10) = s.frc(2,10)+xfrc22;
  frc(3,10) = s.frc(3,10)+xfrc32;
  xtrq12 = ROcp1_19*SWr2(4)+ROcp1_29*SWr2(5)+ROcp1_39*SWr2(6);
  xtrq22 = ROcp1_410*SWr2(4)+ROcp1_510*SWr2(5)+ROcp1_610*SWr2(6);
  xtrq32 = ROcp1_710*SWr2(4)+ROcp1_810*SWr2(5)+ROcp1_910*SWr2(6);
  trq(1,10) = s.trq(1,10)+xtrq12-xfrc22*(SWr2(9)-s.l(3,10))+xfrc32*SWr2(8);
  trq(2,10) = s.trq(2,10)+xtrq22+xfrc12*(SWr2(9)-s.l(3,10))-xfrc32*SWr2(7);
  trq(3,10) = s.trq(3,10)+xtrq32-xfrc12*SWr2(8)+xfrc22*SWr2(7);

% = = Block_0_0_1_3_0_1 = = 
 
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
  OMcp2_26 = OMcp2_25+qd(6)*ROcp2_85;
  OMcp2_36 = OMcp2_35+qd(6)*ROcp2_95;
  OPcp2_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp2_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp2_95-OMcp2_35*S5)-qdd(5)*C4-qdd(6)*ROcp2_85);
  OPcp2_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp2_85-OMcp2_25*S5)+qdd(5)*S4+qdd(6)*ROcp2_95;

% = = Block_0_0_1_3_0_4 = = 
 
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
  OMcp2_111 = OMcp2_16+qd(11)*ROcp2_46;
  OMcp2_211 = OMcp2_26+qd(11)*ROcp2_56;
  OMcp2_311 = OMcp2_36+qd(11)*ROcp2_66;
  ORcp2_111 = OMcp2_26*RLcp2_311-OMcp2_36*RLcp2_211;
  ORcp2_211 = -(OMcp2_16*RLcp2_311-OMcp2_36*RLcp2_111);
  ORcp2_311 = OMcp2_16*RLcp2_211-OMcp2_26*RLcp2_111;
  OMcp2_112 = OMcp2_111+qd(12)*ROcp2_111;
  OMcp2_212 = OMcp2_211+qd(12)*ROcp2_211;
  OMcp2_312 = OMcp2_311+qd(12)*ROcp2_311;
  OPcp2_112 = OPcp2_16+qd(11)*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56)+qd(12)*(OMcp2_211*ROcp2_311-OMcp2_311*ROcp2_211)+qdd(11)*ROcp2_46+qdd(12)*...
 ROcp2_111;
  OPcp2_212 = OPcp2_26-qd(11)*(OMcp2_16*ROcp2_66-OMcp2_36*ROcp2_46)-qd(12)*(OMcp2_111*ROcp2_311-OMcp2_311*ROcp2_111)+qdd(11)*ROcp2_56+qdd(12)*...
 ROcp2_211;
  OPcp2_312 = OPcp2_36+qd(11)*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46)+qd(12)*(OMcp2_111*ROcp2_211-OMcp2_211*ROcp2_111)+qdd(11)*ROcp2_66+qdd(12)*...
 ROcp2_311;
  RLcp2_139 = ROcp2_712*s.dpt(3,20);
  RLcp2_239 = ROcp2_812*s.dpt(3,20);
  RLcp2_339 = ROcp2_912*s.dpt(3,20);
  ORcp2_139 = OMcp2_212*RLcp2_339-OMcp2_312*RLcp2_239;
  ORcp2_239 = -(OMcp2_112*RLcp2_339-OMcp2_312*RLcp2_139);
  ORcp2_339 = OMcp2_112*RLcp2_239-OMcp2_212*RLcp2_139;
  PxF3(1) = q(1)+RLcp2_111+RLcp2_139;
  PxF3(2) = q(2)+RLcp2_211+RLcp2_239;
  PxF3(3) = q(3)+RLcp2_311+RLcp2_339;
  RxF3(1,1) = ROcp2_111;
  RxF3(1,2) = ROcp2_211;
  RxF3(1,3) = ROcp2_311;
  RxF3(2,1) = ROcp2_412;
  RxF3(2,2) = ROcp2_512;
  RxF3(2,3) = ROcp2_612;
  RxF3(3,1) = ROcp2_712;
  RxF3(3,2) = ROcp2_812;
  RxF3(3,3) = ROcp2_912;
  VxF3(1) = qd(1)+ORcp2_111+ORcp2_139;
  VxF3(2) = qd(2)+ORcp2_211+ORcp2_239;
  VxF3(3) = qd(3)+ORcp2_311+ORcp2_339;
  OMxF3(1) = OMcp2_112;
  OMxF3(2) = OMcp2_212;
  OMxF3(3) = OMcp2_312;
  AxF3(1) = qdd(1)+OMcp2_212*ORcp2_339+OMcp2_26*ORcp2_311-OMcp2_312*ORcp2_239-OMcp2_36*ORcp2_211+OPcp2_212*RLcp2_339+OPcp2_26*RLcp2_311-OPcp2_312...
 *RLcp2_239-OPcp2_36*RLcp2_211;
  AxF3(2) = qdd(2)-OMcp2_112*ORcp2_339-OMcp2_16*ORcp2_311+OMcp2_312*ORcp2_139+OMcp2_36*ORcp2_111-OPcp2_112*RLcp2_339-OPcp2_16*RLcp2_311+OPcp2_312...
 *RLcp2_139+OPcp2_36*RLcp2_111;
  AxF3(3) = qdd(3)+OMcp2_112*ORcp2_239+OMcp2_16*ORcp2_211-OMcp2_212*ORcp2_139-OMcp2_26*ORcp2_111+OPcp2_112*RLcp2_239+OPcp2_16*RLcp2_211-OPcp2_212...
 *RLcp2_139-OPcp2_26*RLcp2_111;
  OMPxF3(1) = OPcp2_112;
  OMPxF3(2) = OPcp2_212;
  OMPxF3(3) = OPcp2_312;
 
% Sensor Forces Computation 

  SWr3 = usrfun.fext(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc13 = ROcp2_111*SWr3(1)+ROcp2_211*SWr3(2)+ROcp2_311*SWr3(3);
  xfrc23 = ROcp2_412*SWr3(1)+ROcp2_512*SWr3(2)+ROcp2_612*SWr3(3);
  xfrc33 = ROcp2_712*SWr3(1)+ROcp2_812*SWr3(2)+ROcp2_912*SWr3(3);
  frc(1,12) = s.frc(1,12)+xfrc13;
  frc(2,12) = s.frc(2,12)+xfrc23;
  frc(3,12) = s.frc(3,12)+xfrc33;
  xtrq13 = ROcp2_111*SWr3(4)+ROcp2_211*SWr3(5)+ROcp2_311*SWr3(6);
  xtrq23 = ROcp2_412*SWr3(4)+ROcp2_512*SWr3(5)+ROcp2_612*SWr3(6);
  xtrq33 = ROcp2_712*SWr3(4)+ROcp2_812*SWr3(5)+ROcp2_912*SWr3(6);
  trq(1,12) = s.trq(1,12)+xtrq13-xfrc23*(SWr3(9)-s.l(3,12))+xfrc33*SWr3(8);
  trq(2,12) = s.trq(2,12)+xtrq23+xfrc13*(SWr3(9)-s.l(3,12))-xfrc33*SWr3(7);
  trq(3,12) = s.trq(3,12)+xtrq33-xfrc13*SWr3(8)+xfrc23*SWr3(7);

% = = Block_0_0_1_4_0_1 = = 
 
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
  OMcp3_26 = OMcp3_25+qd(6)*ROcp3_85;
  OMcp3_36 = OMcp3_35+qd(6)*ROcp3_95;
  OPcp3_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp3_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp3_95-OMcp3_35*S5)-qdd(5)*C4-qdd(6)*ROcp3_85);
  OPcp3_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp3_85-OMcp3_25*S5)+qdd(5)*S4+qdd(6)*ROcp3_95;

% = = Block_0_0_1_4_0_5 = = 
 
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
  OMcp3_113 = OMcp3_16+qd(13)*ROcp3_46;
  OMcp3_213 = OMcp3_26+qd(13)*ROcp3_56;
  OMcp3_313 = OMcp3_36+qd(13)*ROcp3_66;
  ORcp3_113 = OMcp3_26*RLcp3_313-OMcp3_36*RLcp3_213;
  ORcp3_213 = -(OMcp3_16*RLcp3_313-OMcp3_36*RLcp3_113);
  ORcp3_313 = OMcp3_16*RLcp3_213-OMcp3_26*RLcp3_113;
  OMcp3_114 = OMcp3_113+qd(14)*ROcp3_113;
  OMcp3_214 = OMcp3_213+qd(14)*ROcp3_213;
  OMcp3_314 = OMcp3_313+qd(14)*ROcp3_313;
  OPcp3_114 = OPcp3_16+qd(13)*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)+qd(14)*(OMcp3_213*ROcp3_313-OMcp3_313*ROcp3_213)+qdd(13)*ROcp3_46+qdd(14)*...
 ROcp3_113;
  OPcp3_214 = OPcp3_26-qd(13)*(OMcp3_16*ROcp3_66-OMcp3_36*ROcp3_46)-qd(14)*(OMcp3_113*ROcp3_313-OMcp3_313*ROcp3_113)+qdd(13)*ROcp3_56+qdd(14)*...
 ROcp3_213;
  OPcp3_314 = OPcp3_36+qd(13)*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)+qd(14)*(OMcp3_113*ROcp3_213-OMcp3_213*ROcp3_113)+qdd(13)*ROcp3_66+qdd(14)*...
 ROcp3_313;
  RLcp3_140 = ROcp3_714*s.dpt(3,22);
  RLcp3_240 = ROcp3_814*s.dpt(3,22);
  RLcp3_340 = ROcp3_914*s.dpt(3,22);
  ORcp3_140 = OMcp3_214*RLcp3_340-OMcp3_314*RLcp3_240;
  ORcp3_240 = -(OMcp3_114*RLcp3_340-OMcp3_314*RLcp3_140);
  ORcp3_340 = OMcp3_114*RLcp3_240-OMcp3_214*RLcp3_140;
  PxF4(1) = q(1)+RLcp3_113+RLcp3_140;
  PxF4(2) = q(2)+RLcp3_213+RLcp3_240;
  PxF4(3) = q(3)+RLcp3_313+RLcp3_340;
  RxF4(1,1) = ROcp3_113;
  RxF4(1,2) = ROcp3_213;
  RxF4(1,3) = ROcp3_313;
  RxF4(2,1) = ROcp3_414;
  RxF4(2,2) = ROcp3_514;
  RxF4(2,3) = ROcp3_614;
  RxF4(3,1) = ROcp3_714;
  RxF4(3,2) = ROcp3_814;
  RxF4(3,3) = ROcp3_914;
  VxF4(1) = qd(1)+ORcp3_113+ORcp3_140;
  VxF4(2) = qd(2)+ORcp3_213+ORcp3_240;
  VxF4(3) = qd(3)+ORcp3_313+ORcp3_340;
  OMxF4(1) = OMcp3_114;
  OMxF4(2) = OMcp3_214;
  OMxF4(3) = OMcp3_314;
  AxF4(1) = qdd(1)+OMcp3_214*ORcp3_340+OMcp3_26*ORcp3_313-OMcp3_314*ORcp3_240-OMcp3_36*ORcp3_213+OPcp3_214*RLcp3_340+OPcp3_26*RLcp3_313-OPcp3_314...
 *RLcp3_240-OPcp3_36*RLcp3_213;
  AxF4(2) = qdd(2)-OMcp3_114*ORcp3_340-OMcp3_16*ORcp3_313+OMcp3_314*ORcp3_140+OMcp3_36*ORcp3_113-OPcp3_114*RLcp3_340-OPcp3_16*RLcp3_313+OPcp3_314...
 *RLcp3_140+OPcp3_36*RLcp3_113;
  AxF4(3) = qdd(3)+OMcp3_114*ORcp3_240+OMcp3_16*ORcp3_213-OMcp3_214*ORcp3_140-OMcp3_26*ORcp3_113+OPcp3_114*RLcp3_240+OPcp3_16*RLcp3_213-OPcp3_214...
 *RLcp3_140-OPcp3_26*RLcp3_113;
  OMPxF4(1) = OPcp3_114;
  OMPxF4(2) = OPcp3_214;
  OMPxF4(3) = OPcp3_314;
 
% Sensor Forces Computation 

  SWr4 = usrfun.fext(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc14 = ROcp3_113*SWr4(1)+ROcp3_213*SWr4(2)+ROcp3_313*SWr4(3);
  xfrc24 = ROcp3_414*SWr4(1)+ROcp3_514*SWr4(2)+ROcp3_614*SWr4(3);
  xfrc34 = ROcp3_714*SWr4(1)+ROcp3_814*SWr4(2)+ROcp3_914*SWr4(3);
  frc(1,14) = s.frc(1,14)+xfrc14;
  frc(2,14) = s.frc(2,14)+xfrc24;
  frc(3,14) = s.frc(3,14)+xfrc34;
  xtrq14 = ROcp3_113*SWr4(4)+ROcp3_213*SWr4(5)+ROcp3_313*SWr4(6);
  xtrq24 = ROcp3_414*SWr4(4)+ROcp3_514*SWr4(5)+ROcp3_614*SWr4(6);
  xtrq34 = ROcp3_714*SWr4(4)+ROcp3_814*SWr4(5)+ROcp3_914*SWr4(6);
  trq(1,14) = s.trq(1,14)+xtrq14-xfrc24*(SWr4(9)-s.l(3,14))+xfrc34*SWr4(8);
  trq(2,14) = s.trq(2,14)+xtrq24+xfrc14*(SWr4(9)-s.l(3,14))-xfrc34*SWr4(7);
  trq(3,14) = s.trq(3,14)+xtrq34-xfrc14*SWr4(8)+xfrc24*SWr4(7);

% = = Block_0_0_1_5_0_1 = = 
 
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
  OMcp4_26 = OMcp4_25+qd(6)*ROcp4_85;
  OMcp4_36 = OMcp4_35+qd(6)*ROcp4_95;
  OPcp4_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp4_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp4_95-OMcp4_35*S5)-qdd(5)*C4-qdd(6)*ROcp4_85);
  OPcp4_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp4_85-OMcp4_25*S5)+qdd(5)*S4+qdd(6)*ROcp4_95;

% = = Block_0_0_1_5_0_6 = = 
 
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
  OMcp4_115 = OMcp4_16+qd(15)*ROcp4_46;
  OMcp4_215 = OMcp4_26+qd(15)*ROcp4_56;
  OMcp4_315 = OMcp4_36+qd(15)*ROcp4_66;
  ORcp4_115 = OMcp4_26*RLcp4_315-OMcp4_36*RLcp4_215;
  ORcp4_215 = -(OMcp4_16*RLcp4_315-OMcp4_36*RLcp4_115);
  ORcp4_315 = OMcp4_16*RLcp4_215-OMcp4_26*RLcp4_115;
  OMcp4_116 = OMcp4_115+qd(16)*ROcp4_115;
  OMcp4_216 = OMcp4_215+qd(16)*ROcp4_215;
  OMcp4_316 = OMcp4_315+qd(16)*ROcp4_315;
  OPcp4_116 = OPcp4_16+qd(15)*(OMcp4_26*ROcp4_66-OMcp4_36*ROcp4_56)+qd(16)*(OMcp4_215*ROcp4_315-OMcp4_315*ROcp4_215)+qdd(15)*ROcp4_46+qdd(16)*...
 ROcp4_115;
  OPcp4_216 = OPcp4_26-qd(15)*(OMcp4_16*ROcp4_66-OMcp4_36*ROcp4_46)-qd(16)*(OMcp4_115*ROcp4_315-OMcp4_315*ROcp4_115)+qdd(15)*ROcp4_56+qdd(16)*...
 ROcp4_215;
  OPcp4_316 = OPcp4_36+qd(15)*(OMcp4_16*ROcp4_56-OMcp4_26*ROcp4_46)+qd(16)*(OMcp4_115*ROcp4_215-OMcp4_215*ROcp4_115)+qdd(15)*ROcp4_66+qdd(16)*...
 ROcp4_315;
  RLcp4_141 = ROcp4_716*s.dpt(3,24);
  RLcp4_241 = ROcp4_816*s.dpt(3,24);
  RLcp4_341 = ROcp4_916*s.dpt(3,24);
  ORcp4_141 = OMcp4_216*RLcp4_341-OMcp4_316*RLcp4_241;
  ORcp4_241 = -(OMcp4_116*RLcp4_341-OMcp4_316*RLcp4_141);
  ORcp4_341 = OMcp4_116*RLcp4_241-OMcp4_216*RLcp4_141;
  PxF5(1) = q(1)+RLcp4_115+RLcp4_141;
  PxF5(2) = q(2)+RLcp4_215+RLcp4_241;
  PxF5(3) = q(3)+RLcp4_315+RLcp4_341;
  RxF5(1,1) = ROcp4_115;
  RxF5(1,2) = ROcp4_215;
  RxF5(1,3) = ROcp4_315;
  RxF5(2,1) = ROcp4_416;
  RxF5(2,2) = ROcp4_516;
  RxF5(2,3) = ROcp4_616;
  RxF5(3,1) = ROcp4_716;
  RxF5(3,2) = ROcp4_816;
  RxF5(3,3) = ROcp4_916;
  VxF5(1) = qd(1)+ORcp4_115+ORcp4_141;
  VxF5(2) = qd(2)+ORcp4_215+ORcp4_241;
  VxF5(3) = qd(3)+ORcp4_315+ORcp4_341;
  OMxF5(1) = OMcp4_116;
  OMxF5(2) = OMcp4_216;
  OMxF5(3) = OMcp4_316;
  AxF5(1) = qdd(1)+OMcp4_216*ORcp4_341+OMcp4_26*ORcp4_315-OMcp4_316*ORcp4_241-OMcp4_36*ORcp4_215+OPcp4_216*RLcp4_341+OPcp4_26*RLcp4_315-OPcp4_316...
 *RLcp4_241-OPcp4_36*RLcp4_215;
  AxF5(2) = qdd(2)-OMcp4_116*ORcp4_341-OMcp4_16*ORcp4_315+OMcp4_316*ORcp4_141+OMcp4_36*ORcp4_115-OPcp4_116*RLcp4_341-OPcp4_16*RLcp4_315+OPcp4_316...
 *RLcp4_141+OPcp4_36*RLcp4_115;
  AxF5(3) = qdd(3)+OMcp4_116*ORcp4_241+OMcp4_16*ORcp4_215-OMcp4_216*ORcp4_141-OMcp4_26*ORcp4_115+OPcp4_116*RLcp4_241+OPcp4_16*RLcp4_215-OPcp4_216...
 *RLcp4_141-OPcp4_26*RLcp4_115;
  OMPxF5(1) = OPcp4_116;
  OMPxF5(2) = OPcp4_216;
  OMPxF5(3) = OPcp4_316;
 
% Sensor Forces Computation 

  SWr5 = usrfun.fext(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_115*SWr5(1)+ROcp4_215*SWr5(2)+ROcp4_315*SWr5(3);
  xfrc25 = ROcp4_416*SWr5(1)+ROcp4_516*SWr5(2)+ROcp4_616*SWr5(3);
  xfrc35 = ROcp4_716*SWr5(1)+ROcp4_816*SWr5(2)+ROcp4_916*SWr5(3);
  frc(1,16) = s.frc(1,16)+xfrc15;
  frc(2,16) = s.frc(2,16)+xfrc25;
  frc(3,16) = s.frc(3,16)+xfrc35;
  xtrq15 = ROcp4_115*SWr5(4)+ROcp4_215*SWr5(5)+ROcp4_315*SWr5(6);
  xtrq25 = ROcp4_416*SWr5(4)+ROcp4_516*SWr5(5)+ROcp4_616*SWr5(6);
  xtrq35 = ROcp4_716*SWr5(4)+ROcp4_816*SWr5(5)+ROcp4_916*SWr5(6);
  trq(1,16) = s.trq(1,16)+xtrq15-xfrc25*(SWr5(9)-s.l(3,16))+xfrc35*SWr5(8);
  trq(2,16) = s.trq(2,16)+xtrq25+xfrc15*(SWr5(9)-s.l(3,16))-xfrc35*SWr5(7);
  trq(3,16) = s.trq(3,16)+xtrq35-xfrc15*SWr5(8)+xfrc25*SWr5(7);

% = = Block_0_0_1_5_1_0 = = 
 
% Symbolic Outputs  

  frc(1,6) = s.frc(1,6);
  frc(2,6) = s.frc(2,6);
  frc(3,6) = s.frc(3,6);
  trq(1,6) = s.trq(1,6);
  trq(2,6) = s.trq(2,6);
  trq(3,6) = s.trq(3,6);

% ====== END Task 0 ====== 

  

