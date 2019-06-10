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
%	==> Function : F19 : External Forces
%	==> Flops complexity : 1328
%
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.020 seconds
%
%-------------------------------------------------------------
%
function [frc,trq] = extforces(s,tsim,usrfun)

 frc = zeros(3,22);
 trq = zeros(3,22);

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

% = = Block_0_0_1_1_0_2 = = 
 
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
  OMcp1_17 = OMcp1_16+qd(7)*ROcp1_16;
  OMcp1_27 = OMcp1_26+qd(7)*ROcp1_26;
  OMcp1_37 = OMcp1_36+qd(7)*ROcp1_36;
  OMcp1_18 = OMcp1_17+qd(8)*ROcp1_47;
  OMcp1_28 = OMcp1_27+qd(8)*ROcp1_57;
  OMcp1_38 = OMcp1_37+qd(8)*ROcp1_67;
  OPcp1_18 = qdd(4)+qd(5)*qd(6)*C5+qd(7)*(OMcp1_26*ROcp1_36-OMcp1_36*ROcp1_26)+qd(8)*(OMcp1_27*ROcp1_67-OMcp1_37*ROcp1_57)+qdd(6)*S5+qdd(7)*...
 ROcp1_16+qdd(8)*ROcp1_47;
  OPcp1_28 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp1_95-OMcp1_35*S5)+qd(7)*(OMcp1_16*ROcp1_36-OMcp1_36*ROcp1_16)+qd(8)*(OMcp1_17*ROcp1_67-OMcp1_37*...
 ROcp1_47)-qdd(5)*C4-qdd(6)*ROcp1_85-qdd(7)*ROcp1_26-qdd(8)*ROcp1_57);
  OPcp1_38 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp1_85-OMcp1_25*S5)+qd(7)*(OMcp1_16*ROcp1_26-OMcp1_26*ROcp1_16)+qd(8)*(OMcp1_17*ROcp1_57-OMcp1_27*...
 ROcp1_47)+qdd(5)*S4+qdd(6)*ROcp1_95+qdd(7)*ROcp1_36+qdd(8)*ROcp1_67;
  RLcp1_128 = ROcp1_78*s.dpt(3,10);
  RLcp1_228 = ROcp1_88*s.dpt(3,10);
  RLcp1_328 = ROcp1_98*s.dpt(3,10);
  ORcp1_128 = OMcp1_28*RLcp1_328-OMcp1_38*RLcp1_228;
  ORcp1_228 = -(OMcp1_18*RLcp1_328-OMcp1_38*RLcp1_128);
  ORcp1_328 = OMcp1_18*RLcp1_228-OMcp1_28*RLcp1_128;
  PxF1(1) = q(1)+RLcp1_128;
  PxF1(2) = q(2)+RLcp1_228;
  PxF1(3) = q(3)+RLcp1_328;
  RxF1(1,1) = ROcp1_18;
  RxF1(1,2) = ROcp1_28;
  RxF1(1,3) = ROcp1_38;
  RxF1(2,1) = ROcp1_47;
  RxF1(2,2) = ROcp1_57;
  RxF1(2,3) = ROcp1_67;
  RxF1(3,1) = ROcp1_78;
  RxF1(3,2) = ROcp1_88;
  RxF1(3,3) = ROcp1_98;
  VxF1(1) = qd(1)+ORcp1_128;
  VxF1(2) = qd(2)+ORcp1_228;
  VxF1(3) = qd(3)+ORcp1_328;
  OMxF1(1) = OMcp1_18;
  OMxF1(2) = OMcp1_28;
  OMxF1(3) = OMcp1_38;
  AxF1(1) = qdd(1)+OMcp1_28*ORcp1_328-OMcp1_38*ORcp1_228+OPcp1_28*RLcp1_328-OPcp1_38*RLcp1_228;
  AxF1(2) = qdd(2)-OMcp1_18*ORcp1_328+OMcp1_38*ORcp1_128-OPcp1_18*RLcp1_328+OPcp1_38*RLcp1_128;
  AxF1(3) = qdd(3)+OMcp1_18*ORcp1_228-OMcp1_28*ORcp1_128+OPcp1_18*RLcp1_228-OPcp1_28*RLcp1_128;
  OMPxF1(1) = OPcp1_18;
  OMPxF1(2) = OPcp1_28;
  OMPxF1(3) = OPcp1_38;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc12 = ROcp1_18*SWr1(1)+ROcp1_28*SWr1(2)+ROcp1_38*SWr1(3);
  xfrc22 = ROcp1_47*SWr1(1)+ROcp1_57*SWr1(2)+ROcp1_67*SWr1(3);
  xfrc32 = ROcp1_78*SWr1(1)+ROcp1_88*SWr1(2)+ROcp1_98*SWr1(3);
  frc(1,8) = s.frc(1,8)+xfrc12;
  frc(2,8) = s.frc(2,8)+xfrc22;
  frc(3,8) = s.frc(3,8)+xfrc32;
  xtrq12 = ROcp1_18*SWr1(4)+ROcp1_28*SWr1(5)+ROcp1_38*SWr1(6);
  xtrq22 = ROcp1_47*SWr1(4)+ROcp1_57*SWr1(5)+ROcp1_67*SWr1(6);
  xtrq32 = ROcp1_78*SWr1(4)+ROcp1_88*SWr1(5)+ROcp1_98*SWr1(6);
  trq(1,8) = s.trq(1,8)+xtrq12-xfrc22*(SWr1(9)-s.l(3,8))+xfrc32*SWr1(8);
  trq(2,8) = s.trq(2,8)+xtrq22+xfrc12*(SWr1(9)-s.l(3,8))-xfrc32*SWr1(7);
  trq(3,8) = s.trq(3,8)+xtrq32-xfrc12*SWr1(8)+xfrc22*SWr1(7);

% = = Block_0_0_1_2_0_1 = = 
 
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

% = = Block_0_0_1_2_0_3 = = 
 
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
  OMcp2_19 = OMcp2_16+qd(9)*ROcp2_46;
  OMcp2_29 = OMcp2_26+qd(9)*ROcp2_56;
  OMcp2_39 = OMcp2_36+qd(9)*ROcp2_66;
  ORcp2_19 = OMcp2_26*RLcp2_39-OMcp2_36*RLcp2_29;
  ORcp2_29 = -(OMcp2_16*RLcp2_39-OMcp2_36*RLcp2_19);
  ORcp2_39 = OMcp2_16*RLcp2_29-OMcp2_26*RLcp2_19;
  OMcp2_110 = OMcp2_19+qd(10)*ROcp2_19;
  OMcp2_210 = OMcp2_29+qd(10)*ROcp2_29;
  OMcp2_310 = OMcp2_39+qd(10)*ROcp2_39;
  OPcp2_110 = OPcp2_16+qd(10)*(OMcp2_29*ROcp2_39-OMcp2_39*ROcp2_29)+qd(9)*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56)+qdd(10)*ROcp2_19+qdd(9)*ROcp2_46;
  OPcp2_210 = OPcp2_26-qd(10)*(OMcp2_19*ROcp2_39-OMcp2_39*ROcp2_19)-qd(9)*(OMcp2_16*ROcp2_66-OMcp2_36*ROcp2_46)+qdd(10)*ROcp2_29+qdd(9)*ROcp2_56;
  OPcp2_310 = OPcp2_36+qd(10)*(OMcp2_19*ROcp2_29-OMcp2_29*ROcp2_19)+qd(9)*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46)+qdd(10)*ROcp2_39+qdd(9)*ROcp2_66;
  RLcp2_129 = ROcp2_710*s.dpt(3,11);
  RLcp2_229 = ROcp2_810*s.dpt(3,11);
  RLcp2_329 = ROcp2_910*s.dpt(3,11);
  ORcp2_129 = OMcp2_210*RLcp2_329-OMcp2_310*RLcp2_229;
  ORcp2_229 = -(OMcp2_110*RLcp2_329-OMcp2_310*RLcp2_129);
  ORcp2_329 = OMcp2_110*RLcp2_229-OMcp2_210*RLcp2_129;
  PxF2(1) = q(1)+RLcp2_129+RLcp2_19;
  PxF2(2) = q(2)+RLcp2_229+RLcp2_29;
  PxF2(3) = q(3)+RLcp2_329+RLcp2_39;
  RxF2(1,1) = ROcp2_19;
  RxF2(1,2) = ROcp2_29;
  RxF2(1,3) = ROcp2_39;
  RxF2(2,1) = ROcp2_410;
  RxF2(2,2) = ROcp2_510;
  RxF2(2,3) = ROcp2_610;
  RxF2(3,1) = ROcp2_710;
  RxF2(3,2) = ROcp2_810;
  RxF2(3,3) = ROcp2_910;
  VxF2(1) = qd(1)+ORcp2_129+ORcp2_19;
  VxF2(2) = qd(2)+ORcp2_229+ORcp2_29;
  VxF2(3) = qd(3)+ORcp2_329+ORcp2_39;
  OMxF2(1) = OMcp2_110;
  OMxF2(2) = OMcp2_210;
  OMxF2(3) = OMcp2_310;
  AxF2(1) = qdd(1)+OMcp2_210*ORcp2_329+OMcp2_26*ORcp2_39-OMcp2_310*ORcp2_229-OMcp2_36*ORcp2_29+OPcp2_210*RLcp2_329+OPcp2_26*RLcp2_39-OPcp2_310*...
 RLcp2_229-OPcp2_36*RLcp2_29;
  AxF2(2) = qdd(2)-OMcp2_110*ORcp2_329-OMcp2_16*ORcp2_39+OMcp2_310*ORcp2_129+OMcp2_36*ORcp2_19-OPcp2_110*RLcp2_329-OPcp2_16*RLcp2_39+OPcp2_310*...
 RLcp2_129+OPcp2_36*RLcp2_19;
  AxF2(3) = qdd(3)+OMcp2_110*ORcp2_229+OMcp2_16*ORcp2_29-OMcp2_210*ORcp2_129-OMcp2_26*ORcp2_19+OPcp2_110*RLcp2_229+OPcp2_16*RLcp2_29-OPcp2_210*...
 RLcp2_129-OPcp2_26*RLcp2_19;
  OMPxF2(1) = OPcp2_110;
  OMPxF2(2) = OPcp2_210;
  OMPxF2(3) = OPcp2_310;
 
% Sensor Forces Computation 

  SWr2 = usrfun.fext(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc13 = ROcp2_19*SWr2(1)+ROcp2_29*SWr2(2)+ROcp2_39*SWr2(3);
  xfrc23 = ROcp2_410*SWr2(1)+ROcp2_510*SWr2(2)+ROcp2_610*SWr2(3);
  xfrc33 = ROcp2_710*SWr2(1)+ROcp2_810*SWr2(2)+ROcp2_910*SWr2(3);
  frc(1,10) = s.frc(1,10)+xfrc13;
  frc(2,10) = s.frc(2,10)+xfrc23;
  frc(3,10) = s.frc(3,10)+xfrc33;
  xtrq13 = ROcp2_19*SWr2(4)+ROcp2_29*SWr2(5)+ROcp2_39*SWr2(6);
  xtrq23 = ROcp2_410*SWr2(4)+ROcp2_510*SWr2(5)+ROcp2_610*SWr2(6);
  xtrq33 = ROcp2_710*SWr2(4)+ROcp2_810*SWr2(5)+ROcp2_910*SWr2(6);
  trq(1,10) = s.trq(1,10)+xtrq13-xfrc23*(SWr2(9)-s.l(3,10))+xfrc33*SWr2(8);
  trq(2,10) = s.trq(2,10)+xtrq23+xfrc13*(SWr2(9)-s.l(3,10))-xfrc33*SWr2(7);
  trq(3,10) = s.trq(3,10)+xtrq33-xfrc13*SWr2(8)+xfrc23*SWr2(7);

% = = Block_0_0_1_3_0_1 = = 
 
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

% = = Block_0_0_1_3_0_4 = = 
 
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
  OMcp3_111 = OMcp3_16+qd(11)*ROcp3_46;
  OMcp3_211 = OMcp3_26+qd(11)*ROcp3_56;
  OMcp3_311 = OMcp3_36+qd(11)*ROcp3_66;
  ORcp3_111 = OMcp3_26*RLcp3_311-OMcp3_36*RLcp3_211;
  ORcp3_211 = -(OMcp3_16*RLcp3_311-OMcp3_36*RLcp3_111);
  ORcp3_311 = OMcp3_16*RLcp3_211-OMcp3_26*RLcp3_111;
  OMcp3_112 = OMcp3_111+qd(12)*ROcp3_111;
  OMcp3_212 = OMcp3_211+qd(12)*ROcp3_211;
  OMcp3_312 = OMcp3_311+qd(12)*ROcp3_311;
  OPcp3_112 = OPcp3_16+qd(11)*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)+qd(12)*(OMcp3_211*ROcp3_311-OMcp3_311*ROcp3_211)+qdd(11)*ROcp3_46+qdd(12)*...
 ROcp3_111;
  OPcp3_212 = OPcp3_26-qd(11)*(OMcp3_16*ROcp3_66-OMcp3_36*ROcp3_46)-qd(12)*(OMcp3_111*ROcp3_311-OMcp3_311*ROcp3_111)+qdd(11)*ROcp3_56+qdd(12)*...
 ROcp3_211;
  OPcp3_312 = OPcp3_36+qd(11)*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)+qd(12)*(OMcp3_111*ROcp3_211-OMcp3_211*ROcp3_111)+qdd(11)*ROcp3_66+qdd(12)*...
 ROcp3_311;
  RLcp3_130 = ROcp3_712*s.dpt(3,12);
  RLcp3_230 = ROcp3_812*s.dpt(3,12);
  RLcp3_330 = ROcp3_912*s.dpt(3,12);
  ORcp3_130 = OMcp3_212*RLcp3_330-OMcp3_312*RLcp3_230;
  ORcp3_230 = -(OMcp3_112*RLcp3_330-OMcp3_312*RLcp3_130);
  ORcp3_330 = OMcp3_112*RLcp3_230-OMcp3_212*RLcp3_130;
  PxF3(1) = q(1)+RLcp3_111+RLcp3_130;
  PxF3(2) = q(2)+RLcp3_211+RLcp3_230;
  PxF3(3) = q(3)+RLcp3_311+RLcp3_330;
  RxF3(1,1) = ROcp3_111;
  RxF3(1,2) = ROcp3_211;
  RxF3(1,3) = ROcp3_311;
  RxF3(2,1) = ROcp3_412;
  RxF3(2,2) = ROcp3_512;
  RxF3(2,3) = ROcp3_612;
  RxF3(3,1) = ROcp3_712;
  RxF3(3,2) = ROcp3_812;
  RxF3(3,3) = ROcp3_912;
  VxF3(1) = qd(1)+ORcp3_111+ORcp3_130;
  VxF3(2) = qd(2)+ORcp3_211+ORcp3_230;
  VxF3(3) = qd(3)+ORcp3_311+ORcp3_330;
  OMxF3(1) = OMcp3_112;
  OMxF3(2) = OMcp3_212;
  OMxF3(3) = OMcp3_312;
  AxF3(1) = qdd(1)+OMcp3_212*ORcp3_330+OMcp3_26*ORcp3_311-OMcp3_312*ORcp3_230-OMcp3_36*ORcp3_211+OPcp3_212*RLcp3_330+OPcp3_26*RLcp3_311-OPcp3_312...
 *RLcp3_230-OPcp3_36*RLcp3_211;
  AxF3(2) = qdd(2)-OMcp3_112*ORcp3_330-OMcp3_16*ORcp3_311+OMcp3_312*ORcp3_130+OMcp3_36*ORcp3_111-OPcp3_112*RLcp3_330-OPcp3_16*RLcp3_311+OPcp3_312...
 *RLcp3_130+OPcp3_36*RLcp3_111;
  AxF3(3) = qdd(3)+OMcp3_112*ORcp3_230+OMcp3_16*ORcp3_211-OMcp3_212*ORcp3_130-OMcp3_26*ORcp3_111+OPcp3_112*RLcp3_230+OPcp3_16*RLcp3_211-OPcp3_212...
 *RLcp3_130-OPcp3_26*RLcp3_111;
  OMPxF3(1) = OPcp3_112;
  OMPxF3(2) = OPcp3_212;
  OMPxF3(3) = OPcp3_312;
 
% Sensor Forces Computation 

  SWr3 = usrfun.fext(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc14 = ROcp3_111*SWr3(1)+ROcp3_211*SWr3(2)+ROcp3_311*SWr3(3);
  xfrc24 = ROcp3_412*SWr3(1)+ROcp3_512*SWr3(2)+ROcp3_612*SWr3(3);
  xfrc34 = ROcp3_712*SWr3(1)+ROcp3_812*SWr3(2)+ROcp3_912*SWr3(3);
  frc(1,12) = s.frc(1,12)+xfrc14;
  frc(2,12) = s.frc(2,12)+xfrc24;
  frc(3,12) = s.frc(3,12)+xfrc34;
  xtrq14 = ROcp3_111*SWr3(4)+ROcp3_211*SWr3(5)+ROcp3_311*SWr3(6);
  xtrq24 = ROcp3_412*SWr3(4)+ROcp3_512*SWr3(5)+ROcp3_612*SWr3(6);
  xtrq34 = ROcp3_712*SWr3(4)+ROcp3_812*SWr3(5)+ROcp3_912*SWr3(6);
  trq(1,12) = s.trq(1,12)+xtrq14-xfrc24*(SWr3(9)-s.l(3,12))+xfrc34*SWr3(8);
  trq(2,12) = s.trq(2,12)+xtrq24+xfrc14*(SWr3(9)-s.l(3,12))-xfrc34*SWr3(7);
  trq(3,12) = s.trq(3,12)+xtrq34-xfrc14*SWr3(8)+xfrc24*SWr3(7);

% = = Block_0_0_1_4_0_1 = = 
 
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

% = = Block_0_0_1_4_0_5 = = 
 
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
  OMcp4_113 = OMcp4_16+qd(13)*ROcp4_46;
  OMcp4_213 = OMcp4_26+qd(13)*ROcp4_56;
  OMcp4_313 = OMcp4_36+qd(13)*ROcp4_66;
  ORcp4_113 = OMcp4_26*RLcp4_313-OMcp4_36*RLcp4_213;
  ORcp4_213 = -(OMcp4_16*RLcp4_313-OMcp4_36*RLcp4_113);
  ORcp4_313 = OMcp4_16*RLcp4_213-OMcp4_26*RLcp4_113;
  OMcp4_114 = OMcp4_113+qd(14)*ROcp4_113;
  OMcp4_214 = OMcp4_213+qd(14)*ROcp4_213;
  OMcp4_314 = OMcp4_313+qd(14)*ROcp4_313;
  OPcp4_114 = OPcp4_16+qd(13)*(OMcp4_26*ROcp4_66-OMcp4_36*ROcp4_56)+qd(14)*(OMcp4_213*ROcp4_313-OMcp4_313*ROcp4_213)+qdd(13)*ROcp4_46+qdd(14)*...
 ROcp4_113;
  OPcp4_214 = OPcp4_26-qd(13)*(OMcp4_16*ROcp4_66-OMcp4_36*ROcp4_46)-qd(14)*(OMcp4_113*ROcp4_313-OMcp4_313*ROcp4_113)+qdd(13)*ROcp4_56+qdd(14)*...
 ROcp4_213;
  OPcp4_314 = OPcp4_36+qd(13)*(OMcp4_16*ROcp4_56-OMcp4_26*ROcp4_46)+qd(14)*(OMcp4_113*ROcp4_213-OMcp4_213*ROcp4_113)+qdd(13)*ROcp4_66+qdd(14)*...
 ROcp4_313;
  RLcp4_131 = ROcp4_714*s.dpt(3,13);
  RLcp4_231 = ROcp4_814*s.dpt(3,13);
  RLcp4_331 = ROcp4_914*s.dpt(3,13);
  ORcp4_131 = OMcp4_214*RLcp4_331-OMcp4_314*RLcp4_231;
  ORcp4_231 = -(OMcp4_114*RLcp4_331-OMcp4_314*RLcp4_131);
  ORcp4_331 = OMcp4_114*RLcp4_231-OMcp4_214*RLcp4_131;
  PxF4(1) = q(1)+RLcp4_113+RLcp4_131;
  PxF4(2) = q(2)+RLcp4_213+RLcp4_231;
  PxF4(3) = q(3)+RLcp4_313+RLcp4_331;
  RxF4(1,1) = ROcp4_113;
  RxF4(1,2) = ROcp4_213;
  RxF4(1,3) = ROcp4_313;
  RxF4(2,1) = ROcp4_414;
  RxF4(2,2) = ROcp4_514;
  RxF4(2,3) = ROcp4_614;
  RxF4(3,1) = ROcp4_714;
  RxF4(3,2) = ROcp4_814;
  RxF4(3,3) = ROcp4_914;
  VxF4(1) = qd(1)+ORcp4_113+ORcp4_131;
  VxF4(2) = qd(2)+ORcp4_213+ORcp4_231;
  VxF4(3) = qd(3)+ORcp4_313+ORcp4_331;
  OMxF4(1) = OMcp4_114;
  OMxF4(2) = OMcp4_214;
  OMxF4(3) = OMcp4_314;
  AxF4(1) = qdd(1)+OMcp4_214*ORcp4_331+OMcp4_26*ORcp4_313-OMcp4_314*ORcp4_231-OMcp4_36*ORcp4_213+OPcp4_214*RLcp4_331+OPcp4_26*RLcp4_313-OPcp4_314...
 *RLcp4_231-OPcp4_36*RLcp4_213;
  AxF4(2) = qdd(2)-OMcp4_114*ORcp4_331-OMcp4_16*ORcp4_313+OMcp4_314*ORcp4_131+OMcp4_36*ORcp4_113-OPcp4_114*RLcp4_331-OPcp4_16*RLcp4_313+OPcp4_314...
 *RLcp4_131+OPcp4_36*RLcp4_113;
  AxF4(3) = qdd(3)+OMcp4_114*ORcp4_231+OMcp4_16*ORcp4_213-OMcp4_214*ORcp4_131-OMcp4_26*ORcp4_113+OPcp4_114*RLcp4_231+OPcp4_16*RLcp4_213-OPcp4_214...
 *RLcp4_131-OPcp4_26*RLcp4_113;
  OMPxF4(1) = OPcp4_114;
  OMPxF4(2) = OPcp4_214;
  OMPxF4(3) = OPcp4_314;
 
% Sensor Forces Computation 

  SWr4 = usrfun.fext(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_113*SWr4(1)+ROcp4_213*SWr4(2)+ROcp4_313*SWr4(3);
  xfrc25 = ROcp4_414*SWr4(1)+ROcp4_514*SWr4(2)+ROcp4_614*SWr4(3);
  xfrc35 = ROcp4_714*SWr4(1)+ROcp4_814*SWr4(2)+ROcp4_914*SWr4(3);
  frc(1,14) = s.frc(1,14)+xfrc15;
  frc(2,14) = s.frc(2,14)+xfrc25;
  frc(3,14) = s.frc(3,14)+xfrc35;
  xtrq15 = ROcp4_113*SWr4(4)+ROcp4_213*SWr4(5)+ROcp4_313*SWr4(6);
  xtrq25 = ROcp4_414*SWr4(4)+ROcp4_514*SWr4(5)+ROcp4_614*SWr4(6);
  xtrq35 = ROcp4_714*SWr4(4)+ROcp4_814*SWr4(5)+ROcp4_914*SWr4(6);
  trq(1,14) = s.trq(1,14)+xtrq15-xfrc25*(SWr4(9)-s.l(3,14))+xfrc35*SWr4(8);
  trq(2,14) = s.trq(2,14)+xtrq25+xfrc15*(SWr4(9)-s.l(3,14))-xfrc35*SWr4(7);
  trq(3,14) = s.trq(3,14)+xtrq35-xfrc15*SWr4(8)+xfrc25*SWr4(7);

% = = Block_0_0_1_5_0_1 = = 
 
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
  OMcp5_26 = OMcp5_25+qd(6)*ROcp5_85;
  OMcp5_36 = OMcp5_35+qd(6)*ROcp5_95;
  OPcp5_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp5_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp5_95-OMcp5_35*S5)-qdd(5)*C4-qdd(6)*ROcp5_85);
  OPcp5_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp5_85-OMcp5_25*S5)+qdd(5)*S4+qdd(6)*ROcp5_95;

% = = Block_0_0_1_5_0_6 = = 
 
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
  OMcp5_115 = OMcp5_16+qd(15)*ROcp5_46;
  OMcp5_215 = OMcp5_26+qd(15)*ROcp5_56;
  OMcp5_315 = OMcp5_36+qd(15)*ROcp5_66;
  ORcp5_115 = OMcp5_26*RLcp5_315-OMcp5_36*RLcp5_215;
  ORcp5_215 = -(OMcp5_16*RLcp5_315-OMcp5_36*RLcp5_115);
  ORcp5_315 = OMcp5_16*RLcp5_215-OMcp5_26*RLcp5_115;
  OMcp5_116 = OMcp5_115+qd(16)*ROcp5_115;
  OMcp5_216 = OMcp5_215+qd(16)*ROcp5_215;
  OMcp5_316 = OMcp5_315+qd(16)*ROcp5_315;
  OPcp5_116 = OPcp5_16+qd(15)*(OMcp5_26*ROcp5_66-OMcp5_36*ROcp5_56)+qd(16)*(OMcp5_215*ROcp5_315-OMcp5_315*ROcp5_215)+qdd(15)*ROcp5_46+qdd(16)*...
 ROcp5_115;
  OPcp5_216 = OPcp5_26-qd(15)*(OMcp5_16*ROcp5_66-OMcp5_36*ROcp5_46)-qd(16)*(OMcp5_115*ROcp5_315-OMcp5_315*ROcp5_115)+qdd(15)*ROcp5_56+qdd(16)*...
 ROcp5_215;
  OPcp5_316 = OPcp5_36+qd(15)*(OMcp5_16*ROcp5_56-OMcp5_26*ROcp5_46)+qd(16)*(OMcp5_115*ROcp5_215-OMcp5_215*ROcp5_115)+qdd(15)*ROcp5_66+qdd(16)*...
 ROcp5_315;
  RLcp5_132 = ROcp5_716*s.dpt(3,14);
  RLcp5_232 = ROcp5_816*s.dpt(3,14);
  RLcp5_332 = ROcp5_916*s.dpt(3,14);
  ORcp5_132 = OMcp5_216*RLcp5_332-OMcp5_316*RLcp5_232;
  ORcp5_232 = -(OMcp5_116*RLcp5_332-OMcp5_316*RLcp5_132);
  ORcp5_332 = OMcp5_116*RLcp5_232-OMcp5_216*RLcp5_132;
  PxF5(1) = q(1)+RLcp5_115+RLcp5_132;
  PxF5(2) = q(2)+RLcp5_215+RLcp5_232;
  PxF5(3) = q(3)+RLcp5_315+RLcp5_332;
  RxF5(1,1) = ROcp5_115;
  RxF5(1,2) = ROcp5_215;
  RxF5(1,3) = ROcp5_315;
  RxF5(2,1) = ROcp5_416;
  RxF5(2,2) = ROcp5_516;
  RxF5(2,3) = ROcp5_616;
  RxF5(3,1) = ROcp5_716;
  RxF5(3,2) = ROcp5_816;
  RxF5(3,3) = ROcp5_916;
  VxF5(1) = qd(1)+ORcp5_115+ORcp5_132;
  VxF5(2) = qd(2)+ORcp5_215+ORcp5_232;
  VxF5(3) = qd(3)+ORcp5_315+ORcp5_332;
  OMxF5(1) = OMcp5_116;
  OMxF5(2) = OMcp5_216;
  OMxF5(3) = OMcp5_316;
  AxF5(1) = qdd(1)+OMcp5_216*ORcp5_332+OMcp5_26*ORcp5_315-OMcp5_316*ORcp5_232-OMcp5_36*ORcp5_215+OPcp5_216*RLcp5_332+OPcp5_26*RLcp5_315-OPcp5_316...
 *RLcp5_232-OPcp5_36*RLcp5_215;
  AxF5(2) = qdd(2)-OMcp5_116*ORcp5_332-OMcp5_16*ORcp5_315+OMcp5_316*ORcp5_132+OMcp5_36*ORcp5_115-OPcp5_116*RLcp5_332-OPcp5_16*RLcp5_315+OPcp5_316...
 *RLcp5_132+OPcp5_36*RLcp5_115;
  AxF5(3) = qdd(3)+OMcp5_116*ORcp5_232+OMcp5_16*ORcp5_215-OMcp5_216*ORcp5_132-OMcp5_26*ORcp5_115+OPcp5_116*RLcp5_232+OPcp5_16*RLcp5_215-OPcp5_216...
 *RLcp5_132-OPcp5_26*RLcp5_115;
  OMPxF5(1) = OPcp5_116;
  OMPxF5(2) = OPcp5_216;
  OMPxF5(3) = OPcp5_316;
 
% Sensor Forces Computation 

  SWr5 = usrfun.fext(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc16 = ROcp5_115*SWr5(1)+ROcp5_215*SWr5(2)+ROcp5_315*SWr5(3);
  xfrc26 = ROcp5_416*SWr5(1)+ROcp5_516*SWr5(2)+ROcp5_616*SWr5(3);
  xfrc36 = ROcp5_716*SWr5(1)+ROcp5_816*SWr5(2)+ROcp5_916*SWr5(3);
  frc(1,16) = s.frc(1,16)+xfrc16;
  frc(2,16) = s.frc(2,16)+xfrc26;
  frc(3,16) = s.frc(3,16)+xfrc36;
  xtrq16 = ROcp5_115*SWr5(4)+ROcp5_215*SWr5(5)+ROcp5_315*SWr5(6);
  xtrq26 = ROcp5_416*SWr5(4)+ROcp5_516*SWr5(5)+ROcp5_616*SWr5(6);
  xtrq36 = ROcp5_716*SWr5(4)+ROcp5_816*SWr5(5)+ROcp5_916*SWr5(6);
  trq(1,16) = s.trq(1,16)+xtrq16-xfrc26*(SWr5(9)-s.l(3,16))+xfrc36*SWr5(8);
  trq(2,16) = s.trq(2,16)+xtrq26+xfrc16*(SWr5(9)-s.l(3,16))-xfrc36*SWr5(7);
  trq(3,16) = s.trq(3,16)+xtrq36-xfrc16*SWr5(8)+xfrc26*SWr5(7);

% = = Block_0_0_1_5_1_0 = = 
 
% Symbolic Outputs  

  frc(1,6) = s.frc(1,6);
  frc(2,6) = s.frc(2,6);
  frc(3,6) = s.frc(3,6);
  frc(1,18) = s.frc(1,18);
  frc(2,18) = s.frc(2,18);
  frc(3,18) = s.frc(3,18);
  frc(1,19) = s.frc(1,19);
  frc(2,19) = s.frc(2,19);
  frc(3,19) = s.frc(3,19);
  frc(1,21) = s.frc(1,21);
  frc(2,21) = s.frc(2,21);
  frc(3,21) = s.frc(3,21);
  frc(1,22) = s.frc(1,22);
  frc(2,22) = s.frc(2,22);
  frc(3,22) = s.frc(3,22);
  trq(1,6) = s.trq(1,6);
  trq(2,6) = s.trq(2,6);
  trq(3,6) = s.trq(3,6);
  trq(1,18) = s.trq(1,18);
  trq(2,18) = s.trq(2,18);
  trq(3,18) = s.trq(3,18);
  trq(1,19) = s.trq(1,19);
  trq(2,19) = s.trq(2,19);
  trq(3,19) = s.trq(3,19);
  trq(1,21) = s.trq(1,21);
  trq(2,21) = s.trq(2,21);
  trq(3,21) = s.trq(3,21);
  trq(1,22) = s.trq(1,22);
  trq(2,22) = s.trq(2,22);
  trq(3,22) = s.trq(3,22);

% ====== END Task 0 ====== 

  

