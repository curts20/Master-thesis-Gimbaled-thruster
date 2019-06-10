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
%	==> Function : F 2 : Inverse Dynamics : RNEA
%	==> Flops complexity : 1356
%
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [Qq] = invdyna(s,tsim,usrfun)

 Qq = zeros(22,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

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

% = = Block_0_1_0_0_0_0 = = 
 
% Forward Kinematics 

  ALPHA33 = qdd(3)-s.g(3);
  ALPHA24 = qdd(2)*C4+ALPHA33*S4;
  ALPHA34 = -(qdd(2)*S4-ALPHA33*C4);
  OM15 = qd(4)*C5;
  OMp15 = -(qd(4)*qd(5)*S5-qdd(4)*C5);
  ALPHA15 = qdd(1)*C5-ALPHA34*S5;
  ALPHA35 = qdd(1)*S5+ALPHA34*C5;
  OM16 = qd(5)*S6+OM15*C6;
  OM26 = qd(5)*C6-OM15*S6;
  OM36 = qd(6)+qd(4)*S5;
  OMp16 = C6*(OMp15+qd(5)*qd(6))+S6*(qdd(5)-qd(6)*OM15);
  OMp26 = C6*(qdd(5)-qd(6)*OM15)-S6*(OMp15+qd(5)*qd(6));
  OMp36 = qdd(6)+qd(4)*qd(5)*C5+qdd(4)*S5;
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BETA26 = BS26-OMp36;
  BETA46 = BS26+OMp36;
  BETA76 = BS36-OMp26;
  BETA86 = BS66+OMp16;
  ALPHA16 = ALPHA15*C6+ALPHA24*S6;
  ALPHA26 = -(ALPHA15*S6-ALPHA24*C6);
  OM17 = qd(7)+OM16;
  OM37 = -(OM26*S7-OM36*C7);
  OMp17 = qdd(7)+OMp16;
  OMp37 = C7*(OMp36-qd(7)*OM26)-S7*(OMp26+qd(7)*OM36);
  ALPHA37 = -(ALPHA26*S7-ALPHA35*C7);
  OM18 = OM17*C8-OM37*S8;
  OM28 = qd(8)+OM26*C7+OM36*S7;
  OM38 = OM17*S8+OM37*C8;
  OMp18 = C8*(OMp17-qd(8)*OM37)-S8*(OMp37+qd(8)*OM17);
  OMp28 = qdd(8)+C7*(OMp26+qd(7)*OM36)+S7*(OMp36-qd(7)*OM26);
  OM29 = qd(9)+OM26;
  OM39 = OM16*S9+OM36*C9;
  OMp29 = qdd(9)+OMp26;
  OMp39 = C9*(OMp36+qd(9)*OM16)+S9*(OMp16-qd(9)*OM36);
  ALPHA29 = ALPHA26+BETA46*s.dpt(1,1);
  ALPHA39 = C9*(ALPHA35+BETA76*s.dpt(1,1))+S9*(ALPHA16+BS16*s.dpt(1,1));
  OM110 = qd(10)+OM16*C9-OM36*S9;
  OM210 = OM29*C10+OM39*S10;
  OM310 = -(OM29*S10-OM39*C10);
  OMp110 = qdd(10)+C9*(OMp16-qd(9)*OM36)-S9*(OMp36+qd(9)*OM16);
  OMp210 = C10*(OMp29+qd(10)*OM39)+S10*(OMp39-qd(10)*OM29);
  OM211 = qd(11)+OM26;
  OM311 = OM16*S11+OM36*C11;
  OMp211 = qdd(11)+OMp26;
  OMp311 = C11*(OMp36+qd(11)*OM16)+S11*(OMp16-qd(11)*OM36);
  ALPHA211 = ALPHA26+BS56*s.dpt(2,2);
  ALPHA311 = C11*(ALPHA35+BETA86*s.dpt(2,2))+S11*(ALPHA16+BETA26*s.dpt(2,2));
  OM112 = qd(12)+OM16*C11-OM36*S11;
  OM212 = OM211*C12+OM311*S12;
  OM312 = -(OM211*S12-OM311*C12);
  OMp112 = qdd(12)+C11*(OMp16-qd(11)*OM36)-S11*(OMp36+qd(11)*OM16);
  OMp212 = C12*(OMp211+qd(12)*OM311)+S12*(OMp311-qd(12)*OM211);
  OM213 = qd(13)+OM26;
  OM313 = OM16*S13+OM36*C13;
  OMp213 = qdd(13)+OMp26;
  OMp313 = C13*(OMp36+qd(13)*OM16)+S13*(OMp16-qd(13)*OM36);
  ALPHA213 = ALPHA26+BETA46*s.dpt(1,3);
  ALPHA313 = C13*(ALPHA35+BETA76*s.dpt(1,3))+S13*(ALPHA16+BS16*s.dpt(1,3));
  OM114 = qd(14)+OM16*C13-OM36*S13;
  OM214 = OM213*C14+OM313*S14;
  OM314 = -(OM213*S14-OM313*C14);
  OMp114 = qdd(14)+C13*(OMp16-qd(13)*OM36)-S13*(OMp36+qd(13)*OM16);
  OMp214 = C14*(OMp213+qd(14)*OM313)+S14*(OMp313-qd(14)*OM213);
  OM215 = qd(15)+OM26;
  OM315 = OM16*S15+OM36*C15;
  OMp215 = qdd(15)+OMp26;
  OMp315 = C15*(OMp36+qd(15)*OM16)+S15*(OMp16-qd(15)*OM36);
  ALPHA215 = ALPHA26+BS56*s.dpt(2,4);
  ALPHA315 = C15*(ALPHA35+BETA86*s.dpt(2,4))+S15*(ALPHA16+BETA26*s.dpt(2,4));
  OM116 = qd(16)+OM16*C15-OM36*S15;
  OM216 = OM215*C16+OM315*S16;
  OM316 = -(OM215*S16-OM315*C16);
  OMp116 = qdd(16)+C15*(OMp16-qd(15)*OM36)-S15*(OMp36+qd(15)*OM16);
  OMp216 = C16*(OMp215+qd(16)*OM315)+S16*(OMp315-qd(16)*OM215);
  OM117 = qd(17)+OM16;
  OM317 = -(OM26*S17-OM36*C17);
  OMp117 = qdd(17)+OMp16;
  OMp317 = C17*(OMp36-qd(17)*OM26)-S17*(OMp26+qd(17)*OM36);
  ALPHA117 = ALPHA16+BS16*s.dpt(1,5);
  ALPHA217 = C17*(ALPHA26+BETA46*s.dpt(1,5))+S17*(ALPHA35+BETA76*s.dpt(1,5));
  ALPHA317 = C17*(ALPHA35+BETA76*s.dpt(1,5))-S17*(ALPHA26+BETA46*s.dpt(1,5));
  OM118 = OM117*C18-OM317*S18;
  OM218 = qd(18)+OM26*C17+OM36*S17;
  OM318 = OM117*S18+OM317*C18;
  OMp118 = C18*(OMp117-qd(18)*OM317)-S18*(OMp317+qd(18)*OM117);
  OMp218 = qdd(18)+C17*(OMp26+qd(17)*OM36)+S17*(OMp36-qd(17)*OM26);
  OMp318 = C18*(OMp317+qd(18)*OM117)+S18*(OMp117-qd(18)*OM317);
  ALPHA118 = ALPHA117*C18-ALPHA317*S18;
  ALPHA318 = ALPHA117*S18+ALPHA317*C18;
  OM120 = qd(20)+OM16;
  OM320 = -(OM26*S20-OM36*C20);
  OMp120 = qdd(20)+OMp16;
  OMp320 = C20*(OMp36-qd(20)*OM26)-S20*(OMp26+qd(20)*OM36);
  ALPHA120 = ALPHA16+BETA26*s.dpt(2,6);
  ALPHA220 = C20*(ALPHA26+BS56*s.dpt(2,6))+S20*(ALPHA35+BETA86*s.dpt(2,6));
  ALPHA320 = C20*(ALPHA35+BETA86*s.dpt(2,6))-S20*(ALPHA26+BS56*s.dpt(2,6));
  OM121 = OM120*C21-OM320*S21;
  OM221 = qd(21)+OM26*C20+OM36*S20;
  OM321 = OM120*S21+OM320*C21;
  OMp121 = C21*(OMp120-qd(21)*OM320)-S21*(OMp320+qd(21)*OM120);
  OMp221 = qdd(21)+C20*(OMp26+qd(20)*OM36)+S20*(OMp36-qd(20)*OM26);
  OMp321 = C21*(OMp320+qd(21)*OM120)+S21*(OMp120-qd(21)*OM320);
  ALPHA121 = ALPHA120*C21-ALPHA320*S21;
  ALPHA321 = ALPHA120*S21+ALPHA320*C21;
 
% Backward Dynamics 

  Fs122 = -(s.frc(1,22)-s.m(22)*(ALPHA121+(2.0)*qd(22)*OM221+Dz223*(OMp221+OM121*OM321)));
  Fs222 = -(s.frc(2,22)-s.m(22)*(ALPHA220-(2.0)*qd(22)*OM121-Dz223*(OMp121-OM221*OM321)));
  Fs322 = -(s.frc(3,22)-s.m(22)*(qdd(22)+ALPHA321-Dz223*(OM121*OM121+OM221*OM221)));
  Fq121 = -(s.frc(1,21)-Fs122-s.m(21)*ALPHA121);
  Fq221 = -(s.frc(2,21)-Fs222-s.m(21)*ALPHA220);
  Fq321 = -(s.frc(3,21)-Fs322-s.m(21)*ALPHA321);
  Cq121 = -(s.trq(1,21)+s.trq(1,22)-s.In(1,21)*OMp121-s.In(1,22)*OMp121+Dz223*Fs222+OM221*OM321*(s.In(5,21)-s.In(9,21))+OM221*OM321*(s.In(5,22)-...
 s.In(9,22)));
  Cq221 = -(s.trq(2,21)+s.trq(2,22)-s.In(5,21)*OMp221-s.In(5,22)*OMp221-Dz223*Fs122-OM121*OM321*(s.In(1,21)-s.In(9,21))-OM121*OM321*(s.In(1,22)-...
 s.In(9,22)));
  Cq321 = -(s.trq(3,21)+s.trq(3,22)-s.In(9,21)*OMp321-s.In(9,22)*OMp321+OM121*OM221*(s.In(1,21)-s.In(5,21))+OM121*OM221*(s.In(1,22)-s.In(5,22)));
  Fq120 = Fq121*C21+Fq321*S21;
  Fq320 = -(Fq121*S21-Fq321*C21);
  Cq120 = Cq121*C21+Cq321*S21;
  Cq320 = -(Cq121*S21-Cq321*C21);
  Fs119 = -(s.frc(1,19)-s.m(19)*(ALPHA118+(2.0)*qd(19)*OM218+Dz193*(OMp218+OM118*OM318)));
  Fs219 = -(s.frc(2,19)-s.m(19)*(ALPHA217-(2.0)*qd(19)*OM118-Dz193*(OMp118-OM218*OM318)));
  Fs319 = -(s.frc(3,19)-s.m(19)*(qdd(19)+ALPHA318-Dz193*(OM118*OM118+OM218*OM218)));
  Fq118 = -(s.frc(1,18)-Fs119-s.m(18)*ALPHA118);
  Fq218 = -(s.frc(2,18)-Fs219-s.m(18)*ALPHA217);
  Fq318 = -(s.frc(3,18)-Fs319-s.m(18)*ALPHA318);
  Cq118 = -(s.trq(1,18)+s.trq(1,19)-s.In(1,18)*OMp118-s.In(1,19)*OMp118+Dz193*Fs219+OM218*OM318*(s.In(5,18)-s.In(9,18))+OM218*OM318*(s.In(5,19)-...
 s.In(9,19)));
  Cq218 = -(s.trq(2,18)+s.trq(2,19)-s.In(5,18)*OMp218-s.In(5,19)*OMp218-Dz193*Fs119-OM118*OM318*(s.In(1,18)-s.In(9,18))-OM118*OM318*(s.In(1,19)-...
 s.In(9,19)));
  Cq318 = -(s.trq(3,18)+s.trq(3,19)-s.In(9,18)*OMp318-s.In(9,19)*OMp318+OM118*OM218*(s.In(1,18)-s.In(5,18))+OM118*OM218*(s.In(1,19)-s.In(5,19)));
  Fq317 = -(Fq118*S18-Fq318*C18);
  Cq117 = Cq118*C18+Cq318*S18;
  Cq317 = -(Cq118*S18-Cq318*C18);
  Fs116 = -(s.frc(1,16)-s.m(16)*(s.l(3,16)*(OMp216+OM116*OM316)+C15*(ALPHA16+BETA26*s.dpt(2,4))-S15*(ALPHA35+BETA86*s.dpt(2,4))));
  Fs216 = -(s.frc(2,16)-s.m(16)*(ALPHA215*C16+ALPHA315*S16-s.l(3,16)*(OMp116-OM216*OM316)));
  Fs316 = -(s.frc(3,16)+s.m(16)*(ALPHA215*S16-ALPHA315*C16+s.l(3,16)*(OM116*OM116+OM216*OM216)));
  Cq116 = -(s.trq(1,16)-s.In(1,16)*OMp116+Fs216*s.l(3,16)+OM216*OM316*(s.In(5,16)-s.In(9,16)));
  Cq216 = -(s.trq(2,16)-s.In(5,16)*OMp216-Fs116*s.l(3,16)-OM116*OM316*(s.In(1,16)-s.In(9,16)));
  Cq316 = -(s.trq(3,16)-s.In(9,16)*(C16*(OMp315-qd(16)*OM215)-S16*(OMp215+qd(16)*OM315))+OM116*OM216*(s.In(1,16)-s.In(5,16)));
  Fq315 = Fs216*S16+Fs316*C16;
  Cq215 = Cq216*C16-Cq316*S16;
  Cq315 = Cq216*S16+Cq316*C16;
  Fs114 = -(s.frc(1,14)-s.m(14)*(s.l(3,14)*(OMp214+OM114*OM314)+C13*(ALPHA16+BS16*s.dpt(1,3))-S13*(ALPHA35+BETA76*s.dpt(1,3))));
  Fs214 = -(s.frc(2,14)-s.m(14)*(ALPHA213*C14+ALPHA313*S14-s.l(3,14)*(OMp114-OM214*OM314)));
  Fs314 = -(s.frc(3,14)+s.m(14)*(ALPHA213*S14-ALPHA313*C14+s.l(3,14)*(OM114*OM114+OM214*OM214)));
  Cq114 = -(s.trq(1,14)-s.In(1,14)*OMp114+Fs214*s.l(3,14)+OM214*OM314*(s.In(5,14)-s.In(9,14)));
  Cq214 = -(s.trq(2,14)-s.In(5,14)*OMp214-Fs114*s.l(3,14)-OM114*OM314*(s.In(1,14)-s.In(9,14)));
  Cq314 = -(s.trq(3,14)-s.In(9,14)*(C14*(OMp313-qd(14)*OM213)-S14*(OMp213+qd(14)*OM313))+OM114*OM214*(s.In(1,14)-s.In(5,14)));
  Fq213 = Fs214*C14-Fs314*S14;
  Fq313 = Fs214*S14+Fs314*C14;
  Cq213 = Cq214*C14-Cq314*S14;
  Cq313 = Cq214*S14+Cq314*C14;
  Fs112 = -(s.frc(1,12)-s.m(12)*(s.l(3,12)*(OMp212+OM112*OM312)+C11*(ALPHA16+BETA26*s.dpt(2,2))-S11*(ALPHA35+BETA86*s.dpt(2,2))));
  Fs212 = -(s.frc(2,12)-s.m(12)*(ALPHA211*C12+ALPHA311*S12-s.l(3,12)*(OMp112-OM212*OM312)));
  Fs312 = -(s.frc(3,12)+s.m(12)*(ALPHA211*S12-ALPHA311*C12+s.l(3,12)*(OM112*OM112+OM212*OM212)));
  Cq112 = -(s.trq(1,12)-s.In(1,12)*OMp112+Fs212*s.l(3,12)+OM212*OM312*(s.In(5,12)-s.In(9,12)));
  Cq212 = -(s.trq(2,12)-s.In(5,12)*OMp212-Fs112*s.l(3,12)-OM112*OM312*(s.In(1,12)-s.In(9,12)));
  Cq312 = -(s.trq(3,12)-s.In(9,12)*(C12*(OMp311-qd(12)*OM211)-S12*(OMp211+qd(12)*OM311))+OM112*OM212*(s.In(1,12)-s.In(5,12)));
  Fq311 = Fs212*S12+Fs312*C12;
  Cq211 = Cq212*C12-Cq312*S12;
  Cq311 = Cq212*S12+Cq312*C12;
  Fs110 = -(s.frc(1,10)-s.m(10)*(s.l(3,10)*(OMp210+OM110*OM310)+C9*(ALPHA16+BS16*s.dpt(1,1))-S9*(ALPHA35+BETA76*s.dpt(1,1))));
  Fs210 = -(s.frc(2,10)-s.m(10)*(ALPHA29*C10+ALPHA39*S10-s.l(3,10)*(OMp110-OM210*OM310)));
  Fs310 = -(s.frc(3,10)+s.m(10)*(ALPHA29*S10-ALPHA39*C10+s.l(3,10)*(OM110*OM110+OM210*OM210)));
  Cq110 = -(s.trq(1,10)-s.In(1,10)*OMp110+Fs210*s.l(3,10)+OM210*OM310*(s.In(5,10)-s.In(9,10)));
  Cq210 = -(s.trq(2,10)-s.In(5,10)*OMp210-Fs110*s.l(3,10)-OM110*OM310*(s.In(1,10)-s.In(9,10)));
  Cq310 = -(s.trq(3,10)-s.In(9,10)*(C10*(OMp39-qd(10)*OM29)-S10*(OMp29+qd(10)*OM39))+OM110*OM210*(s.In(1,10)-s.In(5,10)));
  Fq29 = Fs210*C10-Fs310*S10;
  Fq39 = Fs210*S10+Fs310*C10;
  Cq29 = Cq210*C10-Cq310*S10;
  Cq39 = Cq210*S10+Cq310*C10;
  Fs18 = -(s.frc(1,8)-s.m(8)*(ALPHA16*C8-ALPHA37*S8+s.l(3,8)*(OMp28+OM18*OM38)));
  Fs28 = -(s.frc(2,8)-s.m(8)*(ALPHA26*C7+ALPHA35*S7-s.l(3,8)*(OMp18-OM28*OM38)));
  Fs38 = -(s.frc(3,8)-s.m(8)*(ALPHA16*S8+ALPHA37*C8-s.l(3,8)*(OM18*OM18+OM28*OM28)));
  Cq18 = -(s.trq(1,8)-s.In(1,8)*OMp18+Fs28*s.l(3,8)+OM28*OM38*(s.In(5,8)-s.In(9,8)));
  Cq28 = -(s.trq(2,8)-s.In(5,8)*OMp28-Fs18*s.l(3,8)-OM18*OM38*(s.In(1,8)-s.In(9,8)));
  Cq38 = -(s.trq(3,8)-s.In(9,8)*(C8*(OMp37+qd(8)*OM17)+S8*(OMp17-qd(8)*OM37))+OM18*OM28*(s.In(1,8)-s.In(5,8)));
  Fq37 = -(Fs18*S8-Fs38*C8);
  Cq17 = Cq18*C8+Cq38*S8;
  Cq37 = -(Cq18*S8-Cq38*C8);
  Fs16 = -(s.frc(1,6)-s.m(6)*(ALPHA16+s.l(3,6)*(BS36+OMp26)));
  Fs26 = -(s.frc(2,6)-s.m(6)*(ALPHA26+s.l(3,6)*(BS66-OMp16)));
  Fq16 = Fq120+Fs16+Fq118*C18+Fq311*S11+Fq313*S13+Fq315*S15+Fq318*S18+Fq39*S9+Fs110*C9+Fs112*C11+Fs114*C13+Fs116*C15+Fs18*C8+Fs38*S8;
  Fq26 = Fq213+Fq29+Fs26+Fq218*C17+Fq221*C20-Fq317*S17-Fq320*S20-Fq37*S7+Fs212*C12+Fs216*C16+Fs28*C7-Fs312*S12-Fs316*S16;
  Fq36 = -(s.frc(3,6)-s.m(6)*(ALPHA35-s.l(3,6)*(OM16*OM16+OM26*OM26))-Fq218*S17-Fq221*S20-Fq311*C11-Fq313*C13-Fq315*C15-Fq317*C17-Fq320*C20-Fq37*...
 C7-Fq39*C9+Fs110*S9+Fs112*S11+Fs114*S13+Fs116*S15-Fs28*S7);
  Cq16 = -(s.trq(1,6)-Cq117-Cq120-Cq17-s.In(1,6)*OMp16-s.In(2,6)*OMp26-s.In(3,6)*OMp36-Cq110*C9-Cq112*C11-Cq114*C13-Cq116*C15-Cq311*S11-Cq313*S13...
 -Cq315*S15-Cq39*S9+Fs26*s.l(3,6)-OM26*(s.In(3,6)*OM16+s.In(6,6)*OM26+s.In(9,6)*OM36)+OM36*(s.In(2,6)*OM16+s.In(5,6)*OM26+s.In(6,6)*OM36)-s.dpt(2,2)*(...
 Fq311*C11-Fs112*S11)-s.dpt(2,4)*(Fq315*C15-Fs116*S15)-s.dpt(2,6)*(Fq221*S20+Fq320*C20));
  Cq26 = -(s.trq(2,6)-Cq211-Cq213-Cq215-Cq29-s.In(2,6)*OMp16-s.In(5,6)*OMp26-s.In(6,6)*OMp36-Cq218*C17-Cq221*C20-Cq28*C7+Cq317*S17+Cq320*S20+Cq37...
 *S7-Fs16*s.l(3,6)+OM16*(s.In(3,6)*OM16+s.In(6,6)*OM26+s.In(9,6)*OM36)-OM36*(s.In(1,6)*OM16+s.In(2,6)*OM26+s.In(3,6)*OM36)+s.dpt(1,1)*(Fq39*C9-Fs110*...
 S9)+s.dpt(1,3)*(Fq313*C13-Fs114*S13)+s.dpt(1,5)*(Fq218*S17+Fq317*C17));
  Cq36 = -(s.trq(3,6)-s.In(3,6)*OMp16-s.In(6,6)*OMp26-s.In(9,6)*OMp36+Cq110*S9+Cq112*S11+Cq114*S13+Cq116*S15-Cq218*S17-Cq221*S20-Cq28*S7-Cq311*...
 C11-Cq313*C13-Cq315*C15-Cq317*C17-Cq320*C20-Cq37*C7-Cq39*C9+Fq120*s.dpt(2,6)-Fq213*s.dpt(1,3)-Fq29*s.dpt(1,1)-OM16*(s.In(2,6)*OM16+s.In(5,6)*OM26+...
 s.In(6,6)*OM36)+OM26*(s.In(1,6)*OM16+s.In(2,6)*OM26+s.In(3,6)*OM36)-s.dpt(1,5)*(Fq218*C17-Fq317*S17)+s.dpt(2,2)*(Fq311*S11+Fs112*C11)+s.dpt(2,4)*(...
 Fq315*S15+Fs116*C15));
  Fq15 = Fq16*C6-Fq26*S6;
  Fq25 = Fq16*S6+Fq26*C6;
  Cq25 = Cq16*S6+Cq26*C6;
  Fq14 = Fq15*C5+Fq36*S5;
  Fq34 = -(Fq15*S5-Fq36*C5);
  Cq14 = Cq36*S5+C5*(Cq16*C6-Cq26*S6);
  Fq23 = Fq25*C4-Fq34*S4;
  Fq33 = Fq25*S4+Fq34*C4;

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  Qq(1) = Fq14;
  Qq(2) = Fq23;
  Qq(3) = Fq33;
  Qq(4) = Cq14;
  Qq(5) = Cq25;
  Qq(6) = Cq36;
  Qq(7) = Cq17;
  Qq(8) = Cq28;
  Qq(9) = Cq29;
  Qq(10) = Cq110;
  Qq(11) = Cq211;
  Qq(12) = Cq112;
  Qq(13) = Cq213;
  Qq(14) = Cq114;
  Qq(15) = Cq215;
  Qq(16) = Cq116;
  Qq(17) = Cq117;
  Qq(18) = Cq218;
  Qq(19) = Fs319;
  Qq(20) = Cq120;
  Qq(21) = Cq221;
  Qq(22) = Fs322;

% ====== END Task 0 ====== 

  

