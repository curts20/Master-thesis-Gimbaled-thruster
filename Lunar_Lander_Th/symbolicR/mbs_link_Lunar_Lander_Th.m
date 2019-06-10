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
%	==> Generation Date : Sun Nov 11 19:32:15 2018
%
%	==> Project name : Lunar_Lander_Th
%	==> using XML input file 
%
%	==> Number of joints : 16
%
%	==> Function : F 7 : Point to point Link Forces (frc,trq,Flnk) 
%	==> Flops complexity : 769
%
%	==> Generation Time :  0.010 seconds
%	==> Post-Processing :  0.020 seconds
%
%-------------------------------------------------------------
%
function [frc,trq,Flnk,Z,Zd] = link(s,tsim,usrfun)

 frc = zeros(3,16);
 trq = zeros(3,16);
 Flnk = zeros(10,1);
 Z = zeros(10,1);
 Zd = zeros(10,1);

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

% = = Block_0_1_0_0_1_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk1_710 = C10*S9;
  ROlnk1_910 = C10*C9;
  RLlnk1_118 = ROlnk1_710*s.dpt(3,19);
  RLlnk1_218 = -s.dpt(3,19)*S10;
  RLlnk1_318 = ROlnk1_910*s.dpt(3,19);

% = = Block_0_1_0_0_2_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk2_710 = C10*S9;
  ROlnk2_910 = C10*C9;
  RLlnk2_119 = ROlnk2_710*s.dpt(3,19);
  RLlnk2_219 = -s.dpt(3,19)*S10;
  RLlnk2_319 = ROlnk2_910*s.dpt(3,19);

% = = Block_0_1_0_0_3_4 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk4_712 = S11*C12;
  ROlnk4_912 = C11*C12;
  RLlnk4_121 = ROlnk4_712*s.dpt(3,21);
  RLlnk4_221 = -s.dpt(3,21)*S12;
  RLlnk4_321 = ROlnk4_912*s.dpt(3,21);

% = = Block_0_1_0_0_4_4 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk6_712 = S11*C12;
  ROlnk6_912 = C11*C12;
  RLlnk6_123 = ROlnk6_712*s.dpt(3,21);
  RLlnk6_223 = -s.dpt(3,21)*S12;
  RLlnk6_323 = ROlnk6_912*s.dpt(3,21);

% = = Block_0_1_0_0_5_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk9_714 = S13*C14;
  ROlnk9_914 = C13*C14;
  RLlnk9_126 = ROlnk9_714*s.dpt(3,23);
  RLlnk9_226 = -s.dpt(3,23)*S14;
  RLlnk9_326 = ROlnk9_914*s.dpt(3,23);

% = = Block_0_1_0_0_6_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk11_714 = S13*C14;
  ROlnk11_914 = C13*C14;
  RLlnk11_128 = ROlnk11_714*s.dpt(3,23);
  RLlnk11_228 = -s.dpt(3,23)*S14;
  RLlnk11_328 = ROlnk11_914*s.dpt(3,23);

% = = Block_0_1_0_0_7_6 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk12_716 = S15*C16;
  ROlnk12_916 = C15*C16;
  RLlnk12_129 = ROlnk12_716*s.dpt(3,25);
  RLlnk12_229 = -s.dpt(3,25)*S16;
  RLlnk12_329 = ROlnk12_916*s.dpt(3,25);

% = = Block_0_1_0_0_8_6 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk14_716 = S15*C16;
  ROlnk14_916 = C15*C16;
  RLlnk14_131 = ROlnk14_716*s.dpt(3,25);
  RLlnk14_231 = -s.dpt(3,25)*S16;
  RLlnk14_331 = ROlnk14_916*s.dpt(3,25);

% = = Block_0_1_0_0_9_2 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk17_28 = S7*S8;
  ROlnk17_38 = -C7*S8;
  ROlnk17_88 = -S7*C8;
  ROlnk17_98 = C7*C8;
  OMlnk17_28 = qd(8)*C7;
  OMlnk17_38 = qd(8)*S7;
  RLlnk17_134 = s.dpt(1,16)*C8+s.dpt(3,16)*S8;
  RLlnk17_234 = ROlnk17_28*s.dpt(1,16)+ROlnk17_88*s.dpt(3,16);
  RLlnk17_334 = ROlnk17_38*s.dpt(1,16)+ROlnk17_98*s.dpt(3,16);

% = = Block_0_1_0_0_10_2 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk19_88 = -S7*C8;
  ROlnk19_98 = C7*C8;
  OMlnk19_28 = qd(8)*C7;
  OMlnk19_38 = qd(8)*S7;
  RLlnk19_136 = s.dpt(3,15)*S8;
  RLlnk19_236 = ROlnk19_88*s.dpt(3,15)+s.dpt(2,15)*C7;
  RLlnk19_336 = ROlnk19_98*s.dpt(3,15)+s.dpt(2,15)*S7;

% = = Block_0_1_0_1_1_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk11 = RLlnk1_118+s.dpt(1,1)-s.dpt(1,5);
  Plnk21 = RLlnk1_218-s.dpt(2,5);
  Z1 = sqrt(Plnk11*Plnk11+Plnk21*Plnk21+RLlnk1_318*RLlnk1_318);
  e11 = Plnk11/Z1;
  e21 = Plnk21/Z1;
  e31 = RLlnk1_318/Z1;
  Zd1 = -(qd(10)*e21*s.dpt(3,19)*C10-e11*(qd(10)*RLlnk1_218*S9+qd(9)*RLlnk1_318)-e31*(qd(10)*RLlnk1_218*C9-qd(9)*RLlnk1_118));
 
% Link Force Computation 

  Flink1 = usrfun.flink(Z1,Zd1,s,tsim,1);

% = = Block_0_1_0_1_2_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk12 = -(RLlnk2_119+s.dpt(1,1)-s.dpt(1,6));
  Plnk22 = -(RLlnk2_219-s.dpt(2,6));
  Z2 = sqrt(Plnk12*Plnk12+Plnk22*Plnk22+RLlnk2_319*RLlnk2_319);
  e12 = Plnk12/Z2;
  e22 = Plnk22/Z2;
  e32 = -RLlnk2_319/Z2;
  Zd2 = qd(10)*e22*s.dpt(3,19)*C10-e12*(qd(10)*RLlnk2_219*S9+qd(9)*RLlnk2_319)-e32*(qd(10)*RLlnk2_219*C9-qd(9)*RLlnk2_119);
 
% Link Force Computation 

  Flink2 = usrfun.flink(Z2,Zd2,s,tsim,2);

% = = Block_0_1_0_1_3_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk13 = -(RLlnk4_121-s.dpt(1,8));
  Plnk23 = -(RLlnk4_221+s.dpt(2,2)-s.dpt(2,8));
  Z3 = sqrt(Plnk13*Plnk13+Plnk23*Plnk23+RLlnk4_321*RLlnk4_321);
  e13 = Plnk13/Z3;
  e23 = Plnk23/Z3;
  e33 = -RLlnk4_321/Z3;
  Zd3 = qd(12)*e23*s.dpt(3,21)*C12-e13*(qd(11)*RLlnk4_321+qd(12)*RLlnk4_221*S11)+e33*(qd(11)*RLlnk4_121-qd(12)*RLlnk4_221*C11);
 
% Link Force Computation 

  Flink3 = usrfun.flink(Z3,Zd3,s,tsim,3);

% = = Block_0_1_0_1_4_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk14 = -(RLlnk6_123-s.dpt(1,7));
  Plnk24 = -(RLlnk6_223+s.dpt(2,2)-s.dpt(2,7));
  Z4 = sqrt(Plnk14*Plnk14+Plnk24*Plnk24+RLlnk6_323*RLlnk6_323);
  e14 = Plnk14/Z4;
  e24 = Plnk24/Z4;
  e34 = -RLlnk6_323/Z4;
  Zd4 = qd(12)*e24*s.dpt(3,21)*C12-e14*(qd(11)*RLlnk6_323+qd(12)*RLlnk6_223*S11)+e34*(qd(11)*RLlnk6_123-qd(12)*RLlnk6_223*C11);
 
% Link Force Computation 

  Flink4 = usrfun.flink(Z4,Zd4,s,tsim,4);

% = = Block_0_1_0_1_5_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk15 = RLlnk9_126-s.dpt(1,10)+s.dpt(1,3);
  Plnk25 = RLlnk9_226-s.dpt(2,10);
  Z5 = sqrt(Plnk15*Plnk15+Plnk25*Plnk25+RLlnk9_326*RLlnk9_326);
  e15 = Plnk15/Z5;
  e25 = Plnk25/Z5;
  e35 = RLlnk9_326/Z5;
  Zd5 = -(qd(14)*e25*s.dpt(3,23)*C14-e15*(qd(13)*RLlnk9_326+qd(14)*RLlnk9_226*S13)+e35*(qd(13)*RLlnk9_126-qd(14)*RLlnk9_226*C13));
 
% Link Force Computation 

  Flink5 = usrfun.flink(Z5,Zd5,s,tsim,5);

% = = Block_0_1_0_1_6_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk16 = RLlnk11_128+s.dpt(1,3)-s.dpt(1,9);
  Plnk26 = RLlnk11_228-s.dpt(2,9);
  Z6 = sqrt(Plnk16*Plnk16+Plnk26*Plnk26+RLlnk11_328*RLlnk11_328);
  e16 = Plnk16/Z6;
  e26 = Plnk26/Z6;
  e36 = RLlnk11_328/Z6;
  Zd6 = -(qd(14)*e26*s.dpt(3,23)*C14-e16*(qd(13)*RLlnk11_328+qd(14)*RLlnk11_228*S13)+e36*(qd(13)*RLlnk11_128-qd(14)*RLlnk11_228*C13));
 
% Link Force Computation 

  Flink6 = usrfun.flink(Z6,Zd6,s,tsim,6);

% = = Block_0_1_0_1_7_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk17 = -(RLlnk12_129-s.dpt(1,12));
  Plnk27 = -(RLlnk12_229-s.dpt(2,12)+s.dpt(2,4));
  Z7 = sqrt(Plnk17*Plnk17+Plnk27*Plnk27+RLlnk12_329*RLlnk12_329);
  e17 = Plnk17/Z7;
  e27 = Plnk27/Z7;
  e37 = -RLlnk12_329/Z7;
  Zd7 = qd(16)*e27*s.dpt(3,25)*C16-e17*(qd(15)*RLlnk12_329+qd(16)*RLlnk12_229*S15)+e37*(qd(15)*RLlnk12_129-qd(16)*RLlnk12_229*C15);
 
% Link Force Computation 

  Flink7 = usrfun.flink(Z7,Zd7,s,tsim,7);

% = = Block_0_1_0_1_8_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk18 = -(RLlnk14_131-s.dpt(1,11));
  Plnk28 = -(RLlnk14_231-s.dpt(2,11)+s.dpt(2,4));
  Z8 = sqrt(Plnk18*Plnk18+Plnk28*Plnk28+RLlnk14_331*RLlnk14_331);
  e18 = Plnk18/Z8;
  e28 = Plnk28/Z8;
  e38 = -RLlnk14_331/Z8;
  Zd8 = qd(16)*e28*s.dpt(3,25)*C16-e18*(qd(15)*RLlnk14_331+qd(16)*RLlnk14_231*S15)+e38*(qd(15)*RLlnk14_131-qd(16)*RLlnk14_231*C15);
 
% Link Force Computation 

  Flink8 = usrfun.flink(Z8,Zd8,s,tsim,8);

% = = Block_0_1_0_1_9_2 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk19 = RLlnk17_134-s.dpt(1,14);
  Z9 = sqrt(Plnk19*Plnk19+RLlnk17_234*RLlnk17_234+RLlnk17_334*RLlnk17_334);
  e19 = Plnk19/Z9;
  e29 = RLlnk17_234/Z9;
  e39 = RLlnk17_334/Z9;
  Zd9 = e19*(OMlnk17_28*RLlnk17_334-OMlnk17_38*RLlnk17_234)-e29*(qd(7)*RLlnk17_334-OMlnk17_38*RLlnk17_134)+e39*(qd(7)*RLlnk17_234-OMlnk17_28*...
 RLlnk17_134);
 
% Link Force Computation 

  Flink9 = usrfun.flink(Z9,Zd9,s,tsim,9);

% = = Block_0_1_0_1_10_2 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk210 = RLlnk19_236-s.dpt(2,13);
  Z10 = sqrt(Plnk210*Plnk210+RLlnk19_136*RLlnk19_136+RLlnk19_336*RLlnk19_336);
  e110 = RLlnk19_136/Z10;
  e210 = Plnk210/Z10;
  e310 = RLlnk19_336/Z10;
  Zd10 = e110*(OMlnk19_28*RLlnk19_336-OMlnk19_38*RLlnk19_236)-e210*(qd(7)*RLlnk19_336-OMlnk19_38*RLlnk19_136)+e310*(qd(7)*RLlnk19_236-OMlnk19_28*...
 RLlnk19_136);
 
% Link Force Computation 

  Flink10 = usrfun.flink(Z10,Zd10,s,tsim,10);

% = = Block_0_1_0_2_2_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk11 = Flink1*e11;
  fPlnk21 = Flink1*e21;
  fPlnk31 = Flink1*e31;

% = = Block_0_1_0_2_2_3 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk11 = Flink1*(e11*C9-e31*S9);
  fSlnk21 = Flink1*(e21*C10+S10*(e11*S9+e31*C9));
  s.frc(1,10) = s.frc(1,10)-fSlnk11;
  s.frc(2,10) = s.frc(2,10)-fSlnk21;
  s.frc(3,10) = s.frc(3,10)-Flink1*(ROlnk1_710*e11+ROlnk1_910*e31-e21*S10);
  s.trq(1,10) = s.trq(1,10)+fSlnk21*(s.dpt(3,19)-s.l(3,10));
  s.trq(2,10) = s.trq(2,10)-fSlnk11*(s.dpt(3,19)-s.l(3,10));

% = = Block_0_1_0_2_3_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk12 = Flink2*e12;
  fSlnk22 = Flink2*e22;
  fSlnk32 = Flink2*e32;

% = = Block_0_1_0_2_3_3 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk12 = Flink2*(e12*C9-e32*S9);
  fPlnk22 = Flink2*(e22*C10+S10*(e12*S9+e32*C9));
  fPlnk32 = Flink2*(ROlnk2_710*e12+ROlnk2_910*e32-e22*S10);
  frc(1,10) = fPlnk12+s.frc(1,10);
  frc(2,10) = fPlnk22+s.frc(2,10);
  frc(3,10) = fPlnk32+s.frc(3,10);
  trq(1,10) = s.trq(1,10)-fPlnk22*(s.dpt(3,19)-s.l(3,10));
  trq(2,10) = s.trq(2,10)+fPlnk12*(s.dpt(3,19)-s.l(3,10));

% = = Block_0_1_0_2_4_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk13 = Flink3*e13;
  fSlnk23 = Flink3*e23;
  fSlnk33 = Flink3*e33;

% = = Block_0_1_0_2_4_4 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk13 = Flink3*(e13*C11-e33*S11);
  fPlnk23 = Flink3*(e23*C12+S12*(e13*S11+e33*C11));
  s.frc(1,12) = s.frc(1,12)+fPlnk13;
  s.frc(2,12) = s.frc(2,12)+fPlnk23;
  s.frc(3,12) = s.frc(3,12)+Flink3*(ROlnk4_712*e13+ROlnk4_912*e33-e23*S12);
  s.trq(1,12) = s.trq(1,12)-fPlnk23*(s.dpt(3,21)-s.l(3,12));
  s.trq(2,12) = s.trq(2,12)+fPlnk13*(s.dpt(3,21)-s.l(3,12));

% = = Block_0_1_0_2_5_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk14 = Flink4*e14;
  fSlnk24 = Flink4*e24;
  fSlnk34 = Flink4*e34;

% = = Block_0_1_0_2_5_4 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk14 = Flink4*(e14*C11-e34*S11);
  fPlnk24 = Flink4*(e24*C12+S12*(e14*S11+e34*C11));
  fPlnk34 = Flink4*(ROlnk6_712*e14+ROlnk6_912*e34-e24*S12);
  frc(1,12) = fPlnk14+s.frc(1,12);
  frc(2,12) = fPlnk24+s.frc(2,12);
  frc(3,12) = fPlnk34+s.frc(3,12);
  trq(1,12) = s.trq(1,12)-fPlnk24*(s.dpt(3,21)-s.l(3,12));
  trq(2,12) = s.trq(2,12)+fPlnk14*(s.dpt(3,21)-s.l(3,12));

% = = Block_0_1_0_2_6_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk15 = Flink5*e15;
  fPlnk25 = Flink5*e25;
  fPlnk35 = Flink5*e35;

% = = Block_0_1_0_2_6_5 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk15 = Flink5*(e15*C13-e35*S13);
  fSlnk25 = Flink5*(e25*C14+S14*(e15*S13+e35*C13));
  s.frc(1,14) = s.frc(1,14)-fSlnk15;
  s.frc(2,14) = s.frc(2,14)-fSlnk25;
  s.frc(3,14) = s.frc(3,14)-Flink5*(ROlnk9_714*e15+ROlnk9_914*e35-e25*S14);
  s.trq(1,14) = s.trq(1,14)+fSlnk25*(s.dpt(3,23)-s.l(3,14));
  s.trq(2,14) = s.trq(2,14)-fSlnk15*(s.dpt(3,23)-s.l(3,14));

% = = Block_0_1_0_2_7_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk16 = Flink6*e16;
  fPlnk26 = Flink6*e26;
  fPlnk36 = Flink6*e36;

% = = Block_0_1_0_2_7_5 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk16 = Flink6*(e16*C13-e36*S13);
  fSlnk26 = Flink6*(e26*C14+S14*(e16*S13+e36*C13));
  fSlnk36 = Flink6*(ROlnk11_714*e16+ROlnk11_914*e36-e26*S14);
  frc(1,14) = -(fSlnk16-s.frc(1,14));
  frc(2,14) = -(fSlnk26-s.frc(2,14));
  frc(3,14) = -(fSlnk36-s.frc(3,14));
  trq(1,14) = s.trq(1,14)+fSlnk26*(s.dpt(3,23)-s.l(3,14));
  trq(2,14) = s.trq(2,14)-fSlnk16*(s.dpt(3,23)-s.l(3,14));

% = = Block_0_1_0_2_8_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk17 = Flink7*e17;
  fSlnk27 = Flink7*e27;
  fSlnk37 = Flink7*e37;

% = = Block_0_1_0_2_8_6 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk17 = Flink7*(e17*C15-e37*S15);
  fPlnk27 = Flink7*(e27*C16+S16*(e17*S15+e37*C15));
  s.frc(1,16) = s.frc(1,16)+fPlnk17;
  s.frc(2,16) = s.frc(2,16)+fPlnk27;
  s.frc(3,16) = s.frc(3,16)+Flink7*(ROlnk12_716*e17+ROlnk12_916*e37-e27*S16);
  s.trq(1,16) = s.trq(1,16)-fPlnk27*(s.dpt(3,25)-s.l(3,16));
  s.trq(2,16) = s.trq(2,16)+fPlnk17*(s.dpt(3,25)-s.l(3,16));

% = = Block_0_1_0_2_9_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk18 = Flink8*e18;
  fSlnk28 = Flink8*e28;
  fSlnk38 = Flink8*e38;

% = = Block_0_1_0_2_9_6 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk18 = Flink8*(e18*C15-e38*S15);
  fPlnk28 = Flink8*(e28*C16+S16*(e18*S15+e38*C15));
  fPlnk38 = Flink8*(ROlnk14_716*e18+ROlnk14_916*e38-e28*S16);
  frc(1,16) = fPlnk18+s.frc(1,16);
  frc(2,16) = fPlnk28+s.frc(2,16);
  frc(3,16) = fPlnk38+s.frc(3,16);
  trq(1,16) = s.trq(1,16)-fPlnk28*(s.dpt(3,25)-s.l(3,16));
  trq(2,16) = s.trq(2,16)+fPlnk18*(s.dpt(3,25)-s.l(3,16));

% = = Block_0_1_0_2_10_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk19 = Flink9*e19;
  fPlnk29 = Flink9*e29;
  fPlnk39 = Flink9*e39;
  s.frc(1,6) = s.frc(1,6)+fPlnk11+fPlnk15+fPlnk16+fPlnk19-fSlnk12-fSlnk13-fSlnk14-fSlnk17-fSlnk18;
  s.frc(2,6) = s.frc(2,6)+fPlnk21+fPlnk25+fPlnk26+fPlnk29-fSlnk22-fSlnk23-fSlnk24-fSlnk27-fSlnk28;
  s.frc(3,6) = s.frc(3,6)+fPlnk31+fPlnk35+fPlnk36+fPlnk39-fSlnk32-fSlnk33-fSlnk34-fSlnk37-fSlnk38;
  s.trq(1,6) = s.trq(1,6)+fPlnk21*s.l(3,6)+fPlnk25*s.l(3,6)+fPlnk26*s.l(3,6)+fPlnk29*s.l(3,6)+fPlnk31*s.dpt(2,5)+fPlnk35*s.dpt(2,10)+fPlnk36*...
 s.dpt(2,9)-fSlnk22*s.l(3,6)-fSlnk23*s.l(3,6)-fSlnk24*s.l(3,6)-fSlnk27*s.l(3,6)-fSlnk28*s.l(3,6)-fSlnk32*s.dpt(2,6)-fSlnk33*s.dpt(2,8)-fSlnk34*...
 s.dpt(2,7)-fSlnk37*s.dpt(2,12)-fSlnk38*s.dpt(2,11);
  s.trq(2,6) = s.trq(2,6)-fPlnk11*s.l(3,6)-fPlnk15*s.l(3,6)-fPlnk16*s.l(3,6)-fPlnk19*s.l(3,6)-fPlnk31*s.dpt(1,5)-fPlnk35*s.dpt(1,10)-fPlnk36*...
 s.dpt(1,9)-fPlnk39*s.dpt(1,14)+fSlnk12*s.l(3,6)+fSlnk13*s.l(3,6)+fSlnk14*s.l(3,6)+fSlnk17*s.l(3,6)+fSlnk18*s.l(3,6)+fSlnk32*s.dpt(1,6)+fSlnk33*...
 s.dpt(1,8)+fSlnk34*s.dpt(1,7)+fSlnk37*s.dpt(1,12)+fSlnk38*s.dpt(1,11);
  s.trq(3,6) = s.trq(3,6)-fPlnk11*s.dpt(2,5)-fPlnk15*s.dpt(2,10)-fPlnk16*s.dpt(2,9)+fPlnk21*s.dpt(1,5)+fPlnk25*s.dpt(1,10)+fPlnk26*s.dpt(1,9)+...
 fPlnk29*s.dpt(1,14)+fSlnk12*s.dpt(2,6)+fSlnk13*s.dpt(2,8)+fSlnk14*s.dpt(2,7)+fSlnk17*s.dpt(2,12)+fSlnk18*s.dpt(2,11)-fSlnk22*s.dpt(1,6)-fSlnk23*...
 s.dpt(1,8)-fSlnk24*s.dpt(1,7)-fSlnk27*s.dpt(1,12)-fSlnk28*s.dpt(1,11);

% = = Block_0_1_0_2_10_2 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk19 = Flink9*(ROlnk17_28*e29+ROlnk17_38*e39+e19*C8);
  fSlnk29 = Flink9*(e29*C7+e39*S7);
  fSlnk39 = Flink9*(ROlnk17_88*e29+ROlnk17_98*e39+e19*S8);
  s.frc(1,8) = s.frc(1,8)-fSlnk19;
  s.frc(2,8) = s.frc(2,8)-fSlnk29;
  s.frc(3,8) = s.frc(3,8)-fSlnk39;
  s.trq(1,8) = s.trq(1,8)+fSlnk29*(s.dpt(3,16)-s.l(3,8));
  s.trq(2,8) = s.trq(2,8)-fSlnk19*(s.dpt(3,16)-s.l(3,8))+fSlnk39*s.dpt(1,16);
  s.trq(3,8) = s.trq(3,8)-fSlnk29*s.dpt(1,16);

% = = Block_0_1_0_2_11_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk110 = Flink10*e110;
  fPlnk210 = Flink10*e210;
  fPlnk310 = Flink10*e310;
  frc(1,6) = fPlnk110+s.frc(1,6);
  frc(2,6) = fPlnk210+s.frc(2,6);
  frc(3,6) = fPlnk310+s.frc(3,6);
  trq(1,6) = s.trq(1,6)+fPlnk210*s.l(3,6)+fPlnk310*s.dpt(2,13);
  trq(2,6) = s.trq(2,6)-fPlnk110*s.l(3,6);
  trq(3,6) = s.trq(3,6)-fPlnk110*s.dpt(2,13);

% = = Block_0_1_0_2_11_2 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk110 = Flink10*(e110*C8+e210*S7*S8-e310*C7*S8);
  fSlnk210 = Flink10*(e210*C7+e310*S7);
  fSlnk310 = Flink10*(ROlnk19_88*e210+ROlnk19_98*e310+e110*S8);
  frc(1,8) = -(fSlnk110-s.frc(1,8));
  frc(2,8) = -(fSlnk210-s.frc(2,8));
  frc(3,8) = -(fSlnk310-s.frc(3,8));
  trq(1,8) = s.trq(1,8)+fSlnk210*(s.dpt(3,15)-s.l(3,8))-fSlnk310*s.dpt(2,15);
  trq(2,8) = s.trq(2,8)-fSlnk110*(s.dpt(3,15)-s.l(3,8));
  trq(3,8) = s.trq(3,8)+fSlnk110*s.dpt(2,15);

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  trq(3,10) = s.trq(3,10);
  trq(3,12) = s.trq(3,12);
  trq(3,14) = s.trq(3,14);
  trq(3,16) = s.trq(3,16);
  Flnk(1) = Flink1;
  Flnk(2) = Flink2;
  Flnk(3) = Flink3;
  Flnk(4) = Flink4;
  Flnk(5) = Flink5;
  Flnk(6) = Flink6;
  Flnk(7) = Flink7;
  Flnk(8) = Flink8;
  Flnk(9) = Flink9;
  Flnk(10) = Flink10;
  Z(1) = Z1;
  Z(2) = Z2;
  Z(3) = Z3;
  Z(4) = Z4;
  Z(5) = Z5;
  Z(6) = Z6;
  Z(7) = Z7;
  Z(8) = Z8;
  Z(9) = Z9;
  Z(10) = Z10;
  Zd(1) = Zd1;
  Zd(2) = Zd2;
  Zd(3) = Zd3;
  Zd(4) = Zd4;
  Zd(5) = Zd5;
  Zd(6) = Zd6;
  Zd(7) = Zd7;
  Zd(8) = Zd8;
  Zd(9) = Zd9;
  Zd(10) = Zd10;

% ====== END Task 0 ====== 

  

