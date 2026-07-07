function [Fx,Fy] = tire_model(Fz,SA,SL,IA)

%%比例系数

LMUX                       = 1 ;
LMUY                       = 1.00000;

%纯纵向因子
PCX1                       = 1.1383548030456672  ;   
PDX1                       = 1.581906631207076    ;
PDX2                       = -0.25788348186798904 ;
PDX3                       = 140.6421533432992333  ;
PEX1                       = -0.6114816420143618  ;
PEX2                       = 0.3204467936531163   ;
PEX3                       = -0.17535674505791818 ;
PKX1                       = 45.81262520831058    ;
PKX2                       = -0.1732748761675123  ;
PKX3                       = -0.17432458310515622 ;



%复合纵向因子
RBX1                       = 12.152497798485447   ;%Slope factor for combined slip Fx reduction
RBX2                       = -28.076987505913554  ;%Variation of slope Fx reduction with kappa
RCX1                       = 1.4896938338567027   ;%Shape factor for combined slip Fx reduction
REX1                       = 0.887276885729295    ;%Curvature factor of combined Fx with load
REX2                       = 0.08082619222120445  ;%Shift factor for combined slip Fx reduction

%纯侧向因子
PCY1                       = 1.1237750659722274   ;%Shape factor Cfy for lateral forces
PDY1                       = 1.5314395174403543   ;%Lateral friction Muy
PDY2                       = -0.1914633442279279  ;%Variation of friction Muy with load
PDY3                       = 116.6182537898045;
PEY1                       = -0.9117452727077456  ;%Lateral curvature Efy at Fznom
PEY2                       = -0.08698015482427382 ;%Variation of curvature Efy with load
PKY1                       = -71.3449869144833    ;%Maximum value of stiffness Kfy/Fznom
PKY2                       = 3.7869409523912503   ;%Load at which Kfy reaches maximum value
%PHY1                       = 0.003343018392671645 ;%Horizontal shift Shy at Fznom
%PHY2                       = 0.00022105554637001716;%Variation of shift Shy with load
PHY1=0;
PHY2=0;
PHY3                       = 0.24722484067575076  ;%Variation of shift Shy with camber
% PVY1                       = -0.0001978217232760216;%Vertical shift in Svy/Fz at Fznom
% PVY2                       = 0.0006666690742451348;%Variation of shift Svy/Fz with load
PVY1=0;
PVY2=0;
PVY3                       = -3.568748110061051   ;%Variation of shift Svy/Fz with camber
PVY4                       = -0.5564576508815049  ;%Variation of shift Svy/Fz with camber and load           
%复合侧向因子
RBY1                       =  31.0038719670727 ; 
RBY2                       = -19.158859638212988 ;
RBY3                       =-0.003383645953256665;
RCY1                       = 0.9985783980568643  ;    
REY1                       = 0.184217099111696  ; 
REY2                       = -0.6445552767602826; 



Fznom=700;
dFz=(Fz-Fznom)/Fznom;
%纯纵向
Cx=PCX1;
mux=(PDX1+PDX2.*dFz).*(1-PDX3.*IA.*IA)*LMUX;
Dx=mux.*Fz;
Kx=Fz.*(PKX1+PKX2.*dFz).*exp(PKX3.*dFz);
Bx=Kx./(Cx.*Dx);
Ex=(PEX1+PEX2.*dFz+dFz.*dFz.*PEX3);
Fx0=Dx.*sin(Cx.*atan(Bx.*SL-Ex.*(Bx.*SL-atan(Bx.*SL))));
%纯侧向
Cy=PCY1;
muy=(PDY1+PDY2.*dFz).*(1-PDY3.*IA.*IA)*LMUY;
Dy=muy*Fz;
Ky=PKY1*Fznom.*sin(2*atan(Fz/(PKY2*Fznom)));
By=Ky./(Cy.*Dy);
SHy=(PHY1+PHY2.*dFz)+PHY3.*IA;
SVy=Fz.*((PVY1+PVY2.*dFz)+(PVY3+PVY4.*dFz).*IA)*LMUY;
Ey=(PEY1+PEY2.*dFz);
SAy=SA+SHy;
Fy0=Dy.*sin(Cy.*atan(By.*SAy-Ey.*(By.*SAy-atan(By.*SAy))))+SVy;
%纵向复合
Cxa=RCX1;
Exa=REX1+REX2.*dFz;
Bxa=RBX1.*cos(atan(RBX2.*SL));
Gxa=cos(Cxa.*atan(Bxa.*SA-Exa.*(Bxa.*SA-atan(Bxa.*SA))))/1;
Fx=Fx0*Gxa;
%侧向复合
Byk=RBY1.*cos(atan(RBY2.*(SA-RBY3)));
Cyk=RCY1;
Eyk=REY1+REY2.*dFz;
Gyk=cos(Cyk.*atan(Byk.*SL-Eyk.*(Byk.*SL-atan(Byk.*SL))));
Fy=Fy0*Gyk;
end