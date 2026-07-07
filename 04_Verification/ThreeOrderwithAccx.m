close all;clear;clc

%%轮胎参数
%%20250504添加了侧偏角虚拟控制
%%20250506打上标签，这个就是最终最终的被动逆动力学模型。

tic

%% 车辆参数
m = 263;    % 整车质量 (kg)
I_z = 150*1.5;  % z轴惯量 (kgm虏)
lf = 0.8427;   % 质心与前轴距离 (m)
lr = 0.7473;   % 质心与后轴距离 (m)
gravity=9.81; %重力加速度
wbf=1.24;   %前轴轮距
wbr=1.22;   %后轴轮距
L=lf+lr;       %轴距
cgheight=0.33;  %质心高度
ms=240;    %簧载质量
hs=0.317;  %簧载质量质心高度
hr=0.288;  %侧倾力臂
% 动力总成参数
      % 发动机功率(W)
P_max = 50000;      % 发动机功率(W)
P_max2 = 60000;
% 制动系统
Brake_radio=0.68;
SLabslimite=0.35;SAabslimite=0.35;
Ratio=0.2032;%车轮半径(m)
wheel_ly=1.5;% 车轮旋转惯量 (kgm^2)
% 悬架参数
rollratiof=0.48;   %前后侧倾刚度比
rollratior=1-rollratiof;
% importantlimit=1;
% importantrate=inf;

%importantlimit=0;
kf=40717;         %前悬架垂直刚度（N/m）
kr=40123;         %后悬架垂直刚度（N/m）
krf=21695;        %前悬架角刚度（Nm/rad）
krr=26571;        %后悬架角刚度（Nm/rad）
Kcf=-0.66497;     %前悬架camber gain（rad/m）
Kcr=-0.71453;     %后悬架camber gain（rad/m）
Cf0=0*pi/180;  %前悬架静态camber（rad）
Cr0=-0*pi/180;            %后悬架静态camber（rad）
RG=0.7*pi/180/9.81;  %侧倾梯度[rad/(m/s^2)]
hrcf=0.25;           %前RC高度
hrcr=0.4;            %后RC高度
%空气动力学参数
aerofrount=0;
aerorear=0;
aerodrag=0;
%操纵限制
maxsteerpersecond=30/180*pi;
maxSLpersecond=inf;
Accxmax=20;
Accymax=20;


%%松弛因子
trackwidth=1.7;


%%启动参数
u_int=20;
maxspeed=100;%%最高车速限制
minspeed=7;%%最低车速限制
maxyspeed=10;%%最高侧向速度限制
maxdpsi=30/minspeed;
maxxi=4;

totalfrontdown=aerofrount*u_int*u_int;
totalreardown=aerorear*u_int*u_int;
downmax=maxspeed*maxspeed*(aerofrount+aerorear);
Nfmax=4000;
maxsteering=30*pi/180;


%%完全归一化
nuni=trackwidth*2; % 赛车与中心线距离 (m)
xiuni=maxxi*2;
uuni = maxspeed; % 赛车x方向速度 (m/s)
uunint=u_int/uuni;


vuni = maxyspeed*2;         % 赛车y方向速度  (m/s)
dpsiuni = maxdpsi*2;   % 赛车横摆角速度 (rad/s)

steeringuni = maxsteering*2; % 方向盘转角(rad)
SLfluni= SLabslimite*2;     %左前轮滑移率(-)
SLfruni = SLabslimite*2;     % 右前轮滑移率 (-)
SLrluni = SLabslimite*2;     % 左后轮滑移率 (-)
SLrruni = SLabslimite*2;     % 右后轮滑移率 (-)
Accxuni = Accxmax*2;     % 赛车纵向加速度
Accyuni = Accymax*2;     % 赛车侧向加速度
Nfluni = Nfmax;     % 左前轮垂向力
Nfruni = Nfmax;     % 右前轮垂向力
Nrluni = Nfmax;     % 左后轮垂向力
Nrruni = Nfmax;     % 左前轮垂向力


Nflint = (totalfrontdown/2+gravity*m*lr/(lr+lf))/Nfluni;     % 左前轮主动力
Nfrint = (totalfrontdown/2+gravity*m*lr/(lr+lf))/Nfruni;     % 右前轮主动力
Nrlint = (totalreardown/2+gravity*m*lf/(lr+lf))/Nrluni;     % 左后轮主动力
Nrrint = (totalreardown/2+gravity*m*lf/(lr+lf))/Nrruni;     % 右后轮主动力
Fxuni= Nfmax*2;
Fyuni= Nfmax*2;
TDuni=Nfmax*2;
TBuni=Nfmax*2;
actorquef=0;
actorquer=0;

activetorqueflimit=0;
activetorquerlimit=0;
actorquefspeed=50000*2;
actorquerspeed=50000*2;
actorquefuni=activetorqueflimit*2;
actorqueruni=activetorquerlimit*2;
%%约束归一化
maxsteeringspeeduni=maxsteerpersecond*2;



omegaupbound=maxspeed*(1+SLabslimite);
omegalowbound=minspeed*(1-SLabslimite);
omegalimit=omegaupbound-omegalowbound;
omegauniuplimit=omegaupbound/omegalimit;
omegaunilowlimit=omegalowbound/omegalimit;
omegaint=u_int/omegalimit;
%%非凸约束，归一化与调节缩放，这里非常重要。

power1uni=1;
power2uni=1;
diff1uni=10;
diff2uni=16;
tireFxuni=1000;
tireFyuni=1000;

brackuni=100;
SAtuuni=1;
maxrolluni=0.01;

% frountfactor=1000/Fxuni;
frountfactor=0/Fxuni;
% rearfactor=inf/diff1uni;
rearfactor=0/Fxuni;
brakefactor=0/brackuni;
%%主动悬架参数
verfactor=0;
maxroll1=0;
maxroll2=0;
maxroll3=0/(Nfmax*(wbr+wbf)/2)*maxrolluni;

fronttoe=0/180*pi;
reartoe=0/180*pi;

%主动车轮外倾

cambermax=0/180*pi;
camberuni=2*cambermax;
%%侧偏角范围
SAuni=SAabslimite*2;

% 控制算法参数

      afl    = -0.002552;
      c10fl  = +0.016399;
      c11fl  = +0.010656;
      c20fl  = -0.070451;
      c21fl  = +0.223052;
      c30fl  = +0.126016;
      c31fl  = -0.212556;

      afr    = -0.042968;
      c10fr  = -0.009274;
      c11fr  = -0.047973;
      c20fr  = -0.267264;
      c21fr  = -1.184308;
      c30fr  = -0.161004;
      c31fr  = +1.170954;

      arl    = +0.038376;
      c10rl  = +0.006650;
      c11rl  = +0.063173;
      c20rl  = -0.028846;
      c21rl  = +0.035661;
      c30rl  = +0.204810;
      c31rl  = -0.091680;

      arr    = -0.052404;
      c10rr  = -0.000159;
      c11rr  = -0.026302;
      c20rr  = -0.540252;
      c21rr  = -0.286453;
      c30rr  = -0.075102;
      c31rr  = +0.303517;
%% 道路模型

step_length = 1;

load('Track.mat');
%% Vehicle dynamics
% 系统状态量
n = casadi.SX.sym('n');         % 赛车与中心线距离 (m)
xi = casadi.SX.sym('xi');       % 赛车与中心线夹角 (rad) 
u = casadi.SX.sym('u');         % 赛车x方向速度 (m/s)
v = casadi.SX.sym('v');         % 赛车y方向速度  (m/s)
dpsi = casadi.SX.sym('dpsi');   % 赛车横摆角速度 (rad/s)


states = [n;xi;u;v;dpsi];
nx = length(states);
x_init = [0;0;uunint;0;0];

% 系统控制量
steering = casadi.SX.sym('steering'); % 方向盘转角(rad)
SLfl= casadi.SX.sym('SLfl');     %左前轮滑移率(-)
SLfr = casadi.SX.sym('SLfr');     % 右前轮滑移率 (-)
SLrl = casadi.SX.sym('SLrl');     % 左后轮滑移率 (-)
SLrr = casadi.SX.sym('SLrr');     % 右后轮滑移率 (-)
Accx = casadi.SX.sym('Accx');     % 赛车纵向加速度
Accy = casadi.SX.sym('Accy');     % 赛车侧向加速度
Nfl = casadi.SX.sym('Nfl');     % 左前轮垂向力
Nfr = casadi.SX.sym('Nfr');     % 右前轮垂向力
Nrl = casadi.SX.sym('Nrl');     % 左后轮垂向力
Nrr = casadi.SX.sym('Nrr');     % 左前轮垂向力
Fxfl = casadi.SX.sym('Fxfl ');     % 左前轮纵向力
Fxfr  = casadi.SX.sym('Fxfr');     % 右前轮纵向力
Fxrl  = casadi.SX.sym('Fxrl');     % 左后轮纵向力
Fxrr  = casadi.SX.sym('Fxrr');     % 左前轮纵向力
Fyfl  = casadi.SX.sym('Fyfl');     % 左前轮侧向力
Fyfr  = casadi.SX.sym('Fyfr');     % 右前轮侧向力
Fyrl  = casadi.SX.sym('Fyrl');     % 左后轮侧向力
Fyrr  = casadi.SX.sym('Fyrr');     % 左前轮侧向力
omegafl  = casadi.SX.sym('omegafl');     % 左前轮转速
omegafr  = casadi.SX.sym('omegafr');     % 右前轮转速
omegarl  = casadi.SX.sym('omegarl');     % 左后轮转速
omegarr  = casadi.SX.sym('omegarr');     % 左前轮转速
activetorquef  = casadi.SX.sym('activetorquef');     % 前轮主动防倾杆
activetorquer  = casadi.SX.sym('activetorquer');     % 后轮主动防倾杆
torquedrive  = casadi.SX.sym('torquedrive');     % 总驱动力
torquebrake  = casadi.SX.sym('torquedrive');     % 总制动力
Cam_fl=casadi.SX.sym('Cam_fl');           %左前轮camber
Cam_fr=casadi.SX.sym('Cam_fr');           %右前轮camber
Cam_rl=casadi.SX.sym('Cam_rl');           %左后轮camber
Cam_rr=casadi.SX.sym('Cam_rr');           %右后轮camber
SA_fl=casadi.SX.sym('SA_fl');           %左前轮camber
SA_fr=casadi.SX.sym('SA_fr');           %右前轮camber
SA_rl=casadi.SX.sym('SA_rl');           %左后轮camber
SA_rr=casadi.SX.sym('SA_rr');           %右后轮camber

controls = [steering;SLfl;SLfr;SLrl;SLrr;Accx;Accy;Nfl;Nfr;Nrl;
    Nrr;Fxfl;Fxfr;Fxrl;Fxrr;Fyfl;Fyfr;Fyrl;Fyrr;omegafl;omegafr;omegarl;omegarr;activetorquef;
    activetorquer;torquedrive;torquebrake;Cam_fl;Cam_fr;Cam_rl;Cam_rr;SA_fl;SA_fr;SA_rl;SA_rr];
nu = length(controls);
u_init = [0;0;0;0;0;0;0;Nflint;Nfrint;Nrlint;Nrrint;0;0;0;0;0;0;0;0;
    omegaint;omegaint;omegaint;omegaint;0;0;0;0;0;0;0;0;0;0;0;0];



%%复写归一化
nreal= n*nuni;         % 赛车与中心线距离 (m)
xireal = xi*xiuni;       % 赛车与中心线夹角 (rad)
ureal = u*uuni;         % 赛车x方向速度 (m/s)
vreal= v*vuni;         % 赛车y方向速度  (m/s)
dpsireal= dpsi*dpsiuni;   % 赛车横摆角速度 (rad/s)

steeringreal = steering*steeringuni; % 方向盘转角(rad)
SLflreal= SLfl*SLfluni;     %左前轮滑移率(-)
SLfrreal = SLfr*SLfruni;     % 右前轮滑移率 (-)
SLrlreal = SLrl*SLrluni;     % 左后轮滑移率 (-)
SLrrreal = SLrr*SLrruni;     % 右后轮滑移率 (-)
Accxreal = Accx*Accxuni;     % 赛车纵向加速度
Accyreal = Accy*Accyuni;     % 赛车侧向加速度
Nflreal = Nfl*Nfluni;     % 左前轮垂向力
Nfrreal = Nfr*Nfruni;     % 右前轮垂向力
Nrlreal = Nrl*Nrluni;     % 左后轮垂向力
Nrrreal = Nrr*Nrruni;     % 左前轮垂向力
Fxflreal  = Fxfl*Fxuni;     % 左前轮纵向力
Fxfrreal  = Fxfr*Fxuni;     % 右前轮纵向力
Fxrlreal  = Fxrl*Fxuni;     % 左后轮纵向力
Fxrrreal  = Fxrr*Fxuni;     % 左前轮纵向力
Fyflreal  = Fyfl*Fyuni;     % 左前轮侧向力
Fyfrreal  = Fyfr*Fyuni;     % 右前轮侧向力
Fyrlreal  = Fyrl*Fyuni;     % 左后轮侧向力
Fyrrreal  = Fyrr*Fyuni;     % 左前轮侧向力
omegaflreal  = omegafl*omegalimit;     % 左前轮转速
omegafrreal  = omegafr*omegalimit;     % 右前轮转速
omegarlreal  = omegarl*omegalimit;     % 左后轮转速
omegarrreal  = omegarr*omegalimit;     % 左前轮转速
activetorquefreal  = activetorquef*actorquefuni;     % 前轮主动防倾杆
activetorquerreal  = activetorquer*actorqueruni;     % 后轮主动防倾杆
torquedrivereal  = torquedrive*TDuni;     % 总驱动力
torquebrakereal  = torquebrake*TDuni;     % 总制动力
Cam_flreal=Cam_fl*camberuni;           %左前轮camber
Cam_frreal=Cam_fr*camberuni;           %右前轮camber
Cam_rlreal=Cam_rl*camberuni;           %左后轮camber
Cam_rrreal=Cam_rr*camberuni;           %右后轮camber
SA_flreal=SA_fl*SAuni;           %左前轮侧偏角
SA_frreal=SA_fr*SAuni;           %右前轮侧偏角
SA_rlreal=SA_rl*SAuni;           %左后轮侧偏角
SA_rrreal=SA_rr*SAuni;           %右后轮侧偏角


% 赛道曲率
Crv = casadi.SX.sym('Crv');


%%  轮胎力计算
%轮胎垂向力计算
totalfrontdown=aerofrount*ureal*ureal;
totalreardown=aerorear*ureal*ureal;

steeringfl=steeringreal-fronttoe;
steeringfr=steeringreal+fronttoe;
steeringrl=-reartoe;
steeringrr=reartoe;



Fzfl=Nflreal;
Fzfr=Nfrreal;
Fzrl=Nrlreal;
Fzrr=Nrrreal;

SAfl=-steeringfl + atan((vreal + lf*dpsireal)/(ureal-wbf*dpsireal/2));
SAfr=-steeringfr + atan((vreal + lf*dpsireal)/(ureal+wbf*dpsireal/2));
SArl=-steeringrl + atan((vreal - lr*dpsireal)/(ureal-wbr*dpsireal/2));
SArr=-steeringrr + atan((vreal - lr*dpsireal)/(ureal+wbr*dpsireal/2));

Jfl=-ms*hs/(2*L*kf)*Accxreal-ms*hr*rollratiof/(wbf*kf+2*krf/wbf)*Accyreal;
Jfr=-ms*hs/(2*L*kf)*Accxreal+ms*hr*rollratiof/(wbf*kf+2*krf/wbf)*Accyreal;
Jrl= ms*hs/(2*L*kr)*Accxreal-ms*hr*rollratior/(wbr*kf+2*krr/wbr)*Accyreal;
Jrr= ms*hs/(2*L*kr)*Accxreal+ms*hr*rollratior/(wbr*kf+2*krr/wbr)*Accyreal;

camberfl= afl + (c10fl + c11fl*sign(Accxreal/9.8/1.453))*Accyreal/9.8/1.453 + (c20fl + c21fl*sign(Accxreal/9.8/1.453))*Accyreal*Accyreal/9.8/9.8/1.453/1.453   + (c30fl + c31fl*sign(Accxreal/9.8/1.453))*Accyreal*Accyreal*Accyreal/9.8 /9.8 /9.8/1.453/1.453/1.453 ;
camberfr= afr + (c10fr + c11fr*sign(Accxreal/9.8/1.453))*Accyreal/9.8/1.453  + (c20fr + c21fr*sign(Accxreal/9.8/1.453))*Accyreal*Accyreal/9.8/9.8/1.453/1.453   + (c30fr + c31fr*sign(Accxreal/9.8/1.453))*Accyreal*Accyreal*Accyreal/9.8 /9.8 /9.8/1.453/1.453/1.453 ;
camberrl= arl + (c10rl + c11rl*sign(Accxreal/9.8/1.453))*Accyreal/9.8/1.453  + (c20rl + c21rl*sign(Accxreal/9.8/1.453))*Accyreal*Accyreal/9.8/9.8/1.453/1.453   + (c30rl + c31rl*sign(Accxreal/9.8/1.453))*Accyreal*Accyreal*Accyreal/9.8 /9.8 /9.8/1.453/1.453/1.453 ;
camberrr= arr + (c10rr + c11rr*sign(Accxreal/9.8/1.453))*Accyreal/9.8/1.453  + (c20rr + c21rr*sign(Accxreal/9.8/1.453))*Accyreal*Accyreal/9.8/9.8/1.453/1.453   + (c30rr + c31rr*sign(Accxreal/9.8/1.453))*Accyreal*Accyreal*Accyreal/9.8 /9.8 /9.8/1.453/1.453/1.453 ;




[Fxflc,Fyflc] = tire_model(Fzfl,SA_flreal,SLflreal,-camberfl/180*pi);
[Fxfrc,Fyfrc] = tire_model(Fzfr,SA_frreal,SLfrreal,camberfr/180*pi);
[Fxrlc,Fyrlc] = tire_model(Fzrl,SA_rlreal,SLrlreal,-camberrl/180*pi);
[Fxrrc,Fyrrc] = tire_model(Fzrr,SA_rrreal,SLrrreal,camberrr/180*pi);


%左前轮计算
%纯纵向
fxc=casadi.SX.sym('acclimite',4);
fxc(1)=(Fxflc-Fxflreal)/Fxuni*tireFxuni;
fxc(2)=(Fxfrc-Fxfrreal)/Fxuni*tireFxuni;
fxc(3)=(Fxrlc-Fxrlreal)/Fxuni*tireFxuni;
fxc(4)=(Fxrrc-Fxrrreal)/Fxuni*tireFxuni;
f_fxc = casadi.Function('f_fxc',{states,controls,Crv},{fxc});

fyc=casadi.SX.sym('acclimite',4);
fyc(1)=(Fyflc-Fyflreal)/Fyuni*tireFyuni;
fyc(2)=(Fyfrc-Fyfrreal)/Fyuni*tireFyuni;
fyc(3)=(Fyrlc-Fyrlreal)/Fyuni*tireFyuni;
fyc(4)=(Fyrrc-Fyrrreal)/Fyuni*tireFyuni;
f_fyc = casadi.Function('f_fxc',{states,controls,Crv},{fyc});


fomega=casadi.SX.sym('fomega',4);
fomega(1)=((ureal-dpsireal*wbf*0.5)*(1+SLflreal)-omegaflreal);
fomega(2)=((ureal+dpsireal*wbf*0.5)*(1+SLfrreal)-omegafrreal);
fomega(3)=((ureal-dpsireal*wbr*0.5)*(1+SLrlreal)-omegarlreal);
fomega(4)=((ureal+dpsireal*wbr*0.5)*(1+SLrrreal)-omegarrreal);
f_fomega = casadi.Function('f_fomega',{states,controls,Crv},{fomega});



fltireforcex = Fxflreal/Fxuni;
f_fltireforcex = casadi.Function('fltireforcex',{states,controls,Crv},{fltireforcex});


frtireforcex = Fxfrreal/Fxuni;
f_frtireforcex = casadi.Function('frtireforcex',{states,controls,Crv},{frtireforcex});


rltireforcex =Fxrlreal/Fxuni;
f_rltireforcex = casadi.Function('rltireforcex',{states,controls,Crv},{rltireforcex});


rrtireforcex= Fxrrreal/Fxuni;
f_rrtireforcex = casadi.Function('rrtireforcex',{states,controls,Crv},{rrtireforcex});
% 
fftotal=-torquedrivereal-torquebrakereal+Fxflreal+Fxfrreal+Fxrlreal+Fxrrreal;% 总制动力 
f_fftotal =casadi.Function('fftotal',{states,controls,Crv},{fftotal});
% fftotal=0;     % 总制动力
% f_fftotal = casadi.Function('fftotal',{states,controls,Crv},{fftotal});

%% 最优控制系统
Sf = (step_length-nreal*Crv)/(ureal*cos(xireal)-vreal*sin(xireal));

% 动力学方程
rhs = casadi.SX.sym('rhs',nx);
rhs(1) = (Sf * (ureal*sin(xireal) + vreal*cos(xireal)))/nuni;
rhs(2) = (Sf * dpsireal - Crv*step_length)/xiuni;
rhs(3) = Sf * (dpsireal*vreal + Accxreal)/uuni;
rhs(4) = Sf * (-dpsireal*ureal + Accyreal)/vuni;
rhs(5) = Sf /dpsiuni* 1/I_z*(lf*(Fyflreal*cos(steeringfl) + Fyfrreal*cos(steeringfr) +Fxflreal*sin(steeringfl)+ Fxfrreal*sin(steeringfr))+ (wbf/2)*(-Fxflreal*cos(steeringfl) +Fxfrreal*cos(steeringfr)+ Fyflreal*sin(steeringfl) - Fyfrreal*sin(steeringfr)) - lr*(Fyrlreal*cos(steeringrl)+Fyrrreal*cos(steeringrr)+ Fxrlreal*sin(steeringrl)+ Fxrrreal*sin(steeringrr))+(wbr/2)*(-Fxrlreal*cos(steeringrl) +Fxrrreal*cos(steeringrr)+ Fyrlreal*sin(steeringrl)- Fyrrreal*sin(steeringrr)));

f_rhs = casadi.Function('f_rhs',{states,controls,Crv},{rhs});

%制动力分配
Fenginedrg=200;
traceratio=0;
middleratio=(Brake_radio+traceratio)/2;
changeratio=(Brake_radio-traceratio)/2;
brfvr=Brake_radio/(1-Brake_radio);

brr =(Fxflreal+Fxfrreal-(Fxflreal+Fxfrreal+Fxrlreal+Fxrrreal)*((atan(-(Fxflreal+Fxfrreal+Fxrlreal+Fxrrreal))/pi*2)*changeratio+middleratio))/brackuni;
% brr =0;
f_brr = casadi.Function('f_brr',{states,controls,Crv},{brr});

% 发动机功率限制
Power =((Fxrlreal)*(omegarlreal) + (Fxrrreal)*(omegarrreal))/P_max*power1uni;
f_Power = casadi.Function('f_Power',{states,controls},{Power});


Power2 =((Fxflreal)*(omegaflreal) + (Fxfrreal)*(omegafrreal))/P_max2*power2uni;
f_Power2 = casadi.Function('f_Power2',{states,controls},{Power2});
%差速器开放式差速器
gdrv=0.51;
gbrk=0.29;
T0=100;
releaset0=0;
omega=((omegarlreal)-(omegarrreal));
Tepower=(Fxrlreal+Fxrrreal)*(atan((Fxrlreal+Fxrrreal+Fenginedrg))/pi+0.5)-Fenginedrg*(atan(-(Fxrlreal+Fxrrreal+Fenginedrg))/pi+0.5);


smothfacter=0.2;
dFxr=Fxrlreal-Fxrrreal;
Tdmax=log(exp(smothfacter.*gdrv.*Tepower)+exp(smothfacter.*T0)+exp(-smothfacter.*gbrk.*Tepower))/smothfacter;
tenddT =-2/pi*(atan(10*omega)).*Tdmax;
dT=(dFxr-tenddT)/Fxuni/diff1uni;


f_dT = casadi.Function('dT',{states,controls,Crv},{dT});


%加速度限制

Nlimitetol=(Nflreal+Nfrreal+Nrlreal+Nrrreal-gravity*m-totalfrontdown-totalreardown)/(Nfmax*4);
f_Nlimitetol=casadi.Function('Nlimitetol',{states,controls,Crv},{Nlimitetol});

Nlimitelat=(Nflreal*wbf/2-Nfrreal*wbf/2+Nrlreal*wbr/2-Nrrreal*wbr/2+m*cgheight*Accyreal)/(Nfmax*(wbr+wbf));
f_Nlimitelat=casadi.Function('Nlimitelat',{states,controls,Crv},{Nlimitelat});

Nlimitelong=(Nflreal*lf+Nfrreal*lf-Nrlreal*lr-Nrrreal*lr+m*cgheight*Accxreal)/(Nfmax*(lr+lf));
f_Nlimitelong=casadi.Function('Nlimitelong',{states,controls,Crv},{Nlimitelong});


Nlimitedis=((Nflreal*wbf/2-Nfrreal*wbf/2+activetorquefreal)*(1-rollratiof)-rollratiof*(Nrlreal*wbr/2-Nrrreal*wbr/2+activetorquerreal))/(Nfmax*(wbr+wbf)/2)*maxrolluni;
f_Nlimitedis=casadi.Function('Nlimitedis',{states,controls,Crv},{Nlimitedis});





acclimite=casadi.SX.sym('acclimite',2);
acclimite(1)=(-Accxreal+1/m *(Fxflreal*cos(steeringfl) +Fxfrreal*cos(steeringfr)- Fyflreal*sin(steeringfl) - Fyfrreal*sin(steeringfr)+ Fxrlreal*cos(steeringrl) +Fxrrreal*cos(steeringrr)- Fyrlreal*sin(steeringrl)- Fyrrreal*sin(steeringrr) -aerodrag*ureal*ureal))/Accxuni;
acclimite(2)=(-Accyreal+1/m *(Fyflreal*cos(steeringfl) + Fyfrreal*cos(steeringfr) +Fxflreal*sin(steeringfl)+ Fxfrreal*sin(steeringfr)+ Fyrlreal*cos(steeringrl) +Fyrrreal*cos(steeringrr)+ Fxrlreal*sin(steeringrl)+ Fxrrreal*sin(steeringrr)))/Accyuni;
f_acclimite = casadi.Function('f_acclimite',{states,controls,Crv},{acclimite});


SAlimite=casadi.SX.sym('rhs',4);
SAlimite(1)=SAfl-SA_flreal;
SAlimite(2)=SAfr-SA_frreal;
SAlimite(3)=SArl-SA_rlreal;
SAlimite(4)=SArr-SA_rrreal;
f_SAlimite = casadi.Function('f_SAlimite',{states,controls,Crv},{SAlimite});


Sf = (step_length-nreal*Crv)/(ureal*cos(xireal)-vreal*sin(xireal));
f_Sf = casadi.Function('f_Sf',{states,Crv},{Sf});

M =1; % 梯形积分
ds = 1/M;
X = states;
for i_ = 1:M
    k1 = f_rhs(X,controls,Crv);

   X = k1/Sf;
   % X = X+k1;
end
fX = casadi.Function('fX',{states,controls,Crv},{X});

%% 全配置直接转录
N = Track.N-1;    % number of discretization intervals
% Decision variables (controls)
U = casadi.SX.sym('U',nu,N); 
% Decision variables (states)
X = casadi.SX.sym('X',nx,N+1); 
% P = casadi.SX.sym('X',np,1);
% 车辆动力学方程
g = X(:,1)-X(:,N+1); 
g = [g;X(:,1) - X(:,N)-(fX(X(:,1),U(:,1),Track.curv(1))*f_Sf(X(:,N),Track.curv(N))+fX(X(:,N),U(:,N),Track.curv(N))*f_Sf(X(:,N),Track.curv(N)))/2];
% g = X(:,1)-x_init;
for k = 1:N-1
    g = [g;X(:,k+1) - X(:,k)-(fX(X(:,k+1),U(:,k+1),Track.curv(k+1))*f_Sf(X(:,k+1),Track.curv(k+1))+fX(X(:,k),U(:,k),Track.curv(k))*f_Sf(X(:,k),Track.curv(k)))/2];
end


% 功率限制
for k = 1:N
    g = [g;(f_Power(X(:,k),U(:,k)) - power1uni)];
end
%操纵限制

g = [g;(U(1,1) - u_init(1))*steeringuni/f_Sf(x_init,0)/maxsteeringspeeduni];
for k = 1:(N-1)
    g = [g;(U(1,k+1) - U(1,k))*steeringuni/f_Sf(X(:,k),Track.curv(k))/maxsteeringspeeduni];
end



for k = 1:N

     g = [g;f_fltireforcex(X(:,k),U(:,k),Track.curv(k))-f_frtireforcex(X(:,k),U(:,k),Track.curv(k))];
end

%后轮纵向力约束

for k = 1:N
     g = [g;f_dT(X(:,k),U(:,k),Track.curv(k))];
end

%侧偏角约束

for k = 1:(N)
    g = [g;f_SAlimite(X(:,k),U(:,k),Track.curv(k))];
   
end

%加速度约束

for k = 1:(N)
    g = [g;f_acclimite(X(:,k),U(:,k),Track.curv(k))];
   
end
%制动力分配约束
for k = 1:(N)
    g = [g;f_brr(X(:,k),U(:,k),Track.curv(k))];
   
end

%%垂向力分配约束

for k = 1:(N)
    g = [g;f_Nlimitetol(X(:,k),U(:,k),Track.curv(k))];
   
end

for k = 1:(N)
    g = [g;f_Nlimitelat(X(:,k),U(:,k),Track.curv(k))];
   
end

for k = 1:(N)
    g = [g;f_Nlimitelong(X(:,k),U(:,k),Track.curv(k))];
   
end

for k = 1:(N)
    g = [g;f_Nlimitedis(X(:,k),U(:,k),Track.curv(k))];
   
end

% 后功率限制
for k = 1:N
    g = [g;(f_Power2(X(:,k),U(:,k)) -power2uni)];
end

%%轮胎四向力
for k = 1:(N)
    g = [g;f_fxc(X(:,k),U(:,k),Track.curv(k))];
   
end

for k = 1:(N)
    g = [g;f_fyc(X(:,k),U(:,k),Track.curv(k))];
   
end

%%轮胎转速
for k = 1:(N)
    g = [g;f_fomega(X(:,k),U(:,k),Track.curv(k))];
   
end

%主动前轮轮荷转移限制

g = [g;(U(24,1) - u_init(24))*actorquefuni/f_Sf(x_init,0)/actorquefspeed];
for k = 1:(N-1)
    g = [g;(U(24,k+1) - U(24,k))*actorquefuni/f_Sf(X(:,k),Track.curv(k))/actorquefspeed];
end

%主动后轮轮荷转移限制

g = [g;(U(25,1) - u_init(25))*actorqueruni/f_Sf(x_init,0)/actorquerspeed];
for k = 1:(N-1)
    g = [g;(U(25,k+1) - U(25,k))*actorqueruni/f_Sf(X(:,k),Track.curv(k))/actorquerspeed];
end

for k = 1:(N)
    g = [g;f_fftotal(X(:,k),U(:,k),Track.curv(k))];
   
end

%% 目标函数
obj = 0;
for k = 1:N
    obj = obj + (f_Sf(X(:,k),Track.curv(k)));
end
obj=obj*1000;


Qu = diag([1]);
Qdu = diag([1000]);


obj = obj+ U(1,1).'*Qu*U(1,1)*step_length + (U(1,1)-U(1,N)).'*Qdu*(U(1,1)-U(1,N))/step_length;
for k = 2:N
   obj = obj + U(1,k).'*Qu*U(1,k)*step_length  + (U(1,k)-U(1,k-1)).'*Qdu*(U(1,k)-U(1,k-1))/step_length;
end


Qu = diag([1e-2,1e-2]);
Qdu = diag([10,10]);

obj = obj+ U(24:25,1).'*Qu*U(24:25,1)*step_length + (U(24:25,1)-U(24:25,N)).'*Qdu*(U(24:25,1)-U(24:25,N))/step_length;
for k = 2:N
   obj = obj + U(24:25,k).'*Qu*U(24:25,k)*step_length + (U(24:25,k)-U(24:25,k-1)).'*Qdu*(U(24:25,k)-U(24:25,k-1))/step_length;
end

Qu = diag([1e-2,1e-2]);
Qdu = diag([10,10]);

obj = obj+ U(26:27,1).'*Qu*U(26:27,1) + (U(26:27,1)-U(26:27,N)).'*Qdu*(U(26:27,1)-U(26:27,N));
for k = 2:N
   obj = obj + U(26:27,k).'*Qu*U(26:27,k) + (U(26:27,k)-U(26:27,k-1)).'*Qdu*(U(26:27,k)-U(26:27,k-1));
end

Qu = diag([1e+5]);

for k = 1:N
   obj = obj + (U(26,k).'*Qu*U(27,k))^2;
end

Qu = diag([1,1,1,1]);
Qdu = diag([100,100,100,100]);

obj = obj+ U(28:31,1).'*Qu*U(28:31,1)*step_length + (U(28:31,1)-U(28:31,N)).'*Qdu*(U(28:31,1)-U(28:31,N))/step_length;
for k = 2:N
   obj = obj + U(28:31,k).'*Qu*U(28:31,k)*step_length + (U(28:31,k)-U(28:31,k-1)).'*Qdu*(U(28:31,k)-U(28:31,k-1))/step_length;
end



% ipopt求解器设置
OPT_variables = [reshape(U,nu*N,1);reshape(X,nx*(N+1),1)];
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g);

opts = struct;
% opts.print_time = 0;
% opts.ipopt.linear_solver = 'ma97'; % 指定 MA27
opts.ipopt.max_iter = 24000;
% opts.ipopt.print_level =0;%0,3
 opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
% opts.ipopt.tol=1;

% opts.ipopt.warm_start_init_point = 'yes';
%opts.ipopt.hessian_constant = 'no';
solver = casadi.nlpsol('solver', 'ipopt', nlp_prob,opts);




% 等式约束与不等式约束

args = struct;
args.lbg = zeros(nx*(N+1) + N*31,1);  
args.ubg = zeros(nx*(N+1) + N*31,1);
args.lbg(nx*(N+1)+1:nx*(N+1)+N) = -inf;
%驾驶员限制
args.lbg(nx*(N+1)+N+1:nx*(N+1)+N+1*(N)) = -0.5;  
args.ubg(nx*(N+1)+N+1:nx*(N+1)+N+1*(N))=0.5;

%车轮纵向力约束
args.lbg(nx*(N+1)+2*(N)+1:nx*(N+1) + N*3) = -frountfactor; 
args.ubg(nx*(N+1)+2*(N)+1:nx*(N+1) + N*3)=frountfactor;
args.lbg(nx*(N+1)+3*(N)+1:nx*(N+1) + N*4) =-rearfactor; 
args.ubg(nx*(N+1)+3*(N)+1:nx*(N+1) + N*4)=rearfactor;

%侧偏角约束
args.lbg(nx*(N+1) + N*4+1:nx*(N+1) + N*8) = 0; 
args.ubg(nx*(N+1) + N*4+1:nx*(N+1) + N*8)=0;
args.lbg(nx*(N+1) + N*8+1:nx*(N+1) + N*10) =0; 
args.ubg(nx*(N+1) + N*8+1:nx*(N+1) + N*10)=0;
%%制动力分配约束
args.lbg(nx*(N+1) + N*10+1:nx*(N+1) + N*11) = -brakefactor;  
args.ubg(nx*(N+1) + N*10+1:nx*(N+1) + N*11) = brakefactor;

%%垂向力约束
args.lbg(nx*(N+1) + N*11+1:nx*(N+1) + N*12) = -verfactor; 
args.ubg(nx*(N+1) + N*11+1:nx*(N+1) + N*12) = verfactor; 

args.lbg(nx*(N+1) + N*12+1:nx*(N+1) + N*13) = -maxroll1; 
args.ubg(nx*(N+1) + N*12+1:nx*(N+1) + N*13) = maxroll1;

args.lbg(nx*(N+1) + N*13+1:nx*(N+1) + N*14) = -maxroll2; 
args.ubg(nx*(N+1) + N*13+1:nx*(N+1) + N*14) = maxroll2;


args.lbg(nx*(N+1) + N*14+1:nx*(N+1) + N*15) = -maxroll3; 
args.ubg(nx*(N+1) + N*14+1:nx*(N+1) + N*15) = maxroll3; 


%%后轮电机功率约束
args.lbg(nx*(N+1) + N*15+1:nx*(N+1) + N*16) = -inf; 
args.ubg(nx*(N+1) + N*15+1:nx*(N+1) + N*16) = 0; 


%前轮主动轮荷转移限制
args.lbg(nx*(N+1)+N*28+1:nx*(N+1)+N*29) = -0.5;  
args.ubg(nx*(N+1)+N*28+1:nx*(N+1)+N*29)=0.5;

%后轮主动轮荷转移限制
args.lbg(nx*(N+1)+N*29+1:nx*(N+1)+N*30) = -0.5;  
args.ubg(nx*(N+1)+N*29+1:nx*(N+1)+N*30) = 0.5;



args.lbx = -inf * ones(nu*N + nx*(N+1),1);
args.ubx = inf * ones(nu*N + nx*(N+1),1);
%u limit
args.lbx(1:nu:nu*N) = -0.5;
args.ubx(1:nu:nu*N) = 0.5;
args.lbx(2:nu:nu*N) = -0.5;
args.ubx(2:nu:nu*N) = 0.5;
args.lbx(3:nu:nu*N) = -0.5;
args.ubx(3:nu:nu*N) =0.5;
args.lbx(4:nu:nu*N) = -0.5;
args.ubx(4:nu:nu*N) = 0.5;
args.lbx(5:nu:nu*N) = -0.5;
args.ubx(5:nu:nu*N) = 0.5;
args.lbx(6:nu:nu*N) = -0.5;
args.ubx(6:nu:nu*N) = 0.5;
args.lbx(7:nu:nu*N) = -0.5;
args.ubx(7:nu:nu*N) = 0.5;
args.lbx(8:nu:nu*N) = 0;
args.ubx(8:nu:nu*N) = 1;
args.lbx(9:nu:nu*N) = 0;
args.ubx(9:nu:nu*N) = 1;
args.lbx(10:nu:nu*N) =0;
args.ubx(10:nu:nu*N) = 1;
args.lbx(11:nu:nu*N) = 0;
args.ubx(11:nu:nu*N) = 1;
args.lbx(12:nu:nu*N) = -0.5;
args.ubx(12:nu:nu*N) = 0.5;
args.lbx(13:nu:nu*N) = -0.5;
args.ubx(13:nu:nu*N) = 0.5;
args.lbx(14:nu:nu*N) = -0.5;
args.ubx(14:nu:nu*N) =0.5;
args.lbx(15:nu:nu*N) = -0.5;
args.ubx(15:nu:nu*N) = 0.5;
args.lbx(16:nu:nu*N) = -0.5;
args.ubx(16:nu:nu*N) = 0.5;
args.lbx(17:nu:nu*N) = -0.5;
args.ubx(17:nu:nu*N) = 0.5;
args.lbx(18:nu:nu*N) = -0.5;
args.ubx(18:nu:nu*N) = 0.5;
args.lbx(19:nu:nu*N) = -0.5;
args.ubx(19:nu:nu*N) = 0.5;
args.lbx(20:nu:nu*N) = omegaunilowlimit;
args.ubx(20:nu:nu*N) = omegauniuplimit;
args.lbx(21:nu:nu*N) = omegaunilowlimit;
args.ubx(21:nu:nu*N) = omegauniuplimit;
args.lbx(22:nu:nu*N) = omegaunilowlimit;
args.ubx(22:nu:nu*N) = omegauniuplimit;
args.lbx(23:nu:nu*N) = omegaunilowlimit;
args.ubx(23:nu:nu*N) = omegauniuplimit;
args.lbx(24:nu:nu*N) = -0.5*actorquef;
args.ubx(24:nu:nu*N) = 0.5*actorquef;
args.lbx(25:nu:nu*N) = -0.5*actorquer;
args.ubx(25:nu:nu*N) = 0.5*actorquer;
args.lbx(26:nu:nu*N) = 0;
args.ubx(26:nu:nu*N) = 1;
args.lbx(27:nu:nu*N) = -1;
args.ubx(27:nu:nu*N) = 0;
args.lbx(28:nu:nu*N) = -0.5;
args.ubx(28:nu:nu*N) = 0.5;
args.lbx(29:nu:nu*N) = -0.5;
args.ubx(29:nu:nu*N) = 0.5;
args.lbx(30:nu:nu*N) = -0.5;
args.ubx(30:nu:nu*N) = 0.5;
args.lbx(31:nu:nu*N) = -0.5;
args.ubx(31:nu:nu*N) = 0.5;
args.lbx(32:nu:nu*N) = -0.5;
args.ubx(32:nu:nu*N) = 0.5;
args.lbx(33:nu:nu*N) = -0.5;
args.ubx(33:nu:nu*N) = 0.5;
args.lbx(34:nu:nu*N) = -0.5;
args.ubx(34:nu:nu*N) = 0.5;
args.lbx(35:nu:nu*N) = -0.5;
args.ubx(35:nu:nu*N) = 0.5;



args.ubx(1) = 0;
args.lbx(1) = 0;
args.ubx(nu*(N-1)+1) = 0;
args.lbx(nu*(N-1)+1) = 0;
args.lbx(nu*N+1:nx:nu*N+nx*(N+1))= -0.5;
args.ubx(nu*N+1:nx:nu*N+nx*(N+1))= 0.5;
args.lbx(nu*N+2:nx:nu*N+nx*(N+1))= -0.5;
args.ubx(nu*N+2:nx:nu*N+nx*(N+1))= 0.5;
args.lbx(nu*N+3:nx:nu*N+nx*(N+1)) = 0.07;
args.ubx(nu*N+3:nx:nu*N+nx*(N+1)) = 1;
args.lbx(nu*N+4:nx:nu*N+nx*(N+1))= -0.5;
args.ubx(nu*N+4:nx:nu*N+nx*(N+1))= 0.5;
args.lbx(nu*N+5:nx:nu*N+nx*(N+1))= -0.5;
args.ubx(nu*N+5:nx:nu*N+nx*(N+1))= 0.5;


args.x0 = zeros(nu*N + nx*(N+1),1);

args.x0(8:nu:nu*N) = Nflint;
args.x0(9:nu:nu*N) = Nfrint;
args.x0(10:nu:nu*N) = Nrlint;
args.x0(11:nu:nu*N) = Nrrint;
args.x0(nu*N+3:nx:end) = x_init(3);


%% Solve OCP
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,'lbg', args.lbg, 'ubg', args.ubg);
x_sol = full(sol.x);

% extract result
res.U = x_sol(1:nu*N);
res.U = reshape(res.U,nu,N);
res.X = x_sol(nu*N+1:nu*N+nx*(N+1));
res.X = reshape(res.X,nx,N+1);


for k = 1:N
    res.Power(k) =  full(f_Power(res.X(:,k),res.U(:,k)))*P_max/power1uni;
end

for k = 1:N
    res.Power2(k) =  full(f_Power2(res.X(:,k),res.U(:,k)))*P_max2/power2uni;
end
%%恢复归一化
res.X(1,:)=res.X(1,:)*nuni;
res.X(2,:)=res.X(2,:)*xiuni;
res.X(3,:)=res.X(3,:)*uuni;
res.X(4,:)=res.X(4,:)*vuni;
res.X(5,:)=res.X(5,:)*dpsiuni;


res.U(1,:)=res.U(1,:)*steeringuni;
res.U(2,:)=res.U(2,:)*SLfluni;
res.U(3,:)=res.U(3,:)*SLfruni;
res.U(4,:)=res.U(4,:)*SLrluni;
res.U(5,:)=res.U(5,:)*SLrruni;
res.U(6,:)=res.U(6,:)*Accxuni;
res.U(7,:)=res.U(7,:)*Accyuni;
res.U(8,:)=res.U(8,:)*Nfluni;
res.U(9,:)=res.U(9,:)*Nfruni;
res.U(10,:)=res.U(10,:)*Nrluni;
res.U(11,:)=res.U(11,:)*Nrruni;
res.U(12,:)  = res.U(12,:)*Fxuni;     % 左前轮纵向力
res.U(13,:) = res.U(13,:) *Fxuni;     % 右前轮纵向力
res.U(14,:)= res.U(14,:)*Fxuni;     % 左后轮纵向力
res.U(15,:)= res.U(15,:)*Fxuni;     % 左前轮纵向力
res.U(16,:)= res.U(16,:)*Fyuni;     % 左前轮侧向力
res.U(17,:)= res.U(17,:)*Fyuni;     % 右前轮侧向力
res.U(18,:)= res.U(18,:)*Fyuni;     % 左后轮侧向力
res.U(19,:)= res.U(19,:)*Fyuni;     % 左前轮侧向力
res.U(20,:) = res.U(20,:)*omegalimit;     % 左前轮转速
res.U(21,:)= res.U(21,:)*omegalimit;     % 右前轮转速
res.U(22,:)= res.U(22,:)*omegalimit;     % 左后轮转速
res.U(23,:)  =res.U(23,:)*omegalimit;     % 左前轮转速
res.U(24,:)= res.U(24,:)*actorquefuni;     % 左后轮转速
res.U(25,:)  =res.U(25,:)*actorqueruni;     % 左前轮转速
res.U(26,:) = res.U(26,:) *TDuni;     % 总驱动力
res.U(27,:) = res.U(27,:)*TDuni;     % 总制动力
res.U(28,:) = res.U(28,:)*camberuni;     % 左前轮camber
res.U(29,:) = res.U(29,:)*camberuni;     % 右前轮camber
res.U(30,:) = res.U(30,:)*camberuni;     % 左后轮camber
res.U(31,:) = res.U(31,:)*camberuni;     %右后轮 camber
res.U(32,:) = res.U(32,:)*SAuni;           %左前轮侧偏角
res.U(33,:) = res.U(33,:)*SAuni;           %右前轮侧偏角
res.U(34,:) = res.U(34,:)*SAuni;           %左后轮侧偏角
res.U(35,:) = res.U(35,:)*SAuni;           %右后轮侧偏角



% res.P=x_sol(nu*N+nx*(N+1)+1:end);
% calculate laptime



res.dt = (step_length-step_length.*res.X(1,:).*Track.curv)./(res.X(3,:).*cos(res.X(2,:))-res.X(4,:).*sin(res.X(2,:)));
res.time = [0,cumsum(res.dt(1:N))];
res.laptime = trace(res.dt);
N = Track.N-1;
rt = 0;
for k = 1:N+1
    rt = rt + ((step_length-res.X(1,k)*Track.curv(k))/(res.X(3,k)*cos(res.X(2,k))-res.X(4,k)*sin(res.X(2,k))));
end

res.Fz=[res.U(8,:);
        res.U(9,:);
        res.U(10,:);
        res.U(11,:)];

res.SA=[res.U(32,:);
        res.U(33,:);
        res.U(34,:);
        res.U(35,:)];

res.Fx=[res.U(12,:);
        res.U(13,:);
        res.U(14,:);
        res.U(15,:)];

res.Fy=[res.U(16,:);
        res.U(17,:);
        res.U(18,:);
        res.U(19,:)];

res.beta=atan(res.X(4,:)./res.X(3,:));  %rad

for i=1:N
 res.J(1,i)=-ms*hs/(2*L*kf)*res.U(6,i)-ms*hr*rollratiof/(wbf*kf+2*krf/wbf)*res.U(7,i);
 res.J(2,i)=-ms*hs/(2*L*kf)*res.U(6,i)+ms*hr*rollratiof/(wbf*kf+2*krf/wbf)*res.U(7,i);
 res.J(3,i)= ms*hs/(2*L*kr)*res.U(6,i)-ms*hr*rollratior/(wbr*kf+2*krr/wbr)*res.U(7,i);
 res.J(4,i)= ms*hs/(2*L*kr)*res.U(6,i)+ms*hr*rollratior/(wbr*kf+2*krr/wbr)*res.U(7,i);
end
%camber计算

res.camber(1,:)= afl + (c10fl + c11fl.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:))./9.8./1.453 + (c20fl + c21fl.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:)).*abs(res.U(7,:))./9.8./9.8./1.453./1.453   + (c30fl + c31fl.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:)).*abs(res.U(7,:)).*abs(res.U(7,:))./9.8 ./9.8 ./9.8./1.453./1.453./1.453 ;
res.camber(2,:)= afr + (c10fr + c11fr.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:))./9.8./1.453  + (c20fr + c21fr.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:)).*abs(res.U(7,:))./9.8./9.8./1.453./1.453   + (c30fr + c31fr.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:)).*abs(res.U(7,:)).*abs(res.U(7,:))./9.8 ./9.8 ./9.8./1.453./1.453./1.453 ;
res.camber(3,:)= arl + (c10rl + c11rl.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:))./9.8./1.453  + (c20rl + c21rl.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:)).*abs(res.U(7,:))./9.8./9.8./1.453./1.453   + (c30rl + c31rl.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:)).*abs(res.U(7,:)).*abs(res.U(7,:))./9.8 ./9.8 ./9.8./1.453./1.453./1.453 ;
res.camber(4,:)= arr + (c10rr + c11rr.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:))./9.8./1.453  + (c20rr + c21rr.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:)).*abs(res.U(7,:))./9.8./9.8./1.453./1.453   + (c30rr + c31rr.*sign(res.U(6,:)./9.8./1.453)).*abs(res.U(7,:)).*abs(res.U(7,:)).*abs(res.U(7,:))./9.8 ./9.8 ./9.8./1.453./1.453./1.453 ;


%% plots

%% path
x=[];
y=[];
for i=1:Track.N
    x(i)=Track.x(i)-sin(Track.psi(i)).*res.X(1,i);
    y(i)=Track.y(i)+cos(Track.psi(i)).*res.X(1,i);
end

%%图一赛车路径
figure('Color','w');
%plot(crt(:,3),-crt(:,9)*0.99,'-r','LineWidth',1);hold on;
plot(x,y,'-.b','LineWidth',2);hold on;
plot(Track.x,Track.y,'--g','LineWidth',2);hold on;
plot(Track.x-sin(Track.psi)*trackwidth,Track.y+cos(Track.psi)*trackwidth,'k','LineWidth',2);hold on;
plot(Track.x+sin(Track.psi)*trackwidth,Track.y-cos(Track.psi)*trackwidth,'k','LineWidth',2);grid on;
xlabel('x (m)','FontSize',20),ylabel('y (m)','FontSize',20);
legend({'车辆路径','赛道中心线'},'FontSize',20);
set(gca,'FontSize',16);
daspect([1,1,1])

%% velocities图二
figure('Color','w');
plot(Track.S,res.X(3,:),'-','LineWidth',2);%hold on;
%plot(crt(:,1)*1.01,crt(:,2),':','LineWidth',3);grid on;
xlabel('运动距离(m)','FontSize',20);ylabel('速度 (m/s)','FontSize',20);
set(gca,'FontSize',16);
legend({'仿真数据'},'FontSize',20);

% normal path deviation图三



figure('Color','w');
plot(Track.S,Track.curv,'r','LineWidth',2);hold on;
plot(Track.S,res.X(1,:),'b','LineWidth',2);grid on;
xlabel('distance (m)','FontSize',20),ylabel('车辆与道路中心线距离 (m)','FontSize',2);
legend({'曲率','与中心线距离'},'FontSize',20);
set(gca,'FontSize',16);
%% steering angle 图四
figure('Color','w');
plot(Track.S(1:length(res.U(6,:))),res.U(1,:)*180/pi,'LineWidth',2);grid on;
xlabel('distance(m)','FontSize',14),ylabel('方向盘转角 (deg)','FontSize',14);

%% longitudinal slip图五
figure('Color','w');
plot(Track.S(1:length(res.U(2,:))),res.U(2,:),'LineWidth',2);hold on;
plot(Track.S(1:length(res.U(3,:))),res.U(3,:),'LineWidth',2);grid on;
plot(Track.S(1:length(res.U(4,:))),res.U(4,:),'LineWidth',2);grid on;
plot(Track.S(1:length(res.U(5,:))),res.U(5,:),'LineWidth',2);grid on;
xlabel('distance (m)','FontSize',14),ylabel('longitudinal slip','FontSize',14);
legend({'左前轮','右前轮','左后轮','右后轮'},'FontSize',14);



%% 轮胎垂向载荷图六
figure('Color','w');
plot(Track.S(1:length(res.U(8,:))),res.U(8,:),'LineWidth',2);hold on;
plot(Track.S(1:length(res.U(9,:))),res.U(9,:),'LineWidth',2);hold on;
plot(Track.S(1:length(res.U(10,:))),res.U(10,:),'LineWidth',2);hold on;
plot(Track.S(1:length(res.U(11,:))),res.U(11,:),'LineWidth',2);hold on;
acc=(res.U(8,:).*wbf/2-res.U(9,:).*wbf./2)-(res.U(10,:).*wbr./2-res.U(11,:).*wbr./2);
plot(Track.S(1:length(res.U(8,:))),acc,'LineWidth',1);grid on;
xlabel('distance (m)','FontSize',14),ylabel('车轮垂向力(N)','FontSize',14);
legend({'左前轮','右前轮','左后轮','右后轮','前后侧倾力矩差'},'FontSize',14);


%%质心侧偏角图七

figure('Color','w');

%plot(Track.S(1:length(res.U(2,:))),res.U(8,:),'b','LineWidth',1);grid on;
plot(Track.S,atan(res.X(4,:)./res.X(3,:))*180/pi,'b','LineWidth',2);
xlabel('distance (m)','FontSize',20),ylabel('质心侧偏角(deg)','FontSize',2);
% legend({'主动载荷转移','质心侧偏角'},'FontSize',20);
set(gca,'FontSize',16);




%%横摆角速度图八
figure('Color','w');
%plot(Track.S(1:length(res.U(2,:))),res.U(8,:),'b','LineWidth',1);grid on;
plot(Track.S,res.X(5,:),'b','LineWidth',2);
xlabel('distance (m)','FontSize',20),ylabel('n (m)','FontSize',2);
legend({'横摆角速度'},'FontSize',20);
set(gca,'FontSize',16);




%%侧向加速度与纵向加速度图9
figure('Color','w');
plot(Track.S(1:length(res.U(7,:))),res.U(7,:),'LineWidth',2);hold on;
plot(Track.S(1:length(res.U(6,:))),res.U(6,:),'LineWidth',2);grid on;
% plot(Track.S(1:length(res.X(2,:))),res.X(2,:),'LineWidth',1);
xlabel('distance (m)','FontSize',14),ylabel('longitudinal slip','FontSize',14);
legend({'侧向','纵向加速度'},'FontSize',14);

%%图10gg图
figure('Color','w');
plot(res.U(7,:),res.U(6,:),'LineWidth',2);
xlabel('侧向加速度ay','FontSize',14),ylabel('纵向加速度ax','FontSize',14);
legend({'g-g图'},'FontSize',14);



%% 前后轮电机输出功率图11
% approximation of power output

figure('Color','w');
plot(Track.S(1:length(res.Power)),res.Power/1000,'LineWidth',2);hold on;
plot(Track.S(1:length(res.Power2)),res.Power2/1000,'LineWidth',2);grid on;
xlabel('行驶距离 (m)','FontSize',14); ylabel('功率(kW)','FontSize',14);
legend({'后轮电机','前轮电机'},'FontSize',14);

%%前后轮车轮内倾角
figure('Color','w');
plot(Track.S(1:length(res.U(2,:))),res.camber(1,:),'LineWidth',2);hold on;
plot(Track.S(1:length(res.U(3,:))),res.camber(2,:),'LineWidth',2);hold on;
plot(Track.S(1:length(res.U(4,:))),res.camber(3,:),'LineWidth',2);hold on;
plot(Track.S(1:length(res.U(5,:))),res.camber(4,:),'LineWidth',2);grid on;
xlabel('distance (m)','FontSize',14),ylabel('主动外倾角','FontSize',14);
legend({'左前轮','右前轮','左后轮','右后轮'},'FontSize',14);


%%前后轮车轮内倾角
figure('Color','w');
scatter(res.U(7,:),res.camber(1,:),'LineWidth',2);hold on;
scatter(res.U(7,:),res.camber(2,:),'LineWidth',2);grid on;
scatter(res.U(7,:),res.camber(3,:),'LineWidth',2);grid on;
scatter(res.U(7,:),res.camber(4,:),'LineWidth',2);grid on;
xlabel('侧向加速度(米每秒)','FontSize',14),ylabel('主动外倾角','FontSize',14);
legend({'左前轮','右前轮','左后轮','右后轮'},'FontSize',14);


%%前后轮车轮外倾角vs方向盘转角
figure('Color','w');

scatter(res.U(1,:),res.camber(1,:),'LineWidth',2);hold on;
scatter(res.U(1,:),res.camber(2,:),'LineWidth',2);grid on;
scatter(res.U(1,:),res.camber(3,:),'LineWidth',2);grid on;
scatter(res.U(1,:),res.camber(4,:),'LineWidth',2);grid on;
xlabel('方向盘转角','FontSize',14),ylabel('车轮外倾角','FontSize',14);
legend({'左前轮','右前轮','左后轮','右后轮'},'FontSize',14);


%%前后轮车轮内倾角
figure('Color','w');
scatter(res.U(6,:),res.camber(1,:),'LineWidth',2);hold on;
scatter(res.U(6,:),res.camber(2,:),'LineWidth',2);grid on;
scatter(res.U(6,:),res.camber(3,:),'LineWidth',2);grid on;
scatter(res.U(6,:),res.camber(4,:),'LineWidth',2);grid on;
xlabel('纵向加速度(米每秒)','FontSize',14),ylabel('主动外倾角','FontSize',14);
legend({'左前轮','右前轮','左后轮','右后轮'},'FontSize',14);

toc


camber = res.camber(1,:); % 侧倾角（-10° 到 10°）
lateral_acc =res.U(7,:); % 侧向加速度（-2g 到 2g，添加噪声）
longitudinal_acc = res.X(3,1:length(res.U(7,:))); % 纵向加速度（-1g 到 1g，添加噪声）

% 创建散点图
figure('Position', [100, 100, 800, 600]);
scatter(lateral_acc,camber,  50, longitudinal_acc, 'filled'); % 点大小50，颜色由纵向加速度决定
colormap('parula'); % 使用parula配色方案
colorbar; % 添加颜色条
c = colorbar;
c.Label.String = 'Longitudinal Acceleration (g)'; % 颜色条标签
c.Label.FontSize = 12;

% 设置图形标签和标题
ylabel('Camber Angle (deg)', 'FontSize', 14);
xlabel('Lateral Acceleration (g)', 'FontSize', 14);
title('Distribution of Camber Angle vs. Lateral Acceleration', 'FontSize', 16);
grid on;

% 设置图形属性
set(gca, 'FontSize', 12, 'LineWidth', 1);
set(gcf, 'Color', 'white');

