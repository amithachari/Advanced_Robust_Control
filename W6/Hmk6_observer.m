% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Purpose:
%   This file sets up the pitch matrices 
%   and designs a Hinf SF Accel Cmd Tracking autopilot
%
% Created : 2/8/2017, Kevin A Wise
%
% Modified:
% 2/26/17 added plotting of RSLQR
% 2/4/2020 added noise to control and control rate plotting
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
clc
clear all
close all
format short e

disp('****************************** Homework_4_HINF Program Start ****************');
plot_file_name = 'Homework_6_2022.ppt';
save_plots = 0; % Flag to bypass saving plots
w = logspace(-3,4,1000);
% t = linspace(0,2.5,1000);
dt = 0.002;
t = 0:dt:2;
dd=0.:.001:2*pi;
xx1=cos(dd)-1;yy1=sin(dd);
rtd = 180/pi;

g    = 32.174;

m2ft = 3.2808;    % meters to feet conversion
ft2m = 1/3.2808;  % feet to meters conversion

% Public release airframe parameters
%*************************************************************************
% Plant Model
%*************************************************************************
% State Names 
%  ----------- 
%  AZ fps2         
%  q rps           
%  Dele rad        
%  Dele dot rps    
 
%  Input Names
%  -----------
%  Dele cmd deg    
% Plant.Ap = [ -0.576007     -3255.07            4.88557       9.25796;
%           -0.0410072       -0.488642       -2.03681       0      ;
%                  0                0               0             1      ;
%                  0                0           -8882.64       -133.266  ];
% Plant.Bp = [      0  ;  0   ;     0   ;  8882.64];
% Plant.Cp = [1 0 0 0];
% Plant.Dp = 0.*Plant.Cp*Plant.Bp;
% pstatnam = {'Az fps2' 'q rps' 'dele rad' 'deledot rps'};
% poutnam  = {'Az fps2'};
% pinnam   = {'delecmd rad'};
% plant=ss(Plant.Ap,Plant.Bp,Plant.Cp,Plant.Dp,'statename',pstatnam,'inputname',pinnam,'outputname',poutnam);

%**************************************************************************
% Aircraft Model Data
%**************************************************************************
% Model (taken from example 5.2, the aircraft pitch axis plant data)
% Aero constants used in the aircraft model:
Za_V = -1.3046;
Ma   = 47.711; % Positve Ma = unstable aircraft
Zd_V = -.2142;
Md   = -104.83;
V    = 886.78; % (fps)
Za   = V*Za_V;
Zd   = V*Zd_V;
w_act = 2.*pi*11; % actuator natural frequency rps
z_act = 0.707;    % actuator damping

grav = 32.174; % (fps2)

%**************************************************************************
% Plant model for analysis with actuator 
%**************************************************************************
% States are AOA (rad),  pitch rate (rps), dele (rad), deledot (rps)
disp('Plant Model')
disp('xpdot = Ap*xp + Bp*u')
disp('    y = Cp*xp + Dp*u')
disp('   xp = AOA (rad), pitch rate q (rps), dele (rad), deledot (rps)')
disp('    u =  delec (rad)')
disp('    y =  Az (fps2), AOA (rad), pitch rate q (rps), dele (rad), deledot (rps)')

disp(' ')

Ap = [Za_V  1.      Zd_V          0.; 
      Ma    0.       Md           0.;
      0.    0.       0.           1.;
      0.    0. -w_act*w_act -2*z_act*w_act];

% Input is dele (rad)
Bp = [0.; 0.; 0.; w_act*w_act ];

% Outputs are Az (fps2), AOA (rad), pitch rate (rps), dele (rad), deledot (rps)
Cp = [  Za   0.  Zd  0. ];
Dp = [0.*Cp*Bp];

Ap =[Za_V   1.  Zd_V  0.;
     Ma     0.  Md  0;
     0.     0.  0.  1.;
     0.     0.  -w_act*w_act -2*z_act*w_act]
 
Bp =[0.; 0.; 0.; w_act*w_act]

Cp=[Za 0. Zd 0.;
    eye(4)]

Dp=[ 0*Cp*Bp]

pout(1,:) = 'Az (fps2)';
pout(2,:) = 'AOA (rad)';
pout(3,:) = 'q   (rps)';
pout(4,:) = 'de  (rad)';
pout(5,:) = 'dedt(rps)';

[nCp, nAp] = size(Cp);
[~, nBp] = size(Bp);

%Wiggle Model
Aw = [0. Za   0.;
      0. Za_V 1.;
      0. Ma   0.]
Bw = [Zd;
      Zd_V;
      Md]
  
%LQR Setup
Q = 0.*Aw;
R = 1;
xeig = [];
LQReig = [];

qq=logspace(-6,0,100);

xopenloop = eig(Aw);

% Observer
Hmk3_RSLQR = load('Hmk3_RSLQR.mat');
K_lqr = Hmk3_RSLQR.Kx_lqr;

C = [1. 0. 0.;
     0. 0. 1];
Q0 = diag([0.001, 0.0014, 0.005]);
R0 = 1000*diag([0.025^2, 0.001^2]);
rho = 10^10;

Q = Q0 + (1/rho)*Bw*Bw';
R = R0;
G = eye(3);
[L,P,E] = lqe(Aw,G,C,Q,R)

Aobs_cl = (Aw - Bw*K_lqr - L*C);

%controller
Ac = [0. 0. 0. 0.;
      L(:,1) Aobs_cl];
% zeros = [0. 0.;
%          0. 0.;
%          0. 0.];
Bc1 = [1. 0. 0. 0. 0.;
       zeros(3,2) L(:,2) zeros(3,2)];
Bc2 = [-1. -1. 0. 0.]'
Cc = [0. -K_lqr];
Dc1 = [0. 0. 0. 0. 0.];
Dc2 = 0;

% Form the closed loop system
Z = inv(eye(size(Dc1*Dp))-Dc1*Dp);
Acl = [     (Ap+Bp*Z*Dc1*Cp)              (Bp*Z*Cc)
    (Bc1*(Cp+Dp*Z*Dc1*Cp))  (Ac+Bc1*Dp*Z*Cc)];
Bcl = [       Bp*Z*Dc2
    (Bc2+Bc1*Dp*Z*Dc2)];
Ccl = [(Cp+Dp*Z*Dc1*Cp) (Dp*Z*Cc)];
Dcl =(Dp*Z*Dc2);
sys_cl = ss(Acl,Bcl,Ccl,Dcl);

y = step(sys_cl,t);
az = y(:,1); %  acceleration (fps2)
aze = abs(ones(size(az))-az);  % error for az
taur = 0.; taus= 0.; % rise time and settling time
fv = aze(numel(aze)); % final value of the error
e_n = aze - fv*ones(size(aze)) - 0.36*ones(size(aze));
e_n1 = abs(e_n) + e_n;
taur = crosst(e_n1,t); % rise time 
e_n = aze - fv*ones(size(aze)) - 0.05*ones(size(aze));
e_n1 = abs(e_n) + e_n;
taus = crosst(e_n1,t); % settling time

disp('Closed Loop Eigenvalues')
damp(Acl)
[V,D] = eig(Acl);
disp('Closed Loop Eigenvectors')
V
disp('Closed Loop Eigenvalues')
diag(D)


% SS model of loop gain at the plant input Lu
     A_Lu = [ Ap 0.*Bp*Cc;  Bc1*Cp Ac];
     B_Lu = [ Bp; Bc1*Dp];
     C_Lu = -[ Dc1*Cp Cc];%change sign for loop gain
     D_Lu = -[ Dc1*Dp];
     sys_Lu = ss(A_Lu,B_Lu,C_Lu,D_Lu);
%SS model of loop gain Ly at the plant output
     Aout = [ Ap Bp*Cc;  0.*Bc1*Cp Ac];
     Bout = [ Bp*Dc1; Bc1];
     Cout = -[ Cp Dp*Cc];%change sign for loop gain
     Dout = -[ Dp*Dc1];
     sys_Ly = ss(Aout,Bout,Cout,Dout);
% Analysis at plant inut
     magdb = 20*log10(abs(squeeze(freqresp(sys_Lu,w))));
     wc = crosst(magdb,w); % LGCF, assumes Lu is a scalar
     sr = sigma(sys_Lu,w,3);
     sru_min = min(abs(sr));
     rd = sigma(sys_Lu,w,2);
     rdu_min = min(abs(rd));
     Lu = squeeze(freqresp(sys_Lu,w));
% Analysis at plant output
     T  = freqresp(sys_cl,w); % Complementary Sensitivity
     S = 1 - T; % Sensitivity
     T_Az = 20*log10(abs(squeeze(T(1,1,:))));
     S_Az = 20*log10(abs(squeeze(S(1,1,:))));
     Tmax = max(T_Az); % Inf Norm of T in dB
     Smax = max(S_Az); % Inf Norm of S in dB

%**************************************************************************
% This next set of code computes SISO margins at output with 
     for i=1:numel(w),
         s = sqrt(-1)*w(i);
         Lout = Cout*inv(s*eye(size(Aout))-Aout)*Bout+Dout;
         % This loops computes SISO freq response at plant output one loop open,
         % rest closed
         for jj = 1:nCp,
             Fyjj = eye(nCp);
             Fyjj(jj,jj) = 0.;
             Tyjj = inv(eye(nCp) + Lout*Fyjj)*Lout;
             Tyj(jj,i) = Tyjj(jj,jj);
         end
     end
     for jj=1:nCp,
         v_min(jj) = min(min(abs(1.+Tyj(jj,:))));
     end

%
figure
margin(sys_Lu);grid
if(save_plots == 1) saveppt2(plot_file_name); end
%[Gm,Pm,Wcg,Wcp] = margin(SYS) 
[Gm,Pm,Wcg,Wcp] = margin(sys_Lu);


 disp(' ')
disp('Homework 4 Hinf SF')
    %Compute singluar value margins
     neg_gm =  min([ (1/(1+rdu_min)) (1-sru_min)]); % in dB
     pos_gm =  max([ (1/(1-rdu_min)) (1+sru_min)]); % in dB
     neg_gmdB = 20*log10( neg_gm ); % in dB
     pos_gmdB = 20*log10( pos_gm ); % in dB
     pm = 180*(max([2*asin(rdu_min/2) 2*asin(sru_min/2)]))/pi;% in deg

     disp('Singular value margins')
     disp(['Min Singular value I+Lu =    ' num2str(rdu_min)])
     disp(['Min Singular value I+invLu = ' num2str(sru_min)])
     disp(['Singular value gain margins = [' ...
           num2str(neg_gmdB) ' dB,' num2str(pos_gmdB) ' dB ]' ])
     disp(['Singular value phase margins = [ +/-' ...
           num2str(pm)  ' deg ]' ])
       
       
     y = step(sys_cl,t);
     az = y(:,1); %  acceleration (fps2)
     aze = abs(ones(size(az))-az);  % error for az
     taur = 0.; taus= 0.; % rise time and settling time
     fv = aze(numel(aze)); % final value of the error
     e_n = aze - fv*ones(size(aze)) - 0.36*ones(size(aze));
     e_n1 = abs(e_n) + e_n;
     taur = crosst(e_n1,t); % rise time 
     e_n = aze - fv*ones(size(aze)) - 0.05*ones(size(aze));
     e_n1 = abs(e_n) + e_n;
     taus = crosst(e_n1,t); % settling time
     azmin = abs(min(az))*100; % undershoot
     azmax = (abs(max(az))-1)*100; % overshoot
     dmax = max(abs(y(:,3)))*rtd*grav; % compute in per g commanded
     ddmax = max(abs(y(:,4)))*rtd*grav;
     metric=[wc rdu_min sru_min wc taur taus azmin azmax dmax ddmax Tmax Smax v_min];
     
 % Compute the noise-to-control TF
 Bv = [       Bp*Z*Dc1;
     (Bc1+Bc1*Dp*Z*Dc1)];
 Cv  = [ Z*Dc1*Cp Z*Cc];
 Cvv = [ Cv ; Cv*Acl ];
 Dv = Z*Dc1;
 Dvv = [ Dv; Cv*Bv];
 sys_noise = ss(Acl,Bv,Cvv,Dvv);
 v_2_u  = freqresp(sys_noise,w); % Noise to control freq response
 dele_Az    = 20*log10(abs(squeeze(v_2_u(1,1,:))));
 dele_q     = 20*log10(abs(squeeze(v_2_u(1,3,:))));
 deledot_Az = 20*log10(abs(squeeze(v_2_u(2,1,:))));
 deledot_q  = 20*log10(abs(squeeze(v_2_u(2,3,:))));

