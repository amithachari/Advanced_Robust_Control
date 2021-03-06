%
% Stability Analysis at plant Input and Output
% K.A. Wise  29 September 2015
%
% Modified:
%


clear all
close all

format short e

disp('****************************** Program Start ****************');
plot_file_name = 'stab_analysis1.ppt';
save_plots = 0; % Flag to bypass saving

rtd  = 180/pi;
dt = 0.002;
t = 0:dt:2;
w = logspace(-1,3,500);
dd=0.:.001:2*pi;
xx1=cos(dd)-1;yy1=sin(dd);


%%%%%%%%%%%% PLANT DATA %%%%%%%%%%%%%%%%
% Here Za = Za/V  Zd = Zd/V
% Must multiply by V to get accel      %
% Az = V(Za alp + Zd dele)             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       AOA   Za       Zd    Ma       Md
Aero = [16 -1.3046 -0.2142 47.7109 -104.8346;
         6 -0.8757 -0.1531 -68.0209 -74.9210;
        16 -1.2100 -0.1987 44.2506 -97.2313];
%Za = -1.3046; Zd = -0.2142; Ma = -47.7109; Md = -104.8346;

ak=1;
Za = Aero(ak ,2);
Zd = Aero(ak ,3);
Ma = Aero(ak, 4);
Md = Aero(ak, 5);

V = 886.78;%fps
% 2nd order actuator
wa = 40.; za = 0.6;
% 4th order plant model
% Plant States
%     AoA         q         dele      deledot
Ap =[Za/V 1; Ma 0]
Bp =[Zd/V; Md]
Cp=[Za 0; 0  1];
Dp=[Zd; 0];

% 2nd order plant model
[nx,nx]=size(Ap);
%*******************************************
%********* Classical Control ***************
%*******************************************
% Classical autopilot gains
%     AOA   Ka       Kq    az   aq
KK =[ 6.0  -0.001   -0.2   3.0  6.0;
     16.0  -0.0015  -0.32  2.0  6.0];
ak = 2;
Ka = KK(ak,2);
Kq = KK(ak,3);
az = KK(ak,4);
aq = KK(ak,5);

Ac = [0 0; Kq*aq 0];
Bc1 = [-Ka*az 0; -Ka*Kq*aq -Kq*aq];
Bc2 = [Ka*az; Ka*Kq*aq];
%Cc = [2.1598]
Cc = [Kq 1];
Dc1 = [-Ka*Kq -Kq] ;
%Dc1 = [1.3301 0.1700] 
Dc2 = [Ka*Kq];

%****************************************************
% Close the loop
% Plant form  xdot = Apx + Bpu;
%                      y = Cpx +Dpu
% Controller xcdot = Acxc + Bc1y + Bc2r
%                      u = Ccxc + Dc1y + Dc2r
Z = inv(eye(size(Dc1*Dp))-Dc1*Dp);
Acl = [     (Ap+Bp*Z*Dc1*Cp)              (Bp*Z*Cc)
    (Bc1*(Cp+Dp*Z*Dc1*Cp))  (Ac+Bc1*Dp*Z*Cc)];
Bcl = [       Bp*Z*Dc2
    (Bc2+Bc1*Dp*Z*Dc2)];
Ccl = [(Cp+Dp*Z*Dc1*Cp) (Dp*Z*Cc)];
Dcl =(Dp*Z*Dc2);
Ccl_Az = Ccl(1,:);
Dcl_Az =Dcl(1,:);
sys_cl = ss(Acl,Bcl,Ccl,Dcl);



% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Closed-loop stability check 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if max(real(eig(Acl))) > 0,
  disp('Closed-Loop System is Unstable');
  disp(['  Most unstable eig = ', num2str(max(real(eig(Acl))))]);
  return
else
  disp('Closed-Loop System is stable');
  disp(['  Dominant Re(eig) = ', num2str(max(real(eig(Acl))))]);
end;



% Step Response Time Domain Analysis - this confirms that the system is
% connected properly and is stable
[y_HS,x_HS] = step(Acl,Bcl,Ccl_Az,Dcl_Az,1,t);
sys_HS = ss(Acl,Bcl,Ccl_Az,Dcl_Az);

figure('Name','LQR Step Sim'),
plot(t,y_HS,'LineWidth',2);grid
legend('Az','Location','Best');
xlabel('Time (sec)')
ylabel('Az fps2')
if(save_plots == 1) saveppt2(plot_file_name); end

%SS model of loop gain Lu at the plant input
Ain = [ Ap 0.*Bp*Cc;  Bc1*Cp Ac];
Bin = [ Bp; Bc1*Dp];
Cin = -[ Dc1*Cp Cc];%change sign for loop gain
Din = -[ Dc1*Dp];
sys_u = ss(Ain,Bin,Cin,Din);

%SS model of loop gain L at the plant output
Aout = [ Ap Bp*Cc;  0.*Bc1*Cp Ac];
Bout = [ Bp*Dc1; Bc1];
Cout = -[ Cp Dp*Cc];%change sign for loop gain
Dout = -[ Dp*Dc1];
sys_y = ss(Aout,Bout,Cout,Dout);
%nyquist(sys_u)
Lu = freqresp(sys_u,w);
nyquist(sys_u)
Ly = freqresp(sys_y,w);
T  = freqresp(sys_cl,w);
S = 1 - T;
%nyquist(Lu)
[nCp,nAp] = size(Cp);
[~,nBp]   = size(Bp);

% Plant Input Freqeuncy Domain Analysis
for i=1:numel(w),
    s = sqrt(-1)*w(i);
    GG = Cp*inv(s*eye(size(Ap))-Ap)*Bp+Dp;
    KK = Cc*inv(s*eye(size(Ac))-Ac)*Bc1+Dc1;
    Lu_HS(i)  = -KK*GG;
    RDu_HS(i)  = 1. + Lu_HS(i);
    SRu_HS(i) = 1. + 1./Lu_HS(i);
    Lin = Cin*inv(s*eye(size(Ain))-Ain)*Bin+Din;
    Lout = Cout*inv(s*eye(size(Aout))-Aout)*Bout+Dout;
    for jj = 1:nBp,
        Fyjj = eye(size(Lin));
        Fyjj(jj,jj) = 0.;
        Tujj(jj,i) = inv(eye(size(Lin)) + Lin*Fyjj)*Lin;
    end
    for jj = 1:nCp,
        Fyjj = eye(size(Lout));
        Fyjj(jj,jj) = 0.;
        Tyjj = inv(eye(size(Lout)) + Lout*Fyjj)*Lout;
        Tyj(jj,i) = Tyjj(jj,jj);
    end
    Sout = inv(eye(size(Lout))+Lout);
    Tout = Sout*Lout;
    Tout2 = Lout*Sout;
    sens_HS(i) = max(Sout(1,1));
    compsens_HS11(i) = max(Tout(1,1));
    compsens_HS12(i) = max(Tout2(1,1));
end


figure('Name','Nyquist Plot at Plant Input'),
plot(xx1,yy1,'r',real(squeeze(Lu)),imag(squeeze(Lu)),'k',real(Lu_HS),imag(Lu_HS),'r--',...
real(Tujj),imag(Tujj),'g--','LineWidth',2);grid
axis([-2 2 -2 2]);
legend('Unit Circle','Loop','Short Form','Location','Best');
xlabel('Re(L)')
ylabel('Im(L)')
title('Nyquist Plot at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Nyquist Plot at Plant Output'),
hold on
for jj = 1:nCp,
plot(xx1,yy1,'r',real(Tyj(jj,:)),imag(Tyj(jj,:)),'b--','LineWidth',2);grid
axis([-2 2 -2 2]);
xlabel('Re(L)')
ylabel('Im(L)')
title('Nyquist Tyj at Plant Output')
if(save_plots == 1) saveppt2(plot_file_name); end
end
hold off

for jj =1:nCp,
    figure('Name','Nyquist Plot at Plant Output'),
    plot(xx1,yy1,'r',real(Tyj(jj,:)),imag(Tyj(jj,:)),'r--','LineWidth',2);grid
    legend([num2str(jj) ' Nyquist of Tyj'],'Location','Best');
    axis([-2 2 -2 2]);
    title('Nyquist Tyj at Plant Output')
    xlabel('Re(L)')
    ylabel('Im(L)')
if(save_plots == 1) saveppt2(plot_file_name); end
end

for jj=1:nCp,
    figure('Name','RD at Plant Output'),
    semilogx(w,20*log10(abs(1.+Tyj(jj,:))),'b--','LineWidth',2);grid
    v_min = min(min(abs(1.+Tyj(jj,:))));
    legend([num2str(jj) ' min(1.+Tyj) = ' num2str(v_min)],'Location','Best');
    xlabel('Frequency (rps)')
    ylabel('Mag')
    title('RD 1.+Tyj at Plant Output')
if(save_plots == 1) saveppt2(plot_file_name); end
end

% Compute LGCF
bode_options                = bodeoptions;
% bode_options.FreqUnits      = 'Hz'; 
bode_options.PhaseWrapping  = 'on';
[mag,pha] = bode(sys_u,w);

[GM, PM_deg, wc_GM, wc_Pm] = margin(sys_u);

figure('Name','Bode Magnitude at Plant Input'),
semilogx(w,20*log10(abs(Lu_HS)),'k',w,20*log10(squeeze(mag)),'r--','LineWidth',2);grid
legend('Loop','Short Form','Location','Best');
xlabel('Frequency (rps)')
ylabel('Mag')
title('Bode Magnitude at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Bode Phase at Plant Input'),
semilogx(w,squeeze(pha),'k','LineWidth',2);grid
legend('Loop','Short Form','Location','Best');
xlabel('Frequency (rps)')
ylabel('Phase (deg)')
title('Bode Phase at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Return Difference at Plant Input'),
semilogx(w,20*log10(abs(RDu_HS)),'r--','LineWidth',2);grid
    RDu_min = min(min(abs(RDu_HS)));
    legend([num2str(jj) ' min(I+Lu) = ' num2str(RDu_min)],'Location','Best');
xlabel('Frequency (rps)')
ylabel('Mag dB')
title('Return Difference at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Stability Robustness at Plant Input'),
semilogx(w,20*log10(abs(SRu_HS)),'r--','LineWidth',2);grid
    SRu_min = min(min(abs(SRu_HS)));
    legend([num2str(jj) ' min(I+invLu) = ' num2str(SRu_min)],'Location','Best');
xlabel('Frequency (rps)')
ylabel('Mag dB')
title('Stability Robustness at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end

% Return Difference at plant output
sv_rd_y = sigma(sys_y,w,2);
min_sv_rd_y = sv_rd_y(nCp,:);
min_rd_y = min(abs(min_sv_rd_y));

figure('Name','min SV of RDM at Plant Output'),
semilogx(w,20*log10(abs(min_sv_rd_y)),'k','LineWidth',2);grid
legend(['min(sv(I+Ly)) = ' num2str(min_rd_y)],'Location','Best');
xlabel('Frequency (rps)')
ylabel('Mag dB')
title('Stability Robustness at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end



for jj = 1:nCp,
    figure('Name','Comp Sensitivity at Output'),
    semilogx(w,20*log10(abs(squeeze(T(jj,1,:)))),'b','LineWidth',2);grid
    v_min = max(max(abs(squeeze(T(jj,1,:)))));
    legend([num2str(jj) ' T max = ' num2str(v_min)],'Location','Best');
    %legend('T-Az LQR SF','T-Az LQG OF','T-Az LQR OFW','Location','Best');
    xlabel('Frequency (rps)')
    ylabel('Mag (dB)')
    title('Comp Sensitivity at Output')
end
if(save_plots == 1) saveppt2(plot_file_name); end

for jj = 1:nCp,
    figure('Name','Sensitivity at Output'),
    semilogx(w,20*log10(abs(squeeze(S(jj,1,:)))),'b','LineWidth',2);grid
    v_min = max(max(abs(squeeze(S(jj,1,:)))));
    legend([num2str(jj) ' T max = ' num2str(v_min)],'Location','Best');
    %legend('T-Az LQR SF','T-Az LQG OF','T-Az LQR OFW','Location','Best');
    xlabel('Frequency (rps)')
    ylabel('Mag (dB)')
    title('Sensitivity')
end
if(save_plots == 1) saveppt2(plot_file_name); end

disp('Classical Margins')
allmargin(sys_u)

disp('  ')
disp('SV Margins')
RDu_nGM = 1/(1+RDu_min);
RDu_pGM = 1/(1-RDu_min);
RDu_Pha = 2*asin(RDu_min/2);
RDu_nGM_dB = 20*log10(RDu_nGM);
RDu_pGM_dB = 20*log10(RDu_pGM);
RDu_Pha_deg = 180*RDu_Pha/pi ;
disp('RDu_nGM RDu_pGM RDu_Pha')
disp([num2str(RDu_nGM) ' ' num2str(RDu_pGM) ' ' num2str(RDu_Pha)])
disp([num2str(RDu_nGM_dB) ' ' num2str(RDu_pGM_dB) ' ' num2str(RDu_Pha_deg)])
SRu_nGM = 1-SRu_min;
SRu_pGM = 1+SRu_min;
SRu_Pha = 2*asin(SRu_min/2);
SRu_nGM_dB = 20*log10(SRu_nGM);
SRu_pGM_dB = 20*log10(SRu_pGM);
SRu_Pha_deg = 180*SRu_Pha/pi ;
disp('SRu_nGM SRu_pGM SRu_Pha')
disp([num2str(SRu_nGM) ' ' num2str(SRu_pGM) ' ' num2str(SRu_Pha)])
disp([num2str(SRu_nGM_dB) ' ' num2str(SRu_pGM_dB) ' ' num2str(SRu_Pha_deg)])
disp('  ')


figure('Name','Nyquist Plot at Plant Input'),
plot(xx1,yy1,'r',real(squeeze(Lu)),imag(squeeze(Lu)),'k',real(Lu_HS),imag(Lu_HS),'r--',...
real(Tujj),imag(Tujj),'g--','LineWidth',2);grid
legend('Unit Circle','Loop','Short Form','Location','Best');
xlabel('Re(L)')
ylabel('Im(L)')
title('Nyquist Plot at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end


% SISO loop gains at output
for k = 1:nCp
    % Close other loops for SISO margins
    idx_closed = setxor(k,1:nCp);
    disp(['Output = ' num2str(k)])
    Ly_k = feedback(sys_y,eye(size(sys_y,1)-1),idx_closed,idx_closed);
    [output_GM_design(k), output_PM_design(k), ~ , output_wcp_design(k)] = margin(Ly_k(k,k))
    figure
    margin(Ly_k(k,k))
end


margin(sys_u)

