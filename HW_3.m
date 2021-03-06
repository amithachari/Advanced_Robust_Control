clear all
close all

format short e

disp('****************************** Program Start ****************');
plot_file_name = 'stab_analysis1.ppt';
save_plots = 0; % Flag to bypass saving

V = 886.78;%fps
rtd  = 180/pi;
dt = 0.002;
t = 0:dt:2;
w = logspace(-1,3,500);
dd=0.:.001:2*pi;
xx1=cos(dd)-1;yy1=sin(dd);

%Plant Model
Ma = 47.711;
Za_V = -1.3046;
Zd_V = -0.2142;
Md = -104.83;
Za = V*Za_V;
Zd = V*Zd_V;
w_act = 2.*pi*11;
z_act = 0.707;
grav = 32.174;

Ap =[Za_V   1.  Zd_V  0.;
     Ma     0.  Md  0;
     0.     0.  0.  1.;
     0.     0.  -w_act*w_act -2*z_act*w_act]
Bp =[0.; 0.; 0.; w_act*w_act]
Cp=[Za 0. Zd 0.;
    eye(4)]
Dp=[ 0*Cp*Bp]

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

az_st = 0.* ones(numel(qq), numel(t));
del_st = 0.* ones(numel(qq), numel(t));
deldot_st = 0.* ones(numel(qq), numel(t));
T_st = 0.* ones(numel(qq), numel(w));
S_st = 0.* ones(numel(qq), numel(w));



npts = numel(qq);
ii = 1;
i_cc = 0;

while ii < npts,
    i_cc = ii -1;
    Q(1,1) = qq(ii);
    [Kx_lqr, ~, ~] = lqr(Aw,Bw,Q,R);
    
    Ac = 0.;
    Bc1 = [1. 0. 0. 0. 0.];
    Bc2 = -1;
    Cc = -Kx_lqr(1);
    Dc1 = [0. -Kx_lqr(2:3) 0. 0.];
    Dc2 = 0.;
    
    Z = inv(eye(size(Dc1*Dp))-Dc1*Dp);
    Acl = [     (Ap+Bp*Z*Dc1*Cp)              (Bp*Z*Cc)
        (Bc1*(Cp+Dp*Z*Dc1*Cp))  (Ac+Bc1*Dp*Z*Cc)];
    Bcl = [       Bp*Z*Dc2
        (Bc2+Bc1*Dp*Z*Dc2)];
    Ccl = [(Cp+Dp*Z*Dc1*Cp) (Dp*Z*Cc)];
    Dcl =(Dp*Z*Dc2);
    sys_cl = ss(Acl,Bcl,Ccl,Dcl);
    
    if max(real(eig(Acl))) > 0,
      disp('Closed-Loop System is Unstable');
      disp(['  Most unstable eig = ', num2str(max(real(eig(Acl))))]);
      return
    end;
    
    xx = eig(Acl);
    xeig = [xeig;xx];
    xx = eig(Aw - Bw*Kx_lqr);
    LQReig = [LQReig;xx];
    
    % Frequency Domain Analysis
    % SS model of loop gain at the plant input Lu
    A_Lu = [ Ap 0.*Bp*Cc; Bc1*Cp Ac];
    B_Lu = [ Bp; Bc1*Dp];
    C_Lu = -[ Dc1*Cp Cc];%change sign for loop gain
    D_Lu = -[ Dc1*Dp];
    sys_Lu = ss(A_Lu,B_Lu,C_Lu,D_Lu);
    
    % SS model of loop gain at the plant input Ly
    Aout = [ Ap Bp*Cc; 0.*Bc1*Cp Ac];
    Bout = [ Bp*Dc1; Bc1];
    Cout = -[ Cp Dp*Cc];%change sign for loop gain
    Dout = -[ Dp*Dc1];
    sys_Ly = ss(Aout,Bout,Cout,Dout);
 %Analysis at Plant Input
    magdb = 20*log10(abs(squeeze(freqresp(sys_Lu,w))));
    wc = crosst(magdb,w); % LGCF, assumes Lu is a scalar
    sr = sigma(sys_Lu,w,3); % Stability Robustness
    sru_min = min(abs(sr));
    rd = sigma(sys_Lu,w,2); % Return Difference
    rdu_min = min(abs(rd));
    
 %Analysis at Plant Output
    T = freqresp(sys_cl,w); % Complementary Sensitivity
    S = 1 - T; % Sensitivity
    T_st(ii,:) = 20*log10(abs(squeeze(T(1,1,:))));
    S_st(ii,:) = 20*log10(abs(squeeze(S(1,1,:))));
    Tmax = max(T_st(ii,:)); % Inf Norm of T in dB
    Smax = max(S_st(ii,:)); % Inf Norm of S in dB
    
    for i=1:numel(w),
        s = sqrt(-1)*w(i);
        Lout = Cout*inv(s*eye(size(Aout)) - Aout)*Bout + Dout;

        for jj = 1:nCp,
            Fyjj = eye(nCp);
            Fyjj(jj,jj) = 0.;
            Tyjj = inv(eye(nCp) + Lout*Fyjj)*Lout;
            Tyj(jj,i) = Tyjj(jj,jj);
        end
    end
    for jj=1:nCp,
        v_min(jj) = min(min(abs(1 + Tyj(jj,:))));
    end
    
    if rdu_min <= 0.3,
        disp('Exit loop on rdmin')
        ip = ii -1;
        ii = npts;
    end
    
    % Time Domain Analysis
    y = step(sys_cl,t);
    az = y(:,1); % acceleration (fps2)
    aze = abs(ones(size(az))-az); % error for az
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
    dmax = max(abs(y(:,4)))*rtd*grav; % compute in per g commanded
    ddmax = max(abs(y(:,5)))*rtd*grav;
    metric=[qq(ii) rdu_min sru_min wc taur taus azmin azmax dmax ddmax Tmax Smax];
    data(ii,:) = metric;
    az_st(ii,:) = az';
    del_st(ii,:) = rtd*y(:,4);
    deldot_st(ii,:) = rtd*y(:,5);
    ii = ii+1

end
    
figure('Name','Accel Time Histories');
plot(t,az_st);
title('Acceleration Step Response');
xlabel('Time (sec)');
ylabel('Az (fps2)');
grid;
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Surface Time Histories');
plot(t,del_st);
title('Surface Deflection to Step Command');
xlabel('Time (sec)');
ylabel('Del (deg)');
grid;
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Surface Rate Time Histories');
plot(t,deldot_st);
title('Surface Rate tp Step Command');
xlabel('Time (sec)');
ylabel('Deldot (deg)');
grid;
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name', 'Comp Sens T');
semilogx(w, T_st);
title('Comp Sens T');
xlabel('Frequency (rps)');
ylabel('Mag (dB)');
grid;
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name', 'Sens S');
semilogx(w, S_st);
title('Sens S');
xlabel('Frequency (rps)');
ylabel('Mag (dB)');
grid;
if(save_plots == 1) saveppt2(plot_file_name); end

%Root Locus Plot
figure('Name', 'LQR Root Locus');
plot(real(xopenloop),imag(xopenloop), 'rd', real(xeig),imag(xeig),'x','Linewidth',2);
title('RSLQR Root Locus');
xlabel('Real');
ylabel('Imaginary');
grid;
if(save_plots == 1) saveppt2(plot_file_name); end


%extract the data from the metric matrix
qm = data(1:i_cc,1); %LQR Penalty
rdm = data(1:i_cc,2); %min Singular Value of RD
srm = data(1:i_cc,3); %min Singular value of SRM
wcm = data(1:i_cc,4); %LGCF
trm = data(1:i_cc,5); %Rise Time
tsm = data(1:i_cc,6); %Settling Time
usm = data(1:i_cc,7); %Undershoot
osm = data(1:i_cc,8); %Overshoot
fam = data(1:i_cc,9); %Max control position from step response
frm = data(1:i_cc,10); %Max fin rate from step response
Tinf = data(1:i_cc,11); %Inf Norm of Comp Sens T
Sinf = data(1:i_cc,12); %Inf Norm of Sens S


err = abs(rdm - srm);
[err_min, i_min] = min(err);
ip = i_min;

%Plot minimum values of the return difference and stability roboustness
%matrix
figure('Name','LQR Design Chart - RDM, SRM vs wc')
semilogx(wcm,rdm,'b-', wcm,srm,'r-','LineWidth',2)
hold on
semilogx(wcm(ip),rdm(ip),'bo', wcm(ip),srm(ip),'ro','LineWidth',2)
xlabel('wc (rps)');
ylabel('min(I + L) min(I + inv(L))');
legend('min RDM Lu', 'min SRM Lu', 'Location', 'Best');
if(save_plots == 1) saveppt2(plot_file_name); end
hold off
grid

%Plot of rise time and settling time
figure('Name','LQR Design Chart - Tr, Ts vs wc')
semilogx(wcm,trm,'b-', wcm,tsm,'r-','LineWidth',2)
hold on
semilogx(wcm(ip),trm(ip),'bo', wcm(ip),tsm(ip),'ro','LineWidth',2)
xlabel('wc (rps)');
ylabel('Tr Ts (sec)');
if(save_plots == 1) saveppt2(plot_file_name); end
hold off
grid

%Plot of overshoot
figure('Name','LQR Design Chart - %OS vs wc')
semilogx(wcm,osm, wcm(ip),osm(ip),'bo','LineWidth',2); grid
xlabel('wc (rps)');
ylabel('% OS');
if(save_plots == 1) saveppt2(plot_file_name); end

%Plot of undershoot and overshoot
figure('Name','LQR Design Chart - %US and %OS vs wc')
semilogx(wcm,usm,'b', wcm,osm,'r','LineWidth',2)
hold on
semilogx(wcm(ip),usm(ip),'bo', wcm(ip),osm(ip),'ro','LineWidth',2)
xlabel('wc (rps)');
ylabel('%US %OS');
if(save_plots == 1) saveppt2(plot_file_name); end
hold off
grid

%Plot of undershoot
figure('Name','LQR Design Chart - %US vs wc')
semilogx(wcm,usm, wcm(ip),usm(ip),'bo','LineWidth',2); grid
xlabel('wc (rps)');
ylabel('% US');
if(save_plots == 1) saveppt2(plot_file_name); end

%Plot of Elevon Rate
figure('Name','LQR Design Chart - Deldot_max vs wc')
semilogx(wcm,frm, wcm(ip),frm(ip),'bo','LineWidth',2); grid
xlabel('wc (rps)');
ylabel('Elevon Rate deg/s/g');
if(save_plots == 1) saveppt2(plot_file_name); end

%Plot of Elevon
figure('Name','LQR Design Chart - Del_max vs wc')
semilogx(wcm,fam, wcm(ip),fam(ip),'bo','LineWidth',2); grid
xlabel('wc (rps)');
ylabel('Elevon deg/g');
if(save_plots == 1) saveppt2(plot_file_name); end


%Plot of the elevon
figure('Name','LQR Design Chart - Tr vs %US')
semilogx(wcm,frm,'b', wcm,fam,'r','LineWidth',2)
hold on
semilogx(wcm(ip),frm(ip),'bo', wcm(ip),fam(ip),'ro','LineWidth',2)
xlabel('wc (rps)');
ylabel('Dele (deg) and Deledot (dps)');
if(save_plots == 1) saveppt2(plot_file_name); end
hold off
grid

%Plot of Elevon
figure('Name','LQR Design Chart - Tr vs %US')
semilogx(usm,trm, usm(ip),trm(ip),'bo','LineWidth',2); grid
xlabel('Tr (sec)');
ylabel('% US');
if(save_plots == 1) saveppt2(plot_file_name); end


