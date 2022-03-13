% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Purpose:
%   This file sets up the pitch matrices 
%   and designs a Hmk6_observer_rho1e2 SF Accel Cmd Tracking autopilot
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
save_plots = 0;
rtd = 180/pi;
dd=0.:.001:2*pi;
xx1=cos(dd)-1;yy1=sin(dd);
w = logspace(-1,3,1000);

m2ft = 3.2808;    % meters to feet conversion
ft2m = 1/3.2808;  % feet to meters conversion

% Load Hmk3 3 data
Hmk3_RSLQR = load('Hmk3_RSLQR.mat');

% Load Hmk6_observer_rho1e2 data
Hmk6_observer_rho1e2 = load('Hmk6_observer_rho1e2.mat');

% Load Hmk5 RSLQR data
Hmk6_observer_rho1e4 = load('Hmk6_observer_rho1e4.mat');

% Load Hmk5 SPC data
Hmk6_observer_rho1e10 = load('Hmk6_observer_rho1e10.mat');


% Scale the linear response by 32.174 for "one gee"
% Plot the time histories
figure('Name','Acceleration Time History')
plot(Hmk3_RSLQR.t, Hmk3_RSLQR.y(:,1),Hmk6_observer_rho1e2.t,Hmk6_observer_rho1e2.y(:,1),'b',Hmk6_observer_rho1e4.t, Hmk6_observer_rho1e4.y(:,1), Hmk6_observer_rho1e10.t, Hmk6_observer_rho1e10.y(:,1),'k','LineWidth',2);grid
legend(['Hmk3 63% Tr = ' num2str(Hmk3_RSLQR.metric(5)) ' 95% Ts = ' num2str(Hmk3_RSLQR.metric(6))],...
       ['Hmk6 observer rho1e2 63% Tr = ' num2str(Hmk6_observer_rho1e2.metric(5)) ' 95% Ts = ' num2str(Hmk6_observer_rho1e2.metric(6))],...
       ['Hmk6 observer rho1e4 63% Tr = ' num2str(Hmk6_observer_rho1e4.metric(5)) ' 95% Ts = ' num2str(Hmk6_observer_rho1e4.metric(6))],...
       ['Hmk6 observer rho1e10 63% Tr = ' num2str(Hmk6_observer_rho1e10.metric(5)) ' 95% Ts = ' num2str(Hmk6_observer_rho1e10.metric(6))],'Location','Best');
xlabel('Time (sec)');
ylabel('Az (fps2)');
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Pitch Rate Time History')
plot(Hmk3_RSLQR.t, Hmk3_RSLQR.y(:,3)*rtd,Hmk6_observer_rho1e2.t,Hmk6_observer_rho1e2.y(:,3)*rtd,'b',Hmk6_observer_rho1e4.t, Hmk6_observer_rho1e4.y(:,3)*rtd, 'k',Hmk6_observer_rho1e10.t, Hmk6_observer_rho1e10.y(:,3)*rtd,'LineWidth',2);grid
legend('Hmk3 ', ...
       'Hmk6 observer rho1e2 ',...
       'Hmk6 observer rho1e4 ',...
       'Hmk6 observer rho1e10 ',...
       'Location','Best');
xlabel('Time (sec)');
ylabel('Pitch Rate (dps)');
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Elevon Time History')
plot(Hmk3_RSLQR.t, Hmk3_RSLQR.y(:,4)*rtd, Hmk6_observer_rho1e2.t,Hmk6_observer_rho1e2.y(:,4)*rtd,'b',Hmk6_observer_rho1e4.t, Hmk6_observer_rho1e4.y(:,4)*rtd,'k',Hmk6_observer_rho1e10.t,Hmk6_observer_rho1e10.y(:,4)*rtd,'LineWidth',2);grid
legend('Hmk3 ', ...
       'Hmk6 observer rho1e2 ',...
       'Hmk6 observer rho1e4 ',...
       'Hmk6 observer rho1e10 ',...
       'Location','Best');
xlabel('Time (sec)');
ylabel('Elevon (deg)');
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Elevon Rate Time History')
plot(Hmk3_RSLQR.t, Hmk3_RSLQR.y(:,5)*rtd,Hmk6_observer_rho1e2.t,Hmk6_observer_rho1e2.y(:,5)*rtd,'b',Hmk6_observer_rho1e4.t, Hmk6_observer_rho1e4.y(:,5)*rtd,'k',Hmk6_observer_rho1e10.t,Hmk6_observer_rho1e10.y(:,5)*rtd,'LineWidth',2);grid
legend('Hmk3 ', ...
       'Hmk6 observer rho1e2 ',...
       'Hmk6 observer rho1e4 ',...
       'Hmk6 observer rho1e10 ',...
       'Location','Best');
xlabel('Time (sec)');
ylabel('Elevon Rate(dps)');
if(save_plots == 1) saveppt2(plot_file_name); end


% 
% % ngm = -1/neg_gm;
% % pgm = -1/pos_gm;
% figure('Name','Nyquist Plot'),
% % plot(xx1,yy1,'k:',real(squeeze(Hmk6_observer_rho1e2.Lu)),imag(squeeze(Hmk6_observer_rho1e2.Lu)),...'k',...
% %     [ngm -1.],[0. 0.],'r',[-1 pgm],[0. 0.],'c','LineWidth',2);grid
% plot(xx1,yy1,'k:',real(squeeze(Hmk6_observer_rho1e2.Lu)),imag(squeeze(Hmk6_observer_rho1e2.Lu)),...'k',...
%     'LineWidth',2);grid
% 
% axis([-3 3 -3 3]);
% xlabel('Re(Lu)')
% ylabel('Im(Lu)')
% legend('Unit Circle at -1,j0','Lu','Neg SV Margin','Pos SV Margin');
% if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Nyquist Plot at Plant Input'),
plot(xx1,yy1,'r',real(squeeze(Hmk3_RSLQR.Lu)),imag(squeeze(Hmk3_RSLQR.Lu)),...
    real(squeeze(Hmk6_observer_rho1e2.Lu)),imag(squeeze(Hmk6_observer_rho1e2.Lu)),'b',...
    real(squeeze(Hmk6_observer_rho1e4.Lu)),imag(squeeze(Hmk6_observer_rho1e4.Lu)),...
    real(squeeze(Hmk6_observer_rho1e10.Lu)),imag(squeeze(Hmk6_observer_rho1e10.Lu)),...
'k','LineWidth',2);grid
axis([-2 2 -2 2]);
legend('Unit Circle','Hmk3 RSLQR','Hmk6 observer rho1e2 ',...
       'Hmk6 observer rho1e4 ',...
       'Hmk6 observer rho1e10 ','Location','Best');
xlabel('Re(L)')
ylabel('Im(L)')
title('Nyquist Plot at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Bode Magnitude at Plant Input'),
semilogx(Hmk3_RSLQR.w,20*log10(abs(squeeze(Hmk3_RSLQR.Lu))),...
    Hmk6_observer_rho1e2.w,20*log10(abs(squeeze(Hmk6_observer_rho1e2.Lu))),...
    Hmk6_observer_rho1e4.w,20*log10(abs(squeeze(Hmk6_observer_rho1e4.Lu))),...
    Hmk6_observer_rho1e10.w,20*log10(abs(squeeze(Hmk6_observer_rho1e10.Lu))),...
    'k','LineWidth',2);grid
legend([ 'Hmk3 RSLQR LGCF = '  num2str(Hmk3_RSLQR.metric(4)) ],...
       [ 'Hmk6 observer rho1e2-SF LGCF = '     num2str(Hmk6_observer_rho1e2.metric(4)) ],...
       [ 'Hmk6 observer rho1e4-SF LGCF = '  num2str(Hmk6_observer_rho1e4.metric(4)) ],...
       [ 'Hmk6 observer rho1e10-SF LGCF = '  num2str(Hmk6_observer_rho1e10.metric(4)) ],...
       'Location','Best');
xlabel('Frequency (rps)')
ylabel('Mag')
title('Bode at Input')
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Return Difference at Plant Input'),
semilogx(Hmk3_RSLQR.w,20*log10(abs(Hmk3_RSLQR.rd)),Hmk6_observer_rho1e2.w,20*log10(abs(Hmk6_observer_rho1e2.rd)),'b',...
    Hmk6_observer_rho1e4.w,20*log10(abs(Hmk6_observer_rho1e4.rd)),Hmk6_observer_rho1e10.w,20*log10(abs(Hmk6_observer_rho1e10.rd)),'k','LineWidth',2);grid
legend(['Hmk3 RSLQR I+Lu min = ' num2str(Hmk3_RSLQR.metric(2)) ],...
       ['Hmk6 observer rho1e2 min(I+Lu) = ' num2str(Hmk6_observer_rho1e2.metric(2))],...
       ['Hmk6 observer rho1e4 I+Lu min = ' num2str(Hmk6_observer_rho1e4.metric(2)) ],...
       ['Hmk6 observer rho1e10 I+Lu min = ' num2str(Hmk6_observer_rho1e10.metric(2)) ],...
       'Location','Best');
xlabel('Frequency (rps)')
ylabel('Mag dB')
title('Return Difference at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Stability Robustness at Plant Input'),
semilogx(Hmk6_observer_rho1e2.w,20*log10(abs(Hmk6_observer_rho1e2.sr)),'b',Hmk3_RSLQR.w,20*log10(abs(Hmk3_RSLQR.sr)),...
        Hmk6_observer_rho1e4.w,20*log10(abs(Hmk6_observer_rho1e4.sr)),Hmk6_observer_rho1e10.w,20*log10(abs(Hmk6_observer_rho1e10.sr)),'k', 'LineWidth',2);grid
legend(['Hmk3 RSLQR I+Lu min = ' num2str(Hmk3_RSLQR.metric(3)) ],...
       ['Hmk6 observer rho1e2 min(I+Lu) = ' num2str(Hmk6_observer_rho1e2.metric(3))],...
       ['Hmk6 observer rho1e4 I+Lu min = ' num2str(Hmk6_observer_rho1e4.metric(3)) ],...
       ['Hmk6 observer rho1e10 I+Lu min = ' num2str(Hmk6_observer_rho1e10.metric(3)) ],...
       'Location','Best');
xlabel('Frequency (rps)')
ylabel('Mag dB')
title('Stability Robustness at Plant Input')
if(save_plots == 1) saveppt2(plot_file_name); end

figure('Name','Comp Sens T');
semilogx(Hmk3_RSLQR.w,Hmk3_RSLQR.T_st(Hmk3_RSLQR.ip),...
    Hmk6_observer_rho1e2.w,Hmk6_observer_rho1e2.T_Az,...
    Hmk6_observer_rho1e4.w,Hmk6_observer_rho1e4.T_Az,...
    Hmk6_observer_rho1e10.w,Hmk6_observer_rho1e10.T_Az, 'LineWidth',2);grid
legend( ['Hmk3 RSLQR Tmax = ' num2str(Hmk3_RSLQR.metric(11)) ],...
        ['Hmk6 observer rho1e2 Tmax = ' num2str(Hmk6_observer_rho1e2.metric(11))],...
        ['Hmk6 observer rho1e4 Tmax = ' num2str(Hmk6_observer_rho1e4.metric(11)) ],...
        ['Hmk6 observer rho1e10 Tmax = ' num2str(Hmk6_observer_rho1e10.metric(11)) ],...
        'Location','Best');
title('Comp Sens T');
xlabel('Freq (rps)');ylabel('Mag (dB)');
if(save_plots == 1) saveppt2(plot_file_name); end  

figure('Name','Sens S');
semilogx(Hmk3_RSLQR.w,Hmk3_RSLQR.S_st(Hmk3_RSLQR.ip),...
    Hmk6_observer_rho1e2.w,Hmk6_observer_rho1e2.S_Az,...
    Hmk6_observer_rho1e4.w,Hmk6_observer_rho1e4.S_Az,...
    Hmk6_observer_rho1e10.w,Hmk6_observer_rho1e10.S_Az, 'LineWidth',2);grid
legend( ['Hmk3 RSLQR Smax = ' num2str(Hmk3_RSLQR.metric(12)) ],...
        ['Hmk6 observer rho1e2 Smax = ' num2str(Hmk6_observer_rho1e2.metric(12))],...
        ['Hmk6 observer rho1e4 Smax = ' num2str(Hmk6_observer_rho1e4.metric(12)) ],...
        ['Hmk6 observer rho1e10 Smax = ' num2str(Hmk6_observer_rho1e10.metric(12)) ],...
        'Location','Best');
title('Sens S');
xlabel('Freq (rps)');ylabel('Mag (dB)');
if(save_plots == 1) saveppt2(plot_file_name); end  

% Compute SV Margins for Each Controller
rdu_min = Hmk3_RSLQR.metric(2);
sru_min = Hmk3_RSLQR.metric(3);
disp(' ')
disp('Homework 3 RSLQR')
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

disp('Hmk6_observer_rho1 = e2')
rdu_min = Hmk6_observer_rho1e2.rdu_min;
sru_min = Hmk6_observer_rho1e2.sru_min;
disp(' ')
disp('Rho = 10e2')
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

disp('Hmk6_observer_rho1 = e4')
rdu_min = Hmk6_observer_rho1e4.rdu_min;
sru_min = Hmk6_observer_rho1e4.sru_min;
disp(' ')
disp('Rho = 10e4')
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

disp('Hmk6_observer_rho1 = e10')
rdu_min = Hmk6_observer_rho1e10.rdu_min;
sru_min = Hmk6_observer_rho1e10.sru_min;
disp(' ')
disp('Rho = 10e10')
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



     
figure('Name','Noise-2-Control');
semilogx(w,Hmk3_RSLQR.dele_Az,'k',w,Hmk3_RSLQR.dele_q,'k',...
         w,Hmk6_observer_rho1e2.dele_Az,'b--',w,Hmk6_observer_rho1e2.dele_q,'b--',...
         w,Hmk6_observer_rho1e4.dele_Az,'r--',w,Hmk6_observer_rho1e4.dele_q,'r--',...
         w,Hmk6_observer_rho1e10.dele_Az,'g--',w,Hmk6_observer_rho1e10.dele_q,'g--',...
         'LineWidth',2);
legend('Az HW3 RSLQR','q HW3 RSLQR',...
       'Az Rho = 10e2','q Rho = 10e2',...
       'Az Rho = 10e4','q  Rho = 10e4',...
       'Az Rho = 10e10','q Rho = 10e10',...
       'Location','Best');
title(['Noise to Control ']);
xlabel('Freq (rps)');ylabel('Mag (dB)');
grid;
if(save_plots == 1) saveppt2(plot_file_name); end   

figure('Name','Noise-2-Control');
semilogx(w,Hmk3_RSLQR.deledot_Az,'k',w,Hmk3_RSLQR.deledot_q,'k',...
         w,Hmk6_observer_rho1e2.deledot_Az,'b--',w,Hmk6_observer_rho1e2.deledot_q,'b--',...
         w,Hmk6_observer_rho1e4.deledot_Az,'r--',w,Hmk6_observer_rho1e4.deledot_q,'r--',...
         w,Hmk6_observer_rho1e10.deledot_Az,'g--',w,Hmk6_observer_rho1e10.deledot_q,'g--',...
         'LineWidth',2);
legend('Az HW3 RSLQR','q HW3 RSLQR',...
       'Az Rho = 10e2','q Rho = 10e2',...
       'Az Rho = 10e4','q  Rho = 10e4',...
       'Az Rho = 10e10','q Rho = 10e10',...
       'Location','Best');
title(['Noise to Control ']);
xlabel('Freq (rps)');ylabel('Mag (dB)');
grid;
if(save_plots == 1) saveppt2(plot_file_name); end   


return     


