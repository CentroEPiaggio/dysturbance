clear all;
close all;
clc;
%%nota: 
%%freq. campionamento posizione del pendolo a 250Hz.
%%freq. campionamento forze a 10kHz.
%%struttura dati raw = [t (250hz), dati NI (10kHz), angolo (250Hz) ]

load 'file_di_calibrazione.mat';

mat_ver = load('p_4_4_obj_10_deg_1.CSV');

%% VALIDATION
num_channels = 7; %numero di canali della NI utilizzati 
num_data = 40; %rapporto tra freq. campionamento angolo e freq. campionamento forze. ovvero per ogni valore di angolo e di tempo ho 7x40 dati dele forze.

time = mat_ver(:,1); %estraggo il vettore tempo

%spacchetto i dati provenienti dalla NI e li ordino in vettori (Fx,Fy,Fz,Mx,My,Mz e F_piezoelettrico)
ai_ver = cell(num_channels,1);
ati_val_ver = cell(6, 1);
s = 2;
for j=1:num_channels
    for i=1:size(mat_ver,1)
        ai_ver{j,1} = [ai_ver{j,1}, mat_ver(i, s+num_data*(j-1):s+num_data*(j-1)+(num_data-1))];
    end
end

angle_ver = mat_ver(:,end);%estraggo il vettore angolo

% CALIBRATION ATI
for i=1:size(ai_ver{1,1},2)
    v = [];
    for j=1:6
        v = [v; ai_ver{j,1}(i)];
    end
    y = CALIB_MAT*v;
    for j=1:6
        ati_val_ver{j,1}(i) = y(j,1)/scale(j,1);
    end
end

 piezo_val = ai_ver{7,1};
 piezo_val_ver = -(piezo_val/params_piezo_amp_10(1) - params_piezo_amp_10(2));

%% PLOTS
%%figure;
%%plot(diff(mat_ver(:,1))); grid on;
%%title('Time - Differences between samples');
%%xlabel('Samples');
%%ylabel('Time difference (ms)');

num_rows = round(num_channels/4);
num_cols = round(num_channels/2);
figure('Color', [1 1 1]);
title('Raw acquired data from NIDAQ');
for j=1:num_rows %rows
    for k=1:num_cols % cols
         if (j-1)*num_cols + k <= num_channels
            subplot(num_rows, num_cols, (j-1)*num_cols +k); 
             plot(ai_ver{(j-1)*num_cols +k, 1}); 
%              ylim([0 5]);
             grid on;
             title(['Channel ai_',num2str((j-1)*num_cols +k -1)]);
             xlabel('Samples');
             ylabel('Volts');
         end
    end
end

figure; 
title('ATI and PIEZO calibrated data');
subplot(num_rows, num_cols, 1); 
plot(ati_val_ver{1, 1}-mean(ati_val_ver{1, 1}(1:1000))); grid on; title('ATI F_x');
xlabel('Samples');
ylabel('Force (N)');
subplot(num_rows, num_cols, 2); 
plot(ati_val_ver{2, 1}-mean(ati_val_ver{2, 1}(1:1000))); grid on; title('ATI F_y');
xlabel('Samples');
ylabel('Force (N)');
subplot(num_rows, num_cols, 3); 
plot(ati_val_ver{3, 1}-mean(ati_val_ver{3, 1}(1:1000))); grid on; title('ATI F_z');
xlabel('Samples');
ylabel('Force (N)');
subplot(num_rows, num_cols, 4); 
plot(ati_val_ver{4, 1}-mean(ati_val_ver{4, 1}(1:1000))); grid on; title('ATI T_x');
xlabel('Samples');
ylabel('Torque (Nm)');
subplot(num_rows, num_cols, 5); 
plot(ati_val_ver{5, 1}-mean(ati_val_ver{5, 1}(1:1000))); grid on; title('ATI T_y');
xlabel('Samples');
ylabel('Torque (Nm)');
subplot(num_rows, num_cols, 6); 
plot(ati_val_ver{6, 1}-mean(ati_val_ver{6, 1}(1:1000))); grid on; title('ATI T_z');
xlabel('Samples');
ylabel('Torque (Nm)');
subplot(num_rows, num_cols, 7); 
plot(piezo_val_ver-mean(piezo_val_ver(1:1000))); grid on; title('Piezo');
xlabel('Samples');
ylabel('Force (N)');
subplot(num_rows, num_cols, 8); 
plot(angle_ver); grid on; title('Pendulum');
xlabel('Samples');
ylabel('Angle (deg)');


%%figure;
%%plot(angle_ver); grid on;
%%title('Angle - degrees');
%%xlabel('Samples');
%%ylabel('Angle (deg)');

x = ati_val_ver{1, 1} - mean(ati_val_ver{1, 1}(1:1000));
y = piezo_val_ver - mean(piezo_val_ver(1:1000));
%figure, 
%hold on;
%plot(y/1 , 'r'); grid on; 
%plot(x ); grid on;
%title('ATI F_x vs. PIEZO biased values - Comparison');
%xlabel('Samples');
%ylabel('N');

a_val=min(angle_ver);
ind=find(angle_ver==a_val,1);
ind2=find(angle_ver==a_val,1,'last');

figure
hold on
plot(time,angle_ver)
plot([time(ind) time(ind)],[-100 100], '--')
%plot([time(ind2) time(ind2)],[-100 100], '--')
%plot([40:40:length(piezo_val)]/10,angle_ver)
plot([1:1:length(piezo_val)]/10,y/1,'r')
plot([1:1:length(piezo_val)]/10,x,'g')
xlabel('ms')
grid on

figure
hold on
grid on
plot([1:1:length(angle_ver)]*40,angle_ver)
plot([1:1:length(piezo_val)],y,'r')
plot([ind ind]*40,[-100 100], '--')
xlim([ind-200 ind+300]*40)