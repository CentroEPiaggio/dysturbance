%% data collection for grasp resilience sh hand
%% ORGANIZZA I DATI RAW creando delle cell n_rep x deg_vec

clear all
close all
clc

load 'file_di_calibrazione.mat';

deg_vec=[ 5 10 15 20 25 30 40 50 60 75];
rep_vec=[1 2 3 4 5 6 7 8 9 10 ];

config=[1 2 3 4];
obj_size=[2 4 6];

z=4; %%%config parameter
k=2;%%obj size parameter

obj_size(k)
config(z)


for j=1:length(deg_vec)
for i=1:length(rep_vec)
   filename=strcat('p_',num2str(config(z)),'_',num2str(obj_size(k)),'_obj_',num2str(deg_vec(j)),'_deg_',num2str(rep_vec(i)),'.csv');

    if exist (filename, 'file')
    %call the DATA_read_fun
    %and create cells with ATi and  with piezo values for all the cases
    
        
 [Fx{i,j}, Fy{i,j},Fz{i,j}, tx{i,j}, ty{i,j},tz{i,j},piezo{i,j},angle{i,j},time{i,j}] =DATA_read_fun(filename, CALIB_MAT,params_piezo_amp_10,scale);
   
 f_ati_norm{i,j}=sqrt(Fx{i,j}.^2+Fy{i,j}.^2+Fz{i,j}.^2);
 
    end
    
end
end

[n_row,n_col]=size(f_ati_norm);

% % 
% for b=1:n_col
% for a=1:n_row
%     
% time_ati=([1:1:length(f_ati_norm{a,b})]/10)-1000;
% 
% figure
% hold on
% plot(time_ati,f_ati_norm{a,b})
% plot(time_ati,piezo{a,b},'--b')
% plot(time{a,b},angle{a,b})
% 
% figure
% hold on
% plot(time_ati,f_ati_norm{a,b})
% plot(time_ati,Fx{a,b})
% plot(time_ati,Fy{a,b})
% plot(time_ati,Fz{a,b})
% 
% end
% end









% figure
% hold on
% plot(f_ati_norm)
% plot(Fx{1,1})
% plot(piezo{1,1},'--b')

% figure
% hold on
% plot(tz{1,1}*100)
% plot(tz{1,2}*100,'g')
% plot(tz{1,3}*100,'r')
% plot(piezo{1,1},'--b')
% plot(piezo{1,2},'--g')
% plot(piezo{1,3},'--r')