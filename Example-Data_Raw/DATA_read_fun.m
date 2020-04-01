function [x,y,z,tx,ty,tz, piezo,angle_ver,time] = DATA_read_fun(filename, CALIB_MAT,params_piezo_amp_10,scale)
%%output data unbiased ati= x,y,z,tx,ty,tz; piezo, pendulum angle, time
%%%input filename 

mat_ver = load(filename);

%% data_extraction
num_channels = 7;
num_data = 40;

time = mat_ver(:,1);

ai_ver = cell(num_channels,1);
ati_val_ver = cell(6, 1);
s = 2;
for j=1:num_channels
    for i=1:size(mat_ver,1)
        ai_ver{j,1} = [ai_ver{j,1}, mat_ver(i, s+num_data*(j-1):s+num_data*(j-1)+(num_data-1))];
    end
end

angle_ver = mat_ver(:,end);

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

x = ati_val_ver{1, 1} - mean(ati_val_ver{1, 1}(1:1000));
y= ati_val_ver{2, 1} - mean(ati_val_ver{2, 1}(1:1000));
z= ati_val_ver{3, 1} - mean(ati_val_ver{3, 1}(1:1000));
tx = ati_val_ver{4, 1} - mean(ati_val_ver{4, 1}(1:1000));
ty= ati_val_ver{5, 1} - mean(ati_val_ver{5, 1}(1:1000));
tz= ati_val_ver{6, 1} - mean(ati_val_ver{6, 1}(1:1000));
piezo = (piezo_val_ver - mean(piezo_val_ver(1:1000)));


end

