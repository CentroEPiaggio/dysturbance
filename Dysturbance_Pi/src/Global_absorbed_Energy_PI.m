function Global_absorbed_Energy_PI(Global_DE_matrix, Global_PI_folder)
%--------------------------------------------------------------------------
% This function computes the Global KPI for Absorbed Energy for Protocol 1.
% Global KPI must be stored in a Matrix of this form
%          KPI_value = [KPI_force1, DE_test1, angle_test1;
%                       KPI_test2,  DE_test2, angle_test2;
%                       ........
%                       KPI_testn,  DE_testn, angle_testn];
% This set of datas contains all the experiment that are repetition of a
% certain condition, and the set of experiments at different conditions
%
% NOTE: angle_test column must be ordered in crescent or decrescent
% condition
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

% first, we must group the experiments that are holded in the same
% conditions (degree is the best on to check conditions).
Matrix_Data = sortrows(Global_DE_matrix,7);
mass_check = Matrix_Data(1,7);
K = 1;
j = 0;

for i = 1:size(Matrix_Data,1)
    if (Matrix_Data(i,7) >= mass_check - 0.25) && (Matrix_Data(i,7) <= mass_check + 0.25)
        j = j + 1;
        Data_structure{K}(j,:) = Matrix_Data(i,:);
    else
        K = K + 1;
        j = 1;
        Data_structure{K}(j,:) = Matrix_Data(i,:);
        mass_check = Matrix_Data(i,7);
    end
end
% organize by starting point
for i_mas = 1:size(Data_structure,2)
    clear Matrix_Data;
    Matrix_Data = sortrows(Data_structure{i_mas},6);
    angle_check = Matrix_Data(1,6);
    Kmas = 1;
    jmas = 0;
    for i_mas_2 = 1:size(Matrix_Data,1)
        if (Matrix_Data(i_mas_2,6) >= angle_check - 1.1) && (Matrix_Data(i_mas_2,6) <= angle_check + 1.1)
            jmas = jmas + 1;
            Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
        else
            Kmas = Kmas + 1;
            jmas = 1;
            Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
            angle_check = Matrix_Data(i_mas_2,6);
        end
    end
end
%------------------- Global Absorbed Energy Computation -------------------
% We must get the mean value and standard deviation of values of energy and
% force with the same angle, then put them in a matrix. Save them also in a
% plot if possible

% PI output is a matrix in which it is stored:
%  Energy = [mean_force, std_force, mean_energy,std_energy, angle, Added Mass]

point = 1;
point2 = 1;
point3 = 1;
Absorbed_energy_KPI_matrix = [];
Data_fallen_part = [];
Data_fallen = [];
for KK = 1:size(Data_structure_2,1)
    for ii = 1:size(Data_structure_2,2)
        if ~isempty(Data_structure_2{KK,ii})
            if mean(Data_structure_2{KK,ii}(:,end)) == 0
            Absorbed_energy_KPI_matrix(point,:) = [mean(Data_structure_2{KK,ii}(:,1)), std(Data_structure_2{KK,ii}(:,1)), ...
                mean(Data_structure_2{KK,ii}(:,2)), std(Data_structure_2{KK,ii}(:,2)), ...
                mean(Data_structure_2{KK,ii}(:,3)), std(Data_structure_2{KK,ii}(:,3)), ...
                mean(Data_structure_2{KK,ii}(:,4)), std(Data_structure_2{KK,ii}(:,4)), ...
                mean(Data_structure_2{KK,ii}(:,5)), std(Data_structure_2{KK,ii}(:,5)), ...
                mean(Data_structure_2{KK,ii}(:,8)), std(Data_structure_2{KK,ii}(:,8)), ...
                mean(Data_structure_2{KK,ii}(:,end-1)), std(Data_structure_2{KK,ii}(:,end-1)), ...
                mean(Data_structure_2{KK,ii}(:,6)), mean(Data_structure_2{KK,ii}(:,7)), mean(Data_structure_2{KK,ii}(:,end))];
            point = point + 1;
            elseif mean(Data_structure_2{KK,ii}(:,end)) >= 0.3
                Data_fallen(point2,:) = [mean(Data_structure_2{KK,ii}(:,1)), std(Data_structure_2{KK,ii}(:,1)), ...
                mean(Data_structure_2{KK,ii}(:,2)), std(Data_structure_2{KK,ii}(:,2)), ...
                mean(Data_structure_2{KK,ii}(:,3)), std(Data_structure_2{KK,ii}(:,3)), ...
                mean(Data_structure_2{KK,ii}(:,4)), std(Data_structure_2{KK,ii}(:,4)), ...
                mean(Data_structure_2{KK,ii}(:,5)), std(Data_structure_2{KK,ii}(:,5)), ...
                mean(Data_structure_2{KK,ii}(:,8)), std(Data_structure_2{KK,ii}(:,8)), ...
                mean(Data_structure_2{KK,ii}(:,end-1)), std(Data_structure_2{KK,ii}(:,end-1)), ...
                mean(Data_structure_2{KK,ii}(:,6)), mean(Data_structure_2{KK,ii}(:,7)), mean(Data_structure_2{KK,ii}(:,end))];
                point2 = point2 + 1;
            else
                Data_fallen_part(point3,:) = [mean(Data_structure_2{KK,ii}(:,1)), std(Data_structure_2{KK,ii}(:,1)), ...
                mean(Data_structure_2{KK,ii}(:,2)), std(Data_structure_2{KK,ii}(:,2)), ...
                mean(Data_structure_2{KK,ii}(:,3)), std(Data_structure_2{KK,ii}(:,3)), ...
                mean(Data_structure_2{KK,ii}(:,4)), std(Data_structure_2{KK,ii}(:,4)), ...
                mean(Data_structure_2{KK,ii}(:,5)), std(Data_structure_2{KK,ii}(:,5)), ...
                mean(Data_structure_2{KK,ii}(:,8)), std(Data_structure_2{KK,ii}(:,8)), ...
                mean(Data_structure_2{KK,ii}(:,end-1)), std(Data_structure_2{KK,ii}(:,end-1)), ...
                mean(Data_structure_2{KK,ii}(:,6)), mean(Data_structure_2{KK,ii}(:,7)), mean(Data_structure_2{KK,ii}(:,end))];
                point3 = point3 + 1;
            end
        end
    end
end

figure('defaultAxesFontSize',26);
if ~isempty(Absorbed_energy_KPI_matrix)
plot3(Absorbed_energy_KPI_matrix(:,11),Absorbed_energy_KPI_matrix(:,7),Absorbed_energy_KPI_matrix(:,1),'x','LineWidth',2);
end
hold on;
if ~isempty(Data_fallen_part)
    plot3(Data_fallen_part(:,11),Data_fallen_part(:,7),Data_fallen_part(:,1),'gx','LineWidth',2);
end
if ~isempty(Data_fallen)
    plot3(Data_fallen(:,11),Data_fallen(:,7),Data_fallen(:,1),'rx','LineWidth',2);
end
grid;
xlabel('Impulse [Ns]');
ylabel('Initial Energy [J]');
zlabel('Absorbed Energy [%]');
title('Protocol 1 Absorbed Energy Plot');
header = "[Mean Energy Percentage, Standard Deviation energy percentage,Mean Max force [N], Standard Deviation Max force [N], Mean normalized Max force [N], Standard deviation normalized Max force [N], Mean Initial Energy [J], Standard deviation Initial Energy [J], Mean Normalized Initial Energy [J], Standard deviation normalized Initial Energy [J], Mean Impulse [Ns], Standard deviation Impulse [Ns], Mean Absorbed Energy [J], Standard deviation Absorbed Energy [J], Mean Initial Angle [degrees], added_mass [Kg], fallen]";

if ~isempty(Absorbed_energy_KPI_matrix)
    type = find_type(Absorbed_energy_KPI_matrix);
    
    fileID = fopen(strcat(Global_PI_folder,'\Global_Absorbed_energy.yaml'),'w');
    fprintf(fileID,'type: %s \n',type);
    fprintf(fileID, 'label: %s \n',header);
    fmt = ['value: [[', repmat('%g, ', 1, numel(Absorbed_energy_KPI_matrix(1,:))-1), '%g]'];
    fprintf(fileID, fmt, Absorbed_energy_KPI_matrix(1,:));
    for pi_i = 2:size(Absorbed_energy_KPI_matrix,1)
        fmd = [',[', repmat('%g, ', 1, numel(Absorbed_energy_KPI_matrix(pi_i,:))-1) '%g]'];
        fprintf(fileID, fmd, Absorbed_energy_KPI_matrix(pi_i,:));
    end
    fprintf(fileID, ']\n');
    fclose(fileID);
end
% Data_matrix = [header;Absorbed_energy_KPI_matrix];
% writematrix(Data_matrix, strcat(Global_PI_folder,'\Global_Absorbed_energy.csv'));
end

