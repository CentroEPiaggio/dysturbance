function [Global_energy_KPI] = Global_absorbed_Energy_PI(Global_DE_matrix)
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
% mail: simone.monteleone@iit.it
%--------------------------------------------------------------------------

% first, we must group the experiments that are holded in the same
% conditions (degree is the best on to check conditions).
Matrix_Data = sortrows(Global_DE_matrix,3);
angle_check = Matrix_Data(1,3);
K = 1;
j = 0;
for i = 2:size(Matrix_Data,1)
    if (Matrix_Data(i,3) >= angle_check - 0.5) && (Matrix_Data(i,3) <= angle_check + 0.5)
        j = j + 1;
        Data_structure{K}(j,:) = Matrix_Data(i,:);
    else
        K = K + 1;
        j = 1;
        Data_structure{K}(j,:) = Matrix_Data(i,:);
    end
end
%------------------- Global Absorbed Energy Computation -------------------
% We must get the mean value and standard deviation of values of energy and
% force with the same angle, then put them in a matrix. Save them also in a
% plot if possible

% PI output is a matrix in which it is stored:
%  Energy = [mean_force, std_force, mean_energy,std_energy, angle]
%
for KK = 1:size(Data_structure,2)
    Absorbed_energy_KPI_matrix(KK,:) = [mean(Data_structure{KK}(:,1)), std(Data_structure{KK}(:,1)), ...
        mean(Data_structure{KK}(:,2)), std(Data_structure{KK}(:,2)), mean(Data_structure{KK}(:,3))];
end


end

