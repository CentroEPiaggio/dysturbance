function [PI_plot_output,Stability_margin_PI] = Global_stability_margins(KPI_value, Protocol, Global_PI_folder)
%--------------------------------------------------------------------------
% This function compute the complexive kpi value depending on the KPI it is
% calculated in it, and if it is dependent with frequency.
% KPI Names:
% - 'Impulsive Stability Margin' 1
% - 'Sinusoidal Stability Margin' 2or 3
% - 'Quasi-static Stability Margin' 4 or 5
%
% The KPI_value inserted will be a matrix of all the value derived from
% singular test computation. They will be listed in different rows, as in
% the columns there will be the main values and the frequency(if exists) of
% the single experiment
% ----
% EX:      KPI_value = [KPI_test1, frequency_test1;
%                       KPI_test2, frequency_test2;
%                       ........
%                       KPI_testn, frequency_testn];
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------
switch Protocol
    case 1
        % must divide the different experiment with multiple repetitions
        Matrix_Data = sortrows(KPI_value,3);
        angle_check = Matrix_Data(1,3);
        K = 1;
        j = 0;
        for i = 1:size(Matrix_Data,1)
            if (Matrix_Data(i,3) >= angle_check - 0.5) && (Matrix_Data(i,3) <= angle_check + 0.5)
                j = j + 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
            else
                K = K + 1;
                j = 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
            end
        end
        % now that we have grouped the experiments with the same initial
        % condition, we need to obtain mean value and std of forces and
        % initial energy
        for KK = 1:size(Data_structure,2)
            PI_plot_output(KK,:) = [mean(Data_structure{KK}(:,1)), std(Data_structure{KK}(:,1)), ...
                mean(Data_structure{KK}(:,2)), std(Data_structure{KK}(:,2)) ...
                mean(Data_structure{KK}(:,3)), std(Data_structure{KK}(:,3))];
        end
        % Now, with the flag of isfall, we check the minimum force that
        % unstabilize the systems +30% of the time.
        
        t = 1;
        for kkk = 1:size(Data_structure,2)
            fallen = 0;
            total_exp = size(Data_structure{kkk},1);
            for p = 1:total_exp
                if Data_structure{kkk}(p,4) == 1
                    fallen = fallen + 1;
                end
            end
            Check_for_perc = fallen/total_exp * 100;
            if Check_for_perc < 30
                Stab_margin_PI_part(t,:) = [mean(Data_structure{KK}(:,1)), std(Data_structure{KK}(:,1)), ...
                    mean(Data_structure{KK}(:,2)), std(Data_structure{KK}(:,2)), ...
                    mean(Data_structure{KK}(:,3)), std(Data_structure{KK}(:,3))];
                t = t + 1;
            end
            
        end
        
        [~, index]= min(Stab_margin_PI_part(:,1));
        Stability_margin_PI = Stab_margin_PI_part(index,:);
        header = ["mean force stability margin [N]","Standard deviation force stability margin [N]", ...
            "mean normalized force stability margin [N]","Standard deviation normalized force stability margin [N]", ...
            "Mean Initial_position [degrees]", "Standard deviation on initial position [degrees]"];
        Stability_margin_PI_Plot = [header;PI_plot_output];
        writematrix(Stability_margin_PI_Plot,strcat(Global_PI_folder,'\Global_Stability_margin_plot.csv'));
        
    case 2
        % Initial input has the form [KPI_value, frequency]
        % We must divided the experiments in runs with the same
        % frequencies, and for this subset check the minimum force that
        % unstabilize the system.
        Matrix_Data = sortrows(KPI_value,3);
        frequency_check = Matrix_Data(1,3);
        K = 1;
        j = 0;
        for i = 1:size(Matrix_Data,1)
            if (Matrix_Data(i,3) >= frequency_check - 0.01) && (Matrix_Data(i,3) <= frequency_check + 0.01)
                j = j + 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
            else
                K = K + 1;
                j = 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
            end
        end
        for KK = 1:size(Data_structure,2)
            PI_plot_output(KK,:) = [mean(Data_structure{KK}(:,1)), std(Data_structure{KK}(:,1)), ...
                mean(Data_structure{KK}(:,2)), std(Data_structure{KK}(:,2)) ...
                mean(Data_structure{KK}(:,3)), std(Data_structure{KK}(:,3))];
        end
        
        t = 1;
        for kkk = 1:size(Data_structure,2)
            fallen = 0;
            total_exp = size(Data_structure{kkk},1);
            for p = 1:total_exp
                if Data_structure{kkk}(p,4) == 1
                    fallen = fallen + 1;
                end
            end
            Check_for_perc = fallen/total_exp * 100;
            if Check_for_perc < 30
                Stab_margin_PI_part(t,:) = [mean(Data_structure{KK}(:,1)), std(Data_structure{KK}(:,1)), ...
                    mean(Data_structure{KK}(:,2)), std(Data_structure{KK}(:,2)) ...
                    mean(Data_structure{KK}(:,3)), std(Data_structure{KK}(:,3))];
            end
            
            [~, index]= min(Stab_margin_PI_part(:,1));
            if fallen ~= 0
                Stability_margin_PI(kkk,:) = Stab_margin_PI_part(index,:);
            else
                Stability_margin_PI(kkk,:) = [nan,nan,nan,nan,nan,nan];
            end
        end
        header = ["mean force sinusoidal stability margin [N]","Standard deviation force sinusoidal stability margin [N]", ...
            "mean normalized force sinusoidal stability margin [N]","Standard deviation normalized sinusoidal force stability margin [N]", ...
            "Mean frequency [Hz]", "Standard deviation on frequency [Hz]"];
        Stability_margin_PI_Plot = [header;PI_plot_output];
        writematrix(Stability_margin_PI_Plot,strcat(Global_PI_folder,'\Global_Stability_margin_plot.csv'));
        
    case 3
        % Initial input has the form [KPI_value, frequency]
        % We must divided the experiments in runs with the same
        % frequencies, and for this subset check the minimum force that
        % unstabilize the system.
        Matrix_Data = sortrows(KPI_value,3);
        frequency_check = Matrix_Data(1,3);
        K = 1;
        j = 0;
        for i = 1:size(Matrix_Data,1)
            if (Matrix_Data(i,3) >= frequency_check - 0.1) && (Matrix_Data(i,3) <= frequency_check + 0.1)
                j = j + 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
            else
                K = K + 1;
                j = 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
            end
        end
        % Must find the minimum destabilizing force varying the
        % frequencies
        for KK = 1:size(Data_structure,2)
            PI_plot_output(KK,:) = [mean(Data_structure{KK}(:,1)), std(Data_structure{KK}(:,1)), ...
                mean(Data_structure{KK}(:,2)), std(Data_structure{KK}(:,2)) ...
                mean(Data_structure{KK}(:,3)), std(Data_structure{KK}(:,3))];
        end
        
        t = 1;
        for kkk = 1:size(Data_structure,2)
            fallen = 0;
            total_exp = size(Data_structure{kkk},1);
            for p = 1:total_exp
                if Data_structure{kkk}(p,4) == 1
                    fallen = fallen + 1;
                end
            end
            Check_for_perc = fallen/total_exp * 100;
            if Check_for_perc < 30
                Stab_margin_PI_part(t,:) = [mean(Data_structure{KK}(:,1)), std(Data_structure{KK}(:,1)), ...
                    mean(Data_structure{KK}(:,2)), std(Data_structure{KK}(:,2)) ...
                    mean(Data_structure{KK}(:,3)), std(Data_structure{KK}(:,3))];
            end
            
            [~, index]= min(Stab_margin_PI_part(:,1));
            if fallen ~= 0
                Stability_margin_PI(kkk,:) = Stab_margin_PI_part(index,:);
            else
                Stability_margin_PI(kkk,:) = [nan,nan,nan,nan,nan,nan];
            end
        end
        
        header = ["mean force displacement stability margin [N]","Standard deviation force displacement stability margin [N]", ...
            "mean normalized force displacement stability margin [N]","Standard deviation normalized displacement force stability margin [N]", ...
            "Mean frequency [Hz]", "Standard deviation on frequency [Hz]"];
        Stability_margin_PI_Plot = [header;PI_plot_output];
        writematrix(Stability_margin_PI_Plot,strcat(Global_PI_folder,'\Global_Stability_margin_plot.csv'));
    case 4
        % The experiment always brings the system to fail. Then, we must
        % compute a mean of the value of destabilization
        Stability_margin_PI = mean(KPI_value(:,1));
        header = "Quasi Static Displacement Stability margin [m]";
    case 5
        % The experiment always brings the system to fail. Then, we must
        % compute a mean of the value of destabilization
        Stability_margin_PI = mean(KPI_value(:,1));
        header = "Quasi Static Force Stability margin [N]";
    otherwise
        error('ERROR: unknown protocol e for Key Performance Indicator');
end

Stability_margin_PI_value = [header;Stability_margin_PI];
writematrix(Stability_margin_PI_value,strcat(Global_PI_folder,'\Global_Stability_margin.csv'));
end

