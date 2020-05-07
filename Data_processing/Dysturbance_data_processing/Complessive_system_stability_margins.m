function [PI_plot_output,Stability_margin_PI] = Complessive_system_stability_margins(KPI_name,KPI_value)
%--------------------------------------------------------------------------
% This function compute the complexive kpi value depending on the KPI it is
% calculated in it, and if it is dependent with frequency.
% KPI Names:
% - 'Impulsive Stability Margin'
% - 'Sinusoidal Stability Margin'
% - 'Quasi-static Stability Margin'
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
% mail: simone.monteleone@iit.it
%--------------------------------------------------------------------------
switch KPI_name
    case 'Impulsive Stability Margin'
        % must divide the different experiment with multiple repetitions
        Matrix_Data = sortrows(KPI_value,2);
        angle_check = Matrix_Data(1,2);
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
        % now that we have grouped the experiments with the same initial
        % condition, we need to obtain mean value and std of forces and
        % initial energy
        for KK = 1:size(Data_structure,2)
            PI_plot_output(KK,:) = [mean(Data_structure{KK}(:,1)), std(Data_structure{KK}(:,1)), ...
                    mean(Data_structure{KK}(:,2)), std(Data_structure{KK}(:,2))];
        end
        % Now, with the flag of isfall, we check the minimum force that
        % unstabilize the systems +30% of the time.
        
        %TO BE DONE
%         Stability_margin_PI = 0;
        
        
    case 'Sinusoidal Stability Margin'
        % Initial input has the form [KPI_value, frequency]
        % We must divided the experiments in runs with the same
        % frequencies, and for this subset check the minimum force that
        % unstabilize the system.
        Matrix_Data = sortrows(KPI_value,2);
        angle_check = Matrix_Data(1,2);
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
        % Must find the minimum destabilizing force varying the
        % frequencies
        
        
    case 'Quasi-static Stability Margin'
      % The experiment always brings the system to fail. Then, we must
      % compute a mean of the value of destabilization
      Stability_margin_PI = mean(KPI_value(:,1));
        
%     case 'Equivalent Impedance
%     case 'Absorbed Energy'
    otherwise
        error('ERROR: unknown name for Key Performance Indicator');
end
end

