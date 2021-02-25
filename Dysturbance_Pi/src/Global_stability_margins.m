function [PI_plot_output,Stability_margin_PI] = Global_stability_margins(KPI_value, Protocol, Global_PI_folder)
%--------------------------------------------------------------------------
% This function compute the complexive kpi value depending on the KPI it is
% calculated in it, and if it is dependent with frequency.
% KPI Names:
% - 'Impulsive Stability Margin' 1
% - 'Sinusoidal Stability Margin' 2 or 3
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
        %%
        % must divide the different experiment with multiple repetitions
        Matrix_Data = sortrows(KPI_value,4);
        
        %organize by mass
        mass_check = Matrix_Data(1,4);
        K = 1;
        j = 0;
        for i = 1:size(Matrix_Data,1)
            if (Matrix_Data(i,4) >= mass_check - 0.25) && (Matrix_Data(i,4) <= mass_check + 0.25)
                j = j + 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
            else
                K = K + 1;
                j = 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
                mass_check = Matrix_Data(i,4);
            end
        end
        % organize by starting point
        for i_mas = 1:size(Data_structure,2)
            clear Matrix_Data;
            Matrix_Data = sortrows(Data_structure{i_mas},3);
            angle_check = Matrix_Data(1,3);
            Kmas = 1;
            jmas = 0;
            for i_mas_2 = 1:size(Matrix_Data,1)
                if (Matrix_Data(i_mas_2,3) >= angle_check - 1.1) && (Matrix_Data(i_mas_2,3) <= angle_check + 1.1)
                    jmas = jmas + 1;
                    Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
                else
                    Kmas = Kmas + 1;
                    jmas = 1;
                    Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
                    angle_check = Matrix_Data(i_mas_2,3);
                end
            end
        end
        
        % Get the mean of the dataset and if the system fall or not
        % now that we have grouped the experiments with the same initial
        % condition, we need to obtain mean value and std of forces and
        % initial energy
        point = 1;
        point2 = 1;
        point3 = 1;
        MAX_Force = 0;
        PI_mean_std = [];
        PI_matrix = [];
        Exp_points = [];
        PI_mean_std_fallen = [];
        Data_fallen = [];
        Exp_points_fallen = [];
        PI_mean_std_fallen_part = [];
        Data_fallen_part = [];
        Exp_points_fallen_part = [];
        for i_row = 1:size(Data_structure_2,1)
            for i_col = 1:size(Data_structure_2,2)
                if ~isempty(Data_structure_2{i_row,i_col})
                    Perc_fallen = mean(Data_structure_2{i_row,i_col}(:,end));
                    force_max_mean = mean(Data_structure_2{i_row,i_col}(:,1));
                    force_max_std = std(Data_structure_2{i_row,i_col}(:,1));
                    for i_imp = 1:size(Data_structure_2{i_row,i_col},1)
                        times = Data_structure_2{i_row,i_col}(:,8);
                        times(times > 0.05) = [];
                        medium_impulse_time = mean(times);
                    end
                    medium_force_mean = mean(Data_structure_2{i_row,i_col}(:,9));
                    medium_force_std = std(Data_structure_2{i_row,i_col}(:,9));
                    medium_impulse_mean = mean(Data_structure_2{i_row,i_col}(:,10));
                    medium_impulse__vel_mean = mean(Data_structure_2{i_row,i_col}(:,11));
                    medium_impulse_std = std(Data_structure_2{i_row,i_col}(:,10));
                    norm_PI_force = mean(Data_structure_2{i_row,i_col}(:,2));
                    init_angle = mean(Data_structure_2{i_row,i_col}(:,3));
                    mass = mean(Data_structure_2{i_row,i_col}(:,4));
                    length = mean(Data_structure_2{i_row,i_col}(:,5));
                    Init_E = mean(Data_structure_2{i_row,i_col}(:,7));%mean((Data_structure_2{i_row,i_col}(:,4) + 4.13 *Data_structure_2{i_row,i_col}(:,5)/2) * 9.81 * Data_structure_2{i_row,i_col}(:,5) * (1-cosd(Data_structure_2{i_row,i_col}(:,3))));
                    if medium_impulse__vel_mean > MAX_Force
                        MAX_Force = medium_impulse__vel_mean;
                        max_row = i_row;
                        max_col = i_col;
                        MAX_VALUES = [medium_impulse__vel_mean, medium_force_mean, Init_E,mean(Data_structure_2{i_row,i_col}(:,8)),mean(Data_structure_2{i_row,i_col}(:,6)),mean(Data_structure_2{i_row,i_col}(:,7)),mass,length,init_angle, medium_impulse_time];
                    end
                    if Perc_fallen == 0
                        PI_mean_std(point,:) = [medium_impulse_mean, medium_impulse_std,Init_E,medium_impulse__vel_mean];
                        PI_matrix(point,:) = [force_max_mean,Init_E,medium_force_mean,medium_impulse_time, medium_impulse__vel_mean];
                        Exp_points(point,:) = [sqrt(0.67)*mean(Data_structure_2{i_row,i_col}(:,6)),mean(Data_structure_2{i_row,i_col}(:,7)),mean(Data_structure_2{i_row,i_col}(:,6))];
                        point = point + 1;
                    elseif Perc_fallen >= 0.30
                        PI_mean_std_fallen(point2,:) = [medium_impulse_mean, medium_impulse_std,Init_E,medium_impulse__vel_mean];
                        Data_fallen(point2,:) = [force_max_mean,Init_E,medium_force_mean,medium_impulse_time, medium_impulse__vel_mean];
                        Exp_points_fallen(point2,:) = [sqrt(0.67)*mean(Data_structure_2{i_row,i_col}(:,6)),mean(Data_structure_2{i_row,i_col}(:,7)),mean(Data_structure_2{i_row,i_col}(:,6))];
                        point2 = point2 + 1;
                    else
                        PI_mean_std_fallen_part(point3,:) = [medium_impulse_mean, medium_impulse_std,Init_E,medium_impulse__vel_mean];
                        Data_fallen_part(point3,:) = [force_max_mean,Init_E,medium_force_mean,medium_impulse_time, medium_impulse__vel_mean];
                        Exp_points_fallen_part(point3,:) = [sqrt(0.67)*mean(Data_structure_2{i_row,i_col}(:,6)),mean(Data_structure_2{i_row,i_col}(:,7)),mean(Data_structure_2{i_row,i_col}(:,6))];
                        point3 = point3 + 1;
                    end
                end
            end
        end


        I_check = linspace(0,4.0,101);
        delta = 4.13;
        g = 9.81;
        L_min = 0.5;
        L_max = 1.0;     
        Corrector = 1.0;
        E_check_1 = 3.*I_check.^2 ./ (2*delta*L_min*0.67);
        E_check_2 = I_check.^2./(2 * 0.5 * (10 + delta * L_max));
        E_check_3 = delta*L_min*g*L_min*(1-cosd(10)).*ones(size(I_check));
        E_check_4 = (10 + delta*L_max/2)*g*L_max;
        E_check_5 = g.*L_max./4.*(delta.*L_max + sqrt((delta.*L_max./3)^2 + 8.*I_check./(g.*L_max)));
        
        Corr_VALUE = mean(PI_matrix(:,5)./Exp_points(:,3));
        fprintf("Corrector Value is %f \n",Corr_VALUE);
        
        figure('defaultAxesFontSize',36);
        if ~isempty(Exp_points)
            plot(Exp_points(:,3)/Corrector,Exp_points(:,2),'x','LineWidth',3);
        end
        hold on;
        if ~isempty(Exp_points_fallen)
            plot(Exp_points_fallen(:,3)/Corrector,Exp_points_fallen(:,2),'rx','LineWidth',3);
        end
        if ~isempty(Exp_points_fallen_part)
            plot(Exp_points_fallen_part(:,3)/Corrector,Exp_points_fallen_part(:,2),'gx','LineWidth',3);
        end
        plot(I_check/Corrector,E_check_1,'k','LineWidth',3);
        plot(I_check/Corrector,E_check_2,'k','LineWidth',3);
        plot(I_check/Corrector,E_check_3,'k','LineWidth',3);
        plot(I_check/Corrector,E_check_4,'k','LineWidth',3);
        plot(I_check/Corrector,E_check_5,'k','LineWidth',3);
        grid;
%         xlim([0,2.5]);
        ylim([0,3]);
        xlabel('Expected Impulse [Ns]');
        ylabel('Expected Initial Energy [J]');
        title('Protocol 1 Stability Margin Plot');

        
        
        figure('defaultAxesFontSize',36);
        if ~isempty(PI_matrix)
            plot(PI_matrix(:,5),PI_matrix(:,2),'x','LineWidth',3);
        end
        hold on;
        if ~isempty(Data_fallen)
            plot(Data_fallen(:,5),Data_fallen(:,2),'rx','LineWidth',3);
        end
        if ~isempty(Data_fallen_part)
            plot(Data_fallen_part(:,5),Data_fallen_part(:,2),'gx','LineWidth',3);
        end
        grid;
        xlabel('Impulse [Ns]');
        ylabel('Initial Energy [J]');
        title('Protocol 1 Stability Margin Plot');
        
        figure('defaultAxesFontSize',36);
        if ~isempty(PI_matrix)
            plot(PI_matrix(:,1),PI_matrix(:,2),'x','LineWidth',3);
        end
        hold on;
        if ~isempty(Data_fallen)
            plot(Data_fallen(:,1),Data_fallen(:,2),'rx','LineWidth',3);
        end
        if ~isempty(Data_fallen_part)
            plot(Data_fallen_part(:,1),Data_fallen_part(:,2),'gx','LineWidth',3);
        end
        grid;
        xlabel('Force [N]');
        ylabel('Initial Energy [J]');
        title('Protocol 1 Stability Margin Plot');
                     
        PI_plot_output = [PI_matrix; Data_fallen_part; Data_fallen];
        
        Stability_margin_PI = Data_fallen;
        
        header = ["mean peak force [N]","initial energy [J]", ...
            "mean medium force [N]","mean impulse time [s]", ...
            "Medium Impulse [Ns]"];
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
            if (Matrix_Data(i,3) >= frequency_check - 0.03) && (Matrix_Data(i,3) <= frequency_check + 0.03)
                j = j + 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
            else
                K = K + 1;
                j = 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
                frequency_check = Matrix_Data(i,3);
            end
        end
        
        for i_mas = 1:size(Data_structure,2)
            clear Matrix_Data;
            Matrix_Data = sortrows(Data_structure{i_mas},4);
            force_check = Matrix_Data(1,4);
            Kmas = 1;
            jmas = 0;
            for i_mas_2 = 1:size(Matrix_Data,1)
                if (Matrix_Data(i_mas_2,4) >= force_check - 0.1) && (Matrix_Data(i_mas_2,4) <= force_check + 0.1)
                    jmas = jmas + 1;
                    Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
                else
                    Kmas = Kmas + 1;
                    jmas = 1;
                    Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
                    force_check = Matrix_Data(i_mas_2,4);
                end
            end
        end
        
        point = 1;
        point2 = 1;
        point3 = 1;
        for i_row = 1:size(Data_structure_2,1)
            for i_col = 1:size(Data_structure_2,2)
                if ~isempty(Data_structure_2{i_row,i_col})
                    Perc_fallen = mean(Data_structure_2{i_row,i_col}(:,5));
                    Force_amplitude = mean(Data_structure_2{i_row,i_col}(:,1));
                    std_force = std(Data_structure_2{i_row,i_col}(:,1));
                    Norm_force_amplitude = mean(Data_structure_2{i_row,i_col}(:,2));
                    std_norm_force = std(Data_structure_2{i_row,i_col}(:,2));
                    frequency = mean(Data_structure_2{i_row,i_col}(:,3));
                    if Perc_fallen == 0
                        PI_matrix(point,:) = [Force_amplitude,Norm_force_amplitude,frequency, Perc_fallen,std_force,std_norm_force];
                        Exp_points(point,:) = [mean(Data_structure_2{i_row,i_col}(:,4)),frequency];
                        point = point + 1;
                    elseif Perc_fallen > 0.30
                        Data_fallen(point2,:) = [Force_amplitude,Norm_force_amplitude,frequency, Perc_fallen,std_force,std_norm_force];
                        Exp_points_fallen(point2,:) = [mean(Data_structure_2{i_row,i_col}(:,4)),frequency];
                        point2 = point2 + 1;
                    else
                        Data_fallen_part(point3,:) = [Force_amplitude,Norm_force_amplitude,frequency, Perc_fallen,std_force,std_norm_force];
                        Exp_points_fallen_part(point3,:) = [mean(Data_structure_2{i_row,i_col}(:,4)),frequency];
                        point3 = point3 + 1;
                    end
                end
            end
        end
        
        PI_plot_output = [PI_matrix; Data_fallen];
                
        figure('defaultAxesFontSize',36);
        plot(Exp_points(:,2),Exp_points(:,1),'bx','LineWidth',3);
        hold on;
        plot(Exp_points_fallen(:,2),Exp_points_fallen(:,1),'rx','LineWidth',3);
        plot(Exp_points_fallen_part(:,2),Exp_points_fallen_part(:,1),'gx','LineWidth',3);
        grid;
        xlabel('Frequency [Hz]');
        ylabel('Force [N]');
        title('Protocol 3 Stability Margin Plot');
        
        header = ["mean force sinusoidal stability margin [N]","Standard deviation force sinusoidal stability margin [N]", ...
            "mean normalized force sinusoidal stability margin [N]","Standard deviation normalized sinusoidal force stability margin [N]", ...
            "Mean frequency [Hz]", "Standard deviation on frequency [Hz]"];
        Stability_margin_PI_Plot = [header;PI_plot_output];
        Stability_margin_PI = Data_fallen;
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
            if (Matrix_Data(i,3) >= frequency_check - 0.03) && (Matrix_Data(i,3) <= frequency_check + 0.03)
                j = j + 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
            else
                K = K + 1;
                j = 1;
                Data_structure{K}(j,:) = Matrix_Data(i,:);
                frequency_check = Matrix_Data(i,3);
            end
        end
        
        for i_mas = 1:size(Data_structure,2)
            clear Matrix_Data;
            Matrix_Data = sortrows(Data_structure{i_mas},4);
            angle_check = Matrix_Data(1,4);
            Kmas = 1;
            jmas = 0;
            for i_mas_2 = 1:size(Matrix_Data,1)
                if (Matrix_Data(i_mas_2,4) >= angle_check - 1.1) && (Matrix_Data(i_mas_2,4) <= angle_check + 1.1)
                    jmas = jmas + 1;
                    Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
                else
                    Kmas = Kmas + 1;
                    jmas = 1;
                    Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
                    angle_check = Matrix_Data(i_mas_2,4);
                end
            end
        end
        
        point = 1;
        point2 = 1;
        point3 = 1;
        PI_matrix = [];
        Data_fallen = [];
        Data_fallen_part = [];
        for i_row = 1:size(Data_structure_2,1)
            for i_col = 1:size(Data_structure_2,2)
                if ~isempty(Data_structure_2{i_row,i_col})
                    Perc_fallen = mean(Data_structure_2{i_row,i_col}(:,5));
                    Displ_amplitude = mean(Data_structure_2{i_row,i_col}(:,1));
                    std_displ = std(Data_structure_2{i_row,i_col}(:,1));
                    Norm_displ_amplitude = mean(Data_structure_2{i_row,i_col}(:,2));
                    std_norm_displ = std(Data_structure_2{i_row,i_col}(:,2));
                    frequency = mean(Data_structure_2{i_row,i_col}(:,3));
                    Exp_displ = mean(Data_structure_2{i_row,i_col}(:,4));
                    if Perc_fallen == 0
                        PI_matrix(point,:) = [Displ_amplitude,Norm_displ_amplitude,frequency, Perc_fallen, Exp_displ, std_displ,std_norm_displ];
                        point = point + 1;
                    elseif Perc_fallen >= 0.3
                        Data_fallen(point2,:) = [Displ_amplitude,Norm_displ_amplitude,frequency, Perc_fallen, Exp_displ, std_displ,std_norm_displ];
                        point2 = point2 + 1;
                    else 
                        Data_fallen_part(point3,:) = [Displ_amplitude,Norm_displ_amplitude,frequency, Perc_fallen, Exp_displ, std_displ,std_norm_displ];
                        point3 = point3 + 1;
                    end
                end
            end
        end
                
        figure('defaultAxesFontSize',36);
        if ~isempty(PI_matrix)
        plot(PI_matrix(:,3),sind(PI_matrix(:,5)),'x','LineWidth',3);
        end
        hold on;
        if ~isempty(Data_fallen)
        plot(Data_fallen(:,3),sind(Data_fallen(:,5)),'rx','LineWidth',3);
        end
        if ~isempty(Data_fallen_part)
        plot(Data_fallen_part(:,3),sind(Data_fallen_part(:,5)),'gx','LineWidth',3);
        end
        grid;
        xlabel('Frequency [Hz]');
        ylabel('Displacement [m]');
        title('Protocol 2 Stability Margin Plot');
        
        PI_plot_output = [PI_matrix;Data_fallen_part;Data_fallen];
        
        header = ["mean displacement sinusoidal stability margin [m]","mean normalized displacement sinusoidal stability margin [m]", ...
            "Mean frequency [Hz]", "fall percentage","Expected Displacement [m]", "Standard deviation displacement sinusoidal stability margin [m]", ...
            "Standard deviation normalized displacement sinusoidal stability margin [m]"];
        Stability_margin_PI_Plot = [header;PI_plot_output];
        Stability_margin_PI = Data_fallen;
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

