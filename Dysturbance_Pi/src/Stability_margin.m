function [Stability_Margin] = Stability_margin(datafile,Protocol_number, Data_folder, Pendulum_data, Norm_factor)
%--------------------------------------------------------------------------
% This function computes the Stability Margin of a test, depending on the Protocol of test.
% - If it is impulsive, it returns only the test margin (aka the maximum
%   exerted force). The impulsive stability margin for impulsive tests is
%   the maximum force exerted before going into the instability area.
% - If it is sinusoidal, it returns only the test margin (aka the maximum
%   exerted amplitude) and the frequency the test has been done. The stability margin is the maximum amplitude
%   before the robot falls.
% - If it is linear, the function returns the stability margin of the
%   robot, as the maximum exerted force before the system falls.
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@@phd.unipi.it
%--------------------------------------------------------------------------
%% Pendulum_data
Pendulum_length = Pendulum_data(1);
Pendulum_height = Pendulum_data(2);
Pendulum_mass = Pendulum_data(3);
frequency_pendulum = Pendulum_data(4);
%% data extraction
% in the datafile must not be specified also the data folder

if exist(datafile)
    Data_matrix = readtable(datafile);
else
    error('ERROR: datafile does not exist');
end

Time = Data_matrix.Time_s_;
Pendulum_position = Data_matrix.PendulumAngularPosition_deg_;
Force_sensor = Data_matrix.ContactForce_N_;
Torque_sensor = Data_matrix.OutputTorque_Nm_;

initial_pendulum_position = Pendulum_position(1);
%%
% Torque sensor can be used to compute the force for constant or sinusoidal
% patterns, since piezoelectric systems may have drifts
Force_indirect = Torque_sensor./Pendulum_length;

Norm_force = Norm_factor(1);
Norm_displacement = Norm_factor(3);

switch Protocol_number
    case 1 
        %% Protocol number 1: Impulsive disturbances test
        header = ["Local Stability Margin [N]","Normalized Local Stability Margin [N]","Initial Pendulum Position [deg]"];
        Stability_Margin = [header;[max(Force_sensor), max(Force_sensor)/Norm_force, initial_pendulum_position]];
    case 3 
        %% Protocol number 3: Sinusoidal force disturbances test
        header = ["Local Stability Margin [N]","Normalized Local Stability Margin [N]","Test_frequency [Hz]"];
        Stability_Margin = [header;[(max(Force_indirect) - min(Force_indirect)),(max(Force_indirect) - min(Force_indirect))/Norm_force , frequency_pendulum]];
        
    case 2 
        %% Protocol number 2: Sinusoidal displacements disturbances test
        header = ["Local Stability Margin [m]","Normalized Local Stability Margin [m]","Test_frequency [Hz]"];
        Stability_Margin = [header;[(max(Pendulum_position) - min(Pendulum_position)),(max(Pendulum_position) - min(Pendulum_position))/Norm_displacement, frequency_pendulum]];
    case 4 
        %% Protocol number 4: Quasi-static displacement disturbances test
        header = ["Local Stability Margin [N]","Normalized Local Stability Margin [N]","empty"];
        Stability_Margin = [header;[max(Force_sensor), max(Force_sensor)/Norm_force,0]];
    case 5
         %% Protocol number 5: Quasi-static force disturbances test
        header = ["Local Stability Margin [N]","Normalized Local Stability Margin [N]","empty"];
        Stability_Margin = [header;[max(Pendulum_position), max(Pendulum_position)/Norm_displacement,0]];
    otherwise
        error('ERROR: Wrong Protocol Number Inserted');
end


% data must be saved in a specified folder
Data_local_stability_folder = Data_folder;

% find name from datafile
FILE = cellstr(datafile);
index = cell2mat(strfind(FILE,"Preprocessed_data")) + 18;
Stability_file_name = strcat("Stability_margin_",FILE{1}(index:end));
writematrix(Stability_Margin,strcat(Data_local_stability_folder,'\',Stability_file_name));



end

