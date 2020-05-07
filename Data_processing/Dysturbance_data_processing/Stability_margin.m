function [Stability_Margin] = Stability_margin(datafile,Protocol_number, Pendulum_length, Pendulum_mass,frequency_pendulum)
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
% mail: simone.monteleone@iit.it
%--------------------------------------------------------------------------

%% data extraction
% in the datafile must not be specified also the data folder

% folder
Data_folder = strcat('data_folder\',datafile(1:end-4),'\');

file = strcat(Data_folder,datafile);

if exist(file)
    Data_matrix = csvread(file);
else
    error('ERROR: datafile does not exist');
end

UTC_Time = Data_matrix(:,1);
Pendulum_position = Data_matrix(:,2);
Force_sensor = Data_matrix(:,3);
Torque_sensor = Data_matrix(:,4);

initial_pendulum_position = Pendulum_position(1);
%%
% Torque sensor can be used to compute the force for constant or sinusoidal
% patterns, since piezoelectric systems may have drifts
Force_indirect = Torque_sensor./Pendulum_length;
switch Protocol_number
    case 1 
        %% Protocol number 1: Impulsive disturbances test
        Stability_Margin = [max(Force_sensor), initial_pendulum_position];
    case 2 
        %% Protocol number 2: Sinusoidal force disturbances test
        Stability_Margin = [max(Force_indirect) - min(Force_indirect), frequency_pendulum];
        
    case 3 
        %% Protocol number 3: Sinusoidal displacements disturbances test
        Stability_Margin = [max(Displacement_filtered) - min(Displacement_filtered), frequency_pendulum];
    case 4 
        %% Protocol number 4: Quasi-static disturbances test
        Stability_Margin = [max(Force_sensor),0];
    otherwise
        error('ERROR: Wrong Protocol Number Inserted');
end

%% Now, data must be saved


end

