function [E_max, E_final, DE, E_perc, KPI_matrix] = Absorbed_Energy_KPI(datafile, Pendulum_mass, Pendulum_length, frequency_force, frequency_pendulum)
%--------------------------------------------------------------------------
% This function computes the absorbed energy for an impactive test.
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

if exist(file,'file')
    Data_matrix = csvread(file);
else
    error('ERROR: datafile does not exist');
end

% UTC_Time = Data_matrix(:,1);
Pendulum_position = Data_matrix(:,2);
Force_sensor = Data_matrix(:,3);
% Torque_sensor = Data_matrix(:,4);
%% data
m = Pendulum_mass;
l = Pendulum_length;
grav = 9.81;        % [m/s^2] gravity acceleration
Frequency_ratio = frequency_force/frequency_pendulum;
dt = 1/frequency_pendulum; % it is the number of data in one second of sampling

%% contact point
% First, we must find the point where the pendulum hits the robot. It is
% the moment where the force is maximum
[force_max,i_fmax]= find( Force_sensor == max(Force_sensor),1);
angle_cut = Pendulum_position(floor(i_fmax/Frequency_ratio) -dt:floor(i_fmax/Frequency_ratio) + dt);
index = round(length(angle_cut)/2);
val = angle_cut;
theta_max = max(val(1:index));
theta_2peak = max(val(index:end));

E_max = m * grav* l * (1 - cosd(theta_max));
E_final = m * grav * l * (1-cosd(theta_2peak));
DE = E_max - E_final;

E_perc = DE/ E_max;

% Now, the data must be saved
Initial_angle = Pendulum_position(1); % maybe it is necessary to use another one (ex:10) to obtain steady state one.
KPI_matrix = [E_perc, force_max, Initial_angle];
end

