function [E_max, E_final, DE, E_perc, KPI_matrix] = Absorbed_Energy_KPI(datafile,Pendulum_data,Local_PI_folder, Norm_factor)
%--------------------------------------------------------------------------
% This function computes the absorbed energy for an impactive test.
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

%% Pedulum_data
Pendulum_length = Pendulum_data(1);
Pendulum_height = Pendulum_data(2);
Pendulum_mass = Pendulum_data(3);
frequency_pendulum = Pendulum_data(4);
%% data extraction
% in the datafile must not be specified also the data folder

if exist(datafile,'file')
    Data_matrix = readtable(datafile);
else
    error('ERROR: datafile does not exist');
end

Pendulum_position = Data_matrix.PendulumAngularPosition_deg_;
Force_sensor = Data_matrix.ContactForce_N_;
%% data
m = Pendulum_mass;
l = Pendulum_length;
grav = 9.81;        % [m/s^2] gravity acceleration
dt = 1/frequency_pendulum; % it is the number of data in one second of sampling

%% contact point
% First, we must find the point where the pendulum hits the robot. It is
% the moment where the force is maximum
[force_max,i_fmax]= find( Force_sensor == max(Force_sensor),1);
angle_cut = Pendulum_position(i_fmax -dt:i_fmax + dt);
index = round(length(angle_cut)/2);
val = angle_cut;
theta_max = max(val(1:index));
theta_2peak = max(val(index:end));

E_max = m * grav* l * (1 - cosd(theta_max));
E_final = m * grav * l * (1-cosd(theta_2peak));
DE = E_max - E_final;

E_perc = DE/ E_max;

% Normalization factor division. 1 is normalization of force, 2 is
% normalization of energy
Norm_force = Norm_factor(1);
Norm_energy = Norm_factor(2);

normalized_force_max = force_max/Norm_force;
normalized_E_max = E_max/Norm_energy;
Initial_angle = Pendulum_position(5); % maybe it is necessary to use another one (ex:10) to obtain steady state one.

% Saving Data in a CSV file
header =["E_perc","Force Max [N]","Normalized Max Force [N]","Max (Initial) Energy [J]","Normalized Max (Initial) Energy [J]","Initial Angle [deg]"];
KPI_matrix = [header;[E_perc, force_max, normalized_force_max, E_max, normalized_E_max, Initial_angle]];

% data must be saved in a specified folder
Data_local_energy_folder = Local_PI_folder;
FILE = cellstr(datafile);
index = cell2mat(strfind(FILE,"Preprocessed_data")) + 18;
Energy_file_name = strcat('Absorbed_energy_',FILE{1}(index:end));
writematrix(KPI_matrix,strcat(Data_local_energy_folder,"\",Energy_file_name));
end

