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
%% friction
%we modeled friction as a constant torque
% sperimentally, those force has been measured to be
tau_f = 2.29;
frict_perc = 0.327;

%% Computation on equilibrium displacement due to added mass
L = Pendulum_length;
M = Pendulum_mass;
d = 0.04;                                           % position of added mass respect to pendulum axis
psi_eq = atan2d(M*d,(M*L+4.13/2*L^2));  % [degree] this angle must be added to compute the correct equilibrium (not zero)
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
deg2rad = pi/180;
delta = 4.13;
grav = 9.81;        % [m/s^2] gravity acceleration
dt = 1/frequency_pendulum; % it is the number of data in one second of sampling

%% contact point
% First, we must find the point where the pendulum hits the robot. It is
% the moment where the force is maximum
i_fmax= find(Force_sensor == max(Force_sensor),1);
force_max = Force_sensor(i_fmax);
angle_cut = Pendulum_position(i_fmax - 10:i_fmax + 10);
% index = round(length(angle_cut)/2);
val = mean(angle_cut);
index = i_fmax;
theta_max = min(Pendulum_position(1:index));
theta_2peak = min(Pendulum_position(index:end));

Impact_position = mean(Pendulum_position(index:index + 10));

if theta_2peak  >= Impact_position - 0.2
    % there are cases in which the inertia of the pendulum is so much that
    % it continues its path even after hitting the robot. Find the steady
    % state position.
    theta_ss = mean(Pendulum_position(index+4000:end));
    if abs(theta_ss - Impact_position) < 0.5
        if abs((M + delta* L/2)*L*grav*sind(theta_ss)) < tau_f
            E_final = 0;
        else
            E_final = (M + delta* L/2) * grav * L * (1-cosd(theta_ss));
        end
    else
        if abs((M + delta* L/2)*L*grav*sind(Impact_position)) < tau_f
            E_final = (M + delta* L/2) * grav * L * (1-cosd(theta_ss));
        else
            E_final = 0;
        end
    end
else
    E_final = (M + delta* L/2) * grav * L * (1-cosd(theta_2peak));
end

E_max = (M + delta* L/2) * grav* L * (1 - cosd(theta_max));
% computation of friction losses
displ_ampl = (abs(theta_max -val) + abs(val-theta_2peak))*deg2rad;
E_friction = tau_f * displ_ampl;
% E_friction = frict_perc * E_max;

% Computation of the
DE = E_max - E_final - E_friction;
if DE < 0
    E_perc = 0;
else
    E_perc = DE/(E_max-E_friction);
%     if E_perc > 1
%         E_perc = 1;
%     end
end
% Normalization factor division. 1 is normalization of force, 2 is
% normalization of energy
Norm_force = Norm_factor(1);
Norm_energy = Norm_factor(2);

normalized_force_max = force_max/Norm_force;
normalized_E_max = E_max/Norm_energy;
Initial_angle = theta_max; % maybe it is necessary to use another one (ex:10) to obtain steady state one.


% Computation of impulse

    Force_start = mean(Force_sensor(i_fmax-500:i_fmax-250));
    found_start = 0;
    thres= max(4,force_max/50);
    thres = min(thres,15);
    for steps = (i_fmax-1000):i_fmax
        if Force_sensor(steps) >= Force_start + thres && found_start == 0
            step_start = steps;
            found_start = 1;
        end        
    end
    
    step_stop = i_fmax + (i_fmax -step_start);

    %initial velocity
    Position_one_start = pi/180*mean(Pendulum_position((step_start-220):(step_start-200)));
    Position_two_start = pi/180*mean(Pendulum_position((step_start-120):(step_start-100)));
    
    velocity_start = (Position_two_start-Position_one_start)/0.0100;
    
    %final velocity
    Position_one_stop = pi/180*mean(Pendulum_position((step_stop+100):(step_stop+120)));
    Position_two_stop = pi/180*mean(Pendulum_position((step_stop+200):(step_stop+220)));
    
    velocity_stop = (Position_two_stop-Position_one_stop)/0.0100;
    
    Impulse_vel = (Pendulum_mass *Pendulum_length^2 + 4.13*Pendulum_length^3/3)*(velocity_start-velocity_stop)/Pendulum_length;




% Saving Data in a CSV file
header ="[E_perc, Force Max [N], Normalized Max Force [N], Max (Initial) Energy [J], Normalized Max (Initial) Energy [J], Initial Angle [deg], Added Mass [Kg], Impulse [Ns], Absorbed Energy [J]]";
KPI_matrix = [E_perc, force_max, normalized_force_max, (E_max-E_friction), normalized_E_max, Initial_angle, M, Impulse_vel,DE];

% data must be saved in a specified folder
Data_local_energy_folder = Local_PI_folder;
FILE = cellstr(datafile);
index = cell2mat(strfind(FILE,"Preprocessed_data")) + 18;
Energy_file_name = strcat('Absorbed_energy_',FILE{1}(index:end-4),".yaml");

type = find_type(KPI_matrix);

fileID = fopen(strcat(Data_local_energy_folder,'\',Energy_file_name),'w');
fprintf(fileID,'type: %s \n',type);
fprintf(fileID, 'label: %s \n',header);
fmt = ['value: [', repmat('%g, ', 1, numel(KPI_matrix)-1), '%g]\n'];
fprintf(fileID, fmt, KPI_matrix);
fclose(fileID);

end

