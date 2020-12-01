function [Equivalent_impedance] = Excited_equivalent_impedance(datafile, Pendulum_data, Local_PI_folder, Norm_factor)
%--------------------------------------------------------------------------
% This function computes excited impedance of the system as if it is equivalent
% to a second order system.
% This KPI can be calculated on in case of sinusoidal systems. The solution
% for I,D,K is found with minimum squared regression and BAckward Euler
% Discretization
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------
%% Pendulum_data
Pendulum_length = Pendulum_data(1);
Pendulum_height = Pendulum_data(2);
Pendulum_mass = Pendulum_data(3);
frequency_pendulum = Pendulum_data(4);
%% data extraction
% in the datafile must not be specified also the data folder

% folder is called as the main datafile
if exist(datafile,'file')
    Data_matrix = readtable(datafile);
else
    error('ERROR: datafile does not exist');
end

Time = Data_matrix.Time_s_;
Pendulum_position = Data_matrix.PendulumAngularPosition_deg_;
Force_sensor = Data_matrix.ContactForce_N_;
Torque_sensor = Data_matrix.OutputTorque_Nm_;
%% 

%Sampling time 
T = 1/frequency_pendulum;
Y = Pendulum_position;
U = Torque_sensor./Pendulum_length;
% Minimum square solution 
Y_bar = Y(3:end);
PHI = [Y(2:end-1), Y(1:end-2), U(1:end-2)];

alpha = (PHI' * PHI)^(-1) * PHI' * Y_bar;

% System is computed as I xdd + b xd + K x = u --> discretizing the laplace
% transform to z-Transform using the Backward Euler approximation we obtain

I_equivalent = T^2/alpha(3);
D_equivalent = T/alpha(3) * (alpha(1) - 2);
K_equivalent = (alpha(2)-alpha(1))/alpha(3);

header = ["Equivalent Inertia [Kg]","Equivalent Damping coefficient [Ns/m]","Equivalent Elastic coefficient [N/m]","Test frequency [Hz]"];
Equivalent_impedance = [header;[I_equivalent, D_equivalent, K_equivalent, frequency_pendulum]];

% Data must be saved in the correct path

Data_local_impedance_folder = Local_PI_folder;

FILE = cellstr(datafile);
index = cell2mat(strfind(FILE,"Preprocessed_data")) + 18;
Impedance_file_name = strcat("Equivalent_impedance_",FILE{1}(index:end));

writematrix(Equivalent_impedance,strcat(Data_local_impedance_folder,'\',Impedance_file_name));

end

