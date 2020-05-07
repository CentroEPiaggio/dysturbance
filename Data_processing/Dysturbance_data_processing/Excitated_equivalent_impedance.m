function [Equivalent_impedance] = Excitated_equivalent_impedance(datafile,Pendulum_length, Pendulum_mass, frequency_pendulum)
%--------------------------------------------------------------------------
% This function computes excited impedance of the system as if it is equivalent
% to a second order system.
% This KPI can be calculated on in case of sinusoidal systems. The solution
% for I,D,K is found with minimum squared regression and BAckward Euler
% Discretization
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@iit.it
%--------------------------------------------------------------------------

%% data extraction
% in the datafile must not be specified also the data folder

% folder is called as the main datafile
Data_folder = strcat('data_folder\',datafile(1:end-4),'\');

file = strcat(Data_folder,datafile);

if exist(file,'file')
    Data_matrix = csvread(file);
else
    error('ERROR: datafile does not exist');
end

UTC_Time = Data_matrix(:,1);
Pendulum_position = Data_matrix(:,2);
Force_sensor = Data_matrix(:,3);
Torque_sensor = Data_matrix(:,4);
%% 

%Sampling time 
T = 1/frequency_pendulum;

% Minimum square solution 
Y_bar = Y(3:end);
PHI = [Y(2:end-1), Y(1:end-2), U(1:end-2)];

alpha = (PHI' * PHI)^(-1) * PHI' * Y_bar;

% System is computed as I xdd + b xd + K x = u --> discretizing the laplace
% transform to z-Transform using the Backward Euler approximation we obtain

I_equivalent = T^2/alpha(3);
D_equivalent = T/alpha(3) * (alpha(1) - 2);
K_equivalent = (alpha(2)-alpha(1))/alpha(3);

Equivalent_impedance = [I_equivalent, D_equivalent, K_equivalent, frequency_pendulum];

% Data must be saved in the correct path

end

