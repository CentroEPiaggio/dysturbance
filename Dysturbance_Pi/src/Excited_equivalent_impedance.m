function [Equivalent_impedance] = Excited_equivalent_impedance(datafile, Pendulum_data, Local_PI_folder, protocol_data, Norm_factor, Protocol)
%--------------------------------------------------------------------------
% This function computes excited impedance of the system as if it is equivalent
% to a second order system.
% This KPI can be calculated on in case of sinusoidal systems. The solution
% for I,D,K is found with minimum squared regression and Backward Euler
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
frequency_sampling = Pendulum_data(4);
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
% protocol data
frequency_pendulum = protocol_data(3);
amplitude = protocol_data(2);
%Sampling time
T = 1/frequency_sampling;
% Y = Pendulum_position;
% U = Torque_sensor./Pendulum_length;
% sample = 200;
% for i = 1:floor(size(Pendulum_position,1)/sample)
%     % enhancing the filtering to reduce the noise on velocity and
%     % acceleration
%     Y(i,1) = mean(Pendulum_position((sample*(i-1) + 1):sample*i));
%     U(i,1) = mean(Torque_sensor((sample*(i-1) + 1):sample*i)./Pendulum_length);
% end
Filter_order_position = 5000;
Filter_order_torque = 5000;
Y = smooth(Pendulum_length .* sind(Pendulum_position(2000:end-2000)), Filter_order_position);
U = smooth(Torque_sensor(2000:end-2000)./Pendulum_length, Filter_order_torque);


vdiff = diff([Y;0])/T;
%
Y = Y(100:end-100);
U = U(100:end-100);
vdiff = vdiff(100:end-100);
V = smooth(vdiff, 1000);
adiff = diff([V;0])/T;
%
Y = Y(100:end-100);
U = U(100:end-100);
V = V(100:end-100);
A = adiff(100:end-100);


Y_bar = [ A, V, Y];

PHI = inv(Y_bar' * Y_bar)*Y_bar';

PI = PHI * U;

I_equivalent = abs(PI(1));
D_equivalent = abs(PI(2));
K_equivalent = abs(PI(3));
% % Minimum square solution
% Y_bar = Y(3:end);
% PHI = [Y(2:end-1), Y(1:end-2), U(1:end-2)];
%
% alpha = (PHI' * PHI)^(-1) * PHI' * Y_bar;

% System is computed as I xdd + b xd + K x = u
% I_equivalent = T^2/alpha(3);
% D_equivalent = T/alpha(3) * (alpha(1) - 2);
% K_equivalent = (alpha(2)-alpha(1))/alpha(3);

if Protocol == 2
    header = "[Equivalent Inertia [Kg], Equivalent Damping coefficient [Ns/m], Equivalent Elastic coefficient [N/m], Test frequency [Hz], Displacement Amplitude [degrees]]";
else
    header = "[Equivalent Inertia [Kg], Equivalent Damping coefficient [Ns/m], Equivalent Elastic coefficient [N/m], Test frequency [Hz], Force Amplitude [N]]";
end
Equivalent_impedance = [I_equivalent, D_equivalent, K_equivalent, frequency_pendulum, amplitude];

% Data must be saved in the correct path

Data_local_impedance_folder = Local_PI_folder;

FILE = cellstr(datafile);
index = cell2mat(strfind(FILE,"Preprocessed_data")) + 18;

% Impedance_file_name = strcat("Equivalent_impedance_",FILE{1}(index:end-4),".yaml");
Impedance_file_name = strcat("Equivalent_impedance.yaml");

type = find_type(Equivalent_impedance);

fileID = fopen(fullfile(Data_local_impedance_folder,Impedance_file_name),'w');

fprintf(fileID,'type: %s \n',type);
fprintf(fileID, 'label: %s \n',header);
fmt = ['value: [', repmat('%g, ', 1, numel(Equivalent_impedance)-1), '%g]\n'];
fprintf(fileID, fmt, Equivalent_impedance);
fclose(fileID);

end

