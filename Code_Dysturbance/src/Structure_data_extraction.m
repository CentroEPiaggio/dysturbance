function [Pendulum_data, Pendulum_tip] = Structure_data_extraction(structureyaml)
%--------------------------------------------------------------------------
% This function get the structure data from the specified yaml file, in
% order to be used in the computation of the PIs
% Pendulum_data is [Pendulum_lenght, Pendulum_height, Pendulum_mass];
% Units are [m] for pendulum lenght and height, and Kg for pendulum_mass
%  - Pendulum_lenght is the distance from tip to pendulum rotation axis
%  - Pendulum_height is the distance of the pendulum rotation axis to the
%  ground;
%  - Pendulum mass is the added mass of the pendulum, plus the mass of the
%  bar itself, computed using linear density and lenght
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@iit.it
%--------------------------------------------------------------------------

% retrieve pendulum data from yaml
data_structure = ReadYaml(structureyaml);

Pendulum_length = str2double(data_structure.lenght);
Pendulum__added_mass = str2double(data_structure.added_mass);
Pendulum_height = str2double(data_structure.axis_height);
Pendulum_tip = data_structure.tip_type;
% if sinusoidal 
test_frequency = data_structure.param_1;
% sensor Frequencies
Encoder_frequency = data_structure.encoder_sampling_frequency;
Force_frequency = data_structure.force_sampling_frequency;
Torque_frequency = data_structure.torque_sampling_frequency;

%------------------------- Total mass computation -------------------------
Linear_density = 4.13; %[kg/m] linear density of the pendulum bar
Joint_mass = 0; % [kg] mass of the component at the rotation joint
Joint_distance = 0.02; %[m] the bar is connected to the pendulum at a certain distance from the axis
Bar_mass = 4.13 * (Pendulum_length - Joint_distance) + Joint_mass;

Pendulum_mass = Bar_mass + Pendulum__added_mass;

%store pendulum data in a vector
Pendulum_data = [Pendulum_length, Pendulum_height, Pendulum_mass, test_frequency, Encoder_frequency, Force_frequency, Torque_frequency];

end
