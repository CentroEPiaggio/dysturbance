function [Pendulum_data, Pendulum_tip, Frontal_or_lateral] = Structure_data_extraction(structureyaml)
%--------------------------------------------------------------------------
% This function get the structure data from the specified yaml file, in
% order to be used in the computation of the PIs
% Pendulum_data is [Pendulum_lenght, Pendulum_height, Pendulum_mass];
% Units are [m] for pendulum lenght and height, and Kg for pendulum_mass
%  - Pendulum_length is the distance from tip to pendulum rotation axis
%  - Pendulum_height is the distance of the pendulum rotation axis to the
%  ground;
%  - Pendulum mass is the added mass of the pendulum, plus the mass of the
%  bar itself, computed using linear density and lenght
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

% retrieve pendulum data from yaml
data_structure = ReadYaml(structureyaml);

Pendulum_length = data_structure.pendulum.length;
Pendulum_added_mass = data_structure.pendulum.added_mass;
Pendulum_height = data_structure.pendulum.axis_height;
Pendulum_tip = data_structure.pendulum.tip_type;
sampling_frequency = data_structure.sampling_frequency;
%------------------------- Total mass computation -------------------------
Linear_density = 4.13; %[kg/m] linear density of the pendulum bar
Bar_mass = Linear_density * Pendulum_length;

Pendulum_mass = Bar_mass + Pendulum_added_mass;

%store pendulum data in a vector
Pendulum_data = [Pendulum_length, Pendulum_height, Pendulum_added_mass, sampling_frequency];

Orientation = data_structure.subject.orientation;
if Orientation == 0
    Frontal_or_lateral = "Frontal";
else 
    Frontal_or_lateral = "Lateral";
end
end
