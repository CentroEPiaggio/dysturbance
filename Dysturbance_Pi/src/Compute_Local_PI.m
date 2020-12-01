function Compute_Local_PI(filename,yaml_file, result_folder)
%--------------------------------------------------------------------------
% Computation of Local PI for the Dysturbance Test Bench
% 
% 
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

% Find the folder where to store the experiment data. It should already exists containing the input raw data

FILENAME = cellstr(filename);
index = cell2mat(strfind(FILENAME,"run")) - 2;
name = FILENAME{1}(1:index);
Experiment_folder = strcat("tests\",result_folder,"\",name);

fprintf("Pre-processing Raw Datas ...  \n");
[Protocol_data, preprocessed_datapath, isfall] = Dysturbance_raw_data_extraction(filename, yaml_file, Experiment_folder);

% Experiment data
Protocol_number = Protocol_data(1);
fprintf("Extracting Structure Datas ...  \n");
yaml_data_position = strcat(Experiment_folder,'\raw_data_input\',yaml_file);
[Pendulum_data, ~, Frontal_or_lateral] = Structure_data_extraction(yaml_data_position);

% Save the protocol number in the main folder for a check during global PI
% computation
Protocol_file_name = "protocol_check.csv";
Protocol_matrix = ["Protocol_number"; Protocol_number];
writematrix(Protocol_matrix,strcat(Experiment_folder,'\',Protocol_file_name));

% Normalization factor computation. 
% The output is a vector 1x3 with [norm_force, norm_energy, norm_displacement]
fprintf("Computing Normalization factors ...  \n");
Norm_factor = normalization_factor(yaml_data_position, Frontal_or_lateral);

% local PI folder
Local_PI_folder = strcat(Experiment_folder,"\Local_PI");
mkdir(Local_PI_folder);
fprintf("Checking if the system fails ... \n");
check_isfall(isfall, filename, Local_PI_folder);

fprintf("Computing Local PI ... \n");
% Local PI Computation
switch Protocol_number
case 1
	% protocol 1 is impulsive tests
	Stability_margin(preprocessed_datapath, Protocol_number, Local_PI_folder, Pendulum_data, Norm_factor);
	Absorbed_Energy_KPI(preprocessed_datapath, Pendulum_data, Local_PI_folder, Norm_factor); 
case 2
	% protocol 2 is displacement sinusoidal tests
	Stability_margin(preprocessed_datapath, Protocol_number, Local_PI_folder, Pendulum_data, Norm_factor);
	Excited_equivalent_impedance(preprocessed_datapath, Pendulum_data, Local_PI_folder, Norm_factor);
case 3
	% protocol 3 is force sinusoidal tests
	Stability_margin(preprocessed_datapath, Protocol_number, Local_PI_folder, Pendulum_data, Norm_factor);
	Excited_equivalent_impedance(preprocessed_datapath, Pendulum_data, Local_PI_folder, Norm_factor);
case 4
	% protocol 4 is linear ramp displacement test 
	Stability_margin(preprocessed_datapath, Protocol_number, Local_PI_folder, Pendulum_data, Norm_factor);
case 5
	% protocol 5 is linear ramp force test 
	Stability_margin(preprocessed_datapath, Protocol_number, Local_PI_folder, Pendulum_data, Norm_factor);
end



end

