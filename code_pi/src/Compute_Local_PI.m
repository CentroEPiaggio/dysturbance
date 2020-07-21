function Compute_Local_PI(filename,yaml_file, result_folder, isfall)
%--------------------------------------------------------------------------
% Computation of Local PI for the Dysturbance Test Bench
% 
% 
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

% Find the folder where to store the experiment data. It should already exists containing the input raw data
Experiment_folder = strcat('.\tests\',result_folder,'\',filename(1:end-4));

fprintf("Pre-processing Raw Datas ...  \n");
[Protocol_data, preprocessed_datapath] = Dysturbance_raw_data_extraction(filename, yaml_file, Experiment_folder);

% Experiment data
Protocol_number = str2double(Protocol_data(1));
fprintf("Extracting Structure Datas ...  \n");
[Pendulum_data, ~, Frontal_or_lateral] = Structure_data_extraction(yaml_file);

% Save the protocol number in the main folder for a check during global PI
% computation
Protocol_file_name = strcat('protocol_check.csv');
Protocol_matrix = ["Protocol number: ", Protocol_number];
writematrix(Protocol_matrix,strcat(Experiment_folder,'\',Protocol_file_name));

% Normalization factor computation. 
% The output is a vector 1x3 with [norm_force, norm_energy, norm_displacement]
fprintf("Computing Normalization factors ...  \n");
Norm_factor = normalization_factor(yaml_file, Frontal_or_lateral);

% local PI folder
Local_PI_folder = strcat(Experiment_folder,'\Local_PI');
mkdir(Local_PI_folder);
fprintf("Checking if the system fails ... \n");
check_isfall(isfall, Local_PI_folder);

fprintf("Computing Local PI ... \n");
% Local PI Computation
switch Protocol_number
case 1
	% protocol 1 is impulsive tests
	Stability_margin(preprocessed_datapath, Protocol_number, Local_PI_folder, Pendulum_data, Norm_factor);
	Absorbed_Energy_KPI(preprocessed_datapath, Pendulum_data, Local_PI_folder, Norm_factor); 
case 2
	% protocol 2 is force sinusoidal tests
	Stability_margin(preprocessed_datapath, Protocol_number, Local_PI_folder, Pendulum_data, Norm_factor);
	Excited_equivalent_impedance(preprocessed_datapath, Pendulum_data, Norm_factor);
case 3
	% protocol 3 is displacement sinusoidal tests
	Stability_margin(preprocessed_datapath, Protocol_number, Local_PI_folder, Pendulum_data, Norm_factor);
	Excited_equivalent_impedance(preprocessed_datapath, Pendulum_data, Norm_factor);
case 4
	% protocol 4 is linear ramp force test 
	Stability_margin(preprocessed_datapath, Protocol_number, Local_PI_folder, Pendulum_data, Norm_factor);
end



end

