function Compute_Local_PI(filename,yaml_file, subject_yaml, result_folder)
%--------------------------------------------------------------------------
% Computation of Local PI for the Dysturbance Test Bench
%
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

% Find the folder where to store the experiment data. It should already exists containing the input raw data

warning('OFF', 'MATLAB:table:ModifiedAndSavedVarnames');

FILENAME = cellstr(filename);
index = cell2mat(strfind(FILENAME,filesep));
name_of_file = FILENAME{1}(index(end)+1:end);
% Experiment_folder = fullfile(result_folder,name);
fprintf("Pre-processing Raw Data ...  \n");
[Protocol_data, preprocessed_datapath, Output_folder, isfall, Protocol_TIME, flag_isempty] = Dysturbance_raw_data_extraction(filename, yaml_file, result_folder);

if flag_isempty == 0
    % Experiment data
    Protocol_number = Protocol_data(1);
    fprintf("Extracting Data Structures ...  \n");
    %yaml_data_position = fullfile(Experiment_folder,'raw_data_input',yaml_file);
    [Pendulum_data, ~, Frontal_or_lateral] = Structure_data_extraction(yaml_file,subject_yaml);
    
    % Save the protocol number in the main folder for a check during global PI
    % computation
    Protocol_file_name = "protocol_check.csv";
    Protocol_matrix = ["Protocol_number"; Protocol_number];
    writematrix(Protocol_matrix,fullfile(Output_folder,'Preprocessed_data',Protocol_file_name));
    
    % Normalization factor computation.
    % The output is a vector 1x3 with [norm_force, norm_energy, norm_displacement]
    fprintf("Computing Normalization Factors ...  \n");
    Norm_factor = normalization_factor(subject_yaml, Frontal_or_lateral);
    
    % local PI folder
    Local_PI_folder = fullfile(Output_folder,'Local_PI');
    if ~exist(Local_PI_folder, 'dir')
        mkdir(Local_PI_folder);
    end
    fprintf("Checking if the system fails ... \n");
    check_isfall(isfall, name_of_file, Local_PI_folder);
    
    fprintf("Computing Local PI ... \n");
    % Local PI Computation
    switch Protocol_number
        case 1
            % protocol 1 is impulsive tests
            Stability_margin(preprocessed_datapath, Local_PI_folder, Pendulum_data, Protocol_data, Norm_factor);
            Absorbed_Energy_KPI(preprocessed_datapath, Pendulum_data, Local_PI_folder, Norm_factor);
        case 2
            % protocol 2 is displacement sinusoidal tests
            Stability_margin(preprocessed_datapath, Local_PI_folder, Pendulum_data, Protocol_data, Norm_factor);
            Excited_equivalent_impedance(preprocessed_datapath, Pendulum_data, Local_PI_folder, Protocol_data, Norm_factor, Protocol_number);
        case 3
            % protocol 3 is force sinusoidal tests
            Stability_margin(preprocessed_datapath, Local_PI_folder, Pendulum_data, Protocol_data, Norm_factor);
            Excited_equivalent_impedance(preprocessed_datapath, Pendulum_data, Local_PI_folder, Protocol_data, Norm_factor, Protocol_number);
        case 4
            % protocol 4 is linear ramp displacement test
            Stability_margin(preprocessed_datapath, Local_PI_folder, Pendulum_data, Protocol_data, Norm_factor);
        case 5
            % protocol 5 is linear ramp force test
            Stability_margin(preprocessed_datapath, Local_PI_folder, Pendulum_data, Protocol_data, Norm_factor);
    end
    
    
end
end

