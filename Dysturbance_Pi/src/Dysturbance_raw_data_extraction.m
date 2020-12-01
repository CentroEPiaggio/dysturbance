function [protocol_data, datapath, isfall] = Dysturbance_raw_data_extraction(filename, yaml_file, result_dir)
%--------------------------------------------------------------------------
% This function get the raw data file in format .csv and extract the raw
% data ready for post processing
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

yaml_path = strcat(result_dir,'\raw_data_input\',yaml_file);
raw_filepath = strcat(result_dir,'\raw_data_input\',filename);

if exist(yaml_path,'file')
    Yaml_data = ReadYaml(yaml_path);
    % In the yamlfile must be written the type of format and the frequency of
    % the system
    Test_protocol = Yaml_data.protocol.id;
    
    switch Test_protocol
        case 1
            Test_condition = [Yaml_data.protocol.parameters.initial_energy, Yaml_data.protocol.parameters.impact_force];
        case 2
            Test_condition = [Yaml_data.protocol.parameters.displacement_amplitude, Yaml_data.protocol.parameters.frequency, Yaml_data.protocol.parameters.cycles_number];
        case 3
            Test_condition = [Yaml_data.protocol.parameters.torque_amplitude, Yaml_data.protocol.parameters.frequency, Yaml_data.protocol.parameters.cycles_number];
        case 4
            Test_condition = Yaml_data.protocol.parameters.displacement_ramp_slope;
        case 5 
            Test_condition = Yaml_data.protocol.parameters.torque_ramp_slope;
    end
else
    error('ERROR: yaml file missing');
end

% open and process raw datas
if exist(raw_filepath, 'file')
    Data_Matrix = readtable(raw_filepath,'Format','%f%f%f%f%s%s');
    %--------------------------------------------------------------------------
    % Data_Matrix has the form M = [UTC_Time, Pendulum_position, Force_sensor, Torque_sensor];
    %--------------------------------------------------------------------------
    if isempty(Data_Matrix)
        error('ERROR: Raw Data File %s is empty. Please remove the file from folder',filename);
    end
    % The first row of Data matrix is the header, and must cut out
    Raw_Data_Matrix = Data_Matrix;
    
    %% UTC_Time
    % collect the UTC time the test is started. It is done to avoid to use
    % the same data when we compute the Global PI
    UTC_Time = Raw_Data_Matrix(1,:).UTC_time_YY_MM_DDHH_MM_SS__OFF_{1};
    %% ISfall
    % collect if the system is fall or not
    isfall = Raw_Data_Matrix(end,:).fallen_bool_{1};
    %% Experiment Time
    Time = Raw_Data_Matrix.time_s_;
    %% Encoder Pendulum position
    Pendulum_position = Raw_Data_Matrix.pendulum_position_deg_;
    %% Force sensor at the tip
    % force is measured by a piezoelectric sensor.
    Force_sensor = Raw_Data_Matrix.contact_force_N_;
    %% Torque sensor
    % Torque sensor is placed between motor and clutch. Those measurements
    % means something only if the clutch is engaged.
    Torque_sensor = Raw_Data_Matrix.pendulum_torque_Nm_;
else
    error('ERROR: Raw Data file is missing');
end

% Save important datas from Yaml
protocol_data = [Test_protocol, Test_condition];
Protocol_TIME = UTC_Time;
% Some datas, as pendulum_position and torque_sensor, requires a filtering.
% Force will be not filtered because we are interested in the peaks of the
% piezo, and doing that, we will lose it.

%--------------------------- Outliers filtering ---------------------------
% Outliers measures in torques and position of the pendulum are not
% feasible, so a filtering is necessary.
Torque_No_outliers = medfilt1(Torque_sensor,'omitnan');
Position_No_outliers = medfilt1(Pendulum_position, 'omitnan');

%----------------------- Symmetric Noise filtering ------------------------
Filter_order = 6;
Torque_filtered = smooth(Torque_No_outliers,Filter_order);
Position_filtered = smooth(Position_No_outliers,Filter_order);

%------------------ Pre - Processed Data File Creation --------------------
header =["Time_s_","PendulumAngularPosition_deg_","ContactForce_N_","OutputTorque_Nm_"];
Pre_processed_data_matrix = [header; [Time, Position_filtered, Force_sensor, Torque_filtered]];

% data must be saved in a specified folder
FILE = cellstr(filename);
index = cell2mat(strfind(FILE,"platformData")) - 1;
Pre_processed_file_name = strcat(FILE{1}(1:index),'pp_platformData.csv');
Pre_processed_data_folder = strcat(result_dir,'\Preprocessed_data');

mkdir(Pre_processed_data_folder);
datapath = strcat(Pre_processed_data_folder,'\',Pre_processed_file_name);
% creating the file in datapath
writematrix(Pre_processed_data_matrix,datapath);

end

