function [protocol_data, datapath, Output_folder, isfall, Protocol_TIME, flag_isempty] = Dysturbance_raw_data_extraction(filename, yaml_file, result_dir)
%--------------------------------------------------------------------------
% This function get the raw data file in format .csv and extract the raw
% data ready for post processing
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------
%% Data
grav = 9.81;            % [m/s^2] gravity coefficient
delta = 4.13;           % [Kg/m] linear density of the pendulum

%% data extraction from raw files

yaml_path = yaml_file;%strcat(result_dir,'\raw_data_input\',yaml_file);
raw_filepath = filename;%strcat(result_dir,'\raw_data_input\',filename);
flag_yaml_empty = 0;
if exist(yaml_path,'file')
    Yaml_data = ReadYaml(yaml_path);
    % In the yamlfile must be written the type of format and the frequency of
    % the system
    Test_protocol = Yaml_data.protocol.id;
    added_mass = Yaml_data.pendulum.added_mass;
    pend_length = Yaml_data.pendulum.length;
    switch Test_protocol
        case 1
            init_E = (added_mass + delta * pend_length/2)*grav*pend_length*(1-cosd(Yaml_data.protocol.parameters.initial_upper_position));
            exp_F = 1/0.01*sqrt(2*0.67*(added_mass*pend_length^2 + delta*pend_length^3/3)*init_E)/pend_length;
            Test_condition = [Yaml_data.protocol.parameters.initial_upper_position, added_mass, init_E, exp_F];
        case 2
            Test_condition = [Yaml_data.protocol.parameters.displacement_amplitude, Yaml_data.protocol.parameters.frequency, Yaml_data.protocol.parameters.cycles_number];
        case 3
            Test_condition = [Yaml_data.protocol.parameters.force_amplitude, Yaml_data.protocol.parameters.frequency, Yaml_data.protocol.parameters.cycles_number];
        case 4
            Test_condition = Yaml_data.protocol.parameters.displacement_ramp_slope;
        case 5
            Test_condition = Yaml_data.protocol.parameters.force_ramp_slope;
    end
else
    fprintf('ERROR: yaml file %s missing \n ',yaml_path);
    flag_yaml_empty = 1;
end

% open and process raw datas
if exist(raw_filepath, 'file') && (flag_yaml_empty ~= 1)
    Data_Matrix = readtable(raw_filepath,'Format','%f%f%f%f%s%s');
    %--------------------------------------------------------------------------
    % Data_Matrix has the form M = [UTC_Time, Pendulum_position, Force_sensor, Torque_sensor];
    %--------------------------------------------------------------------------
    if isempty(Data_Matrix)
        %         delete(raw_filepath);
        fprintf('ERROR: Raw Data File %s is empty. Computations are avoided for this file... \n',filename);
        flag_isempty = 1;
    else
        % if the csv file contains less than 0.5 seconds of data, then
        % something is not working. To avoid computation of PI with a wrong
        % set of data, we neglect potential errors.
        if size(Data_Matrix,1)<= 5000
            flag_isempty = 1;
            fprintf('Error:Raw Data File %s contains less than 0.5 seconds records of data... \n',filename);
        else
            flag_isempty = 0;
        end
    end
else
    fprintf('Error:Raw Data File %s  is missing... \n',filename);
    flag_isempty = 1;
end

if flag_isempty == 0
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
    Time = Raw_Data_Matrix(2:end-1,:).time_s_;
    %% Encoder Pendulum position
    Pendulum_position = Raw_Data_Matrix(2:end-1,:).pendulum_position_deg_;
    %% Force sensor at the tip
    % force is measured by a piezoelectric sensor.
    Force_sensor = Raw_Data_Matrix(2:end-1,:).contact_force_N_;
    %% Torque sensor
    % Torque sensor is placed between motor and clutch. Those measurements
    % means something only if the clutch is engaged.
    Torque_sensor = Raw_Data_Matrix(2:end-1,:).pendulum_torque_Nm_;
    
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
    Filter_order_position = 100;
    Filter_order_torque = 5000;
    Torque_filtered = smooth(Torque_No_outliers,Filter_order_torque);
    Position_filtered = smooth(Position_No_outliers,Filter_order_position);
    
    %---------------------------- Gravity Rejection ---------------------------
    
    % DA CANCELLARE!!!
    if Position_filtered(1,1) > 6
        Position_filtered = Position_filtered - Position_filtered(1,1);
    end
    
    for i = 1:size(Torque_filtered,1)
        if mean(Position_filtered(1:100)) < 8
            Torque_No_grav(i,1) = Torque_filtered(i,1) - (added_mass * pend_length + delta*pend_length * pend_length/2)*grav*sind(Position_filtered(i,1));
        else
            % the pendulum would not start in a position that is higher
            % than 8 degrees. There may be some errors in collecting data.
            % Initial Position have been adjusted.
            Torque_No_grav(i,1) = Torque_filtered(i,1) - (added_mass * pend_length + delta*pend_length * pend_length/2)*grav*sind(Position_filtered(i,1) - mean(Position_filtered(1:100)));
        end
    end
    %------------------ Pre - Processed Data File Creation --------------------
    header =["Time_s_","PendulumAngularPosition_deg_","ContactForce_N_","OutputTorque_Nm_"];
    Pre_processed_data_matrix = [header; [Time, Position_filtered, Force_sensor, Torque_No_grav]];
    
    % data must be saved in a specified folder
    FILE = cellstr(filename);
    index_1 = cell2mat(strfind(FILE,"raw_data_input")) - 1;
    preprocessed_folder = strcat(result_dir, FILE{1}(index_1-26:index_1));
    
    index_2 = cell2mat(strfind(FILE,"platformData")) - 1;
    Pre_processed_file_name = strcat(FILE{1}(index_1+16:index_2),'pp_platformData.csv');
    Pre_processed_data_folder = strcat(preprocessed_folder,'Preprocessed_data');
    Output_folder = preprocessed_folder;
    mkdir(Pre_processed_data_folder);
    datapath = strcat(Pre_processed_data_folder,'\',Pre_processed_file_name);
    % creating the file in datapath
    writematrix(Pre_processed_data_matrix,datapath);
else
    protocol_data = 0;
    Protocol_TIME = 0;
    datapath = [];
    isfall = 1;
end

end

