function [protocol_data, datapath] = Dysturbance_raw_data_extraction(filename, yaml_file, result_dir)
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
    Test_protocol = str2double(Yaml_data.name);
    Test_time = Yaml_data.time;
    Filter_order = str2double(Yaml_data.filter_order);
%------------------ extract pendulum frequency from txt -------------------
    Frequency = str2double(Yaml_data.sampling_frequency);
    
%     frequency_pendulum = 0;
% 
%     for i = 1:(size(rows(2,:),2)-1)
%         [value, logical] = str2num(rows(2,i));
%         if logical == 1
%             frequency_pendulum = str2double(strcat(num2str(frequency_pendulum), num2str(value)));
%         end
%     end
% 
% %-------------------- extract force frequency from txt --------------------
%     Frequency_force = 0;
%     clear value logical;
%     for i = 1:(size(rows(3,:),2)-1)
%         [value, logical] = str2num(rows(3,i));
%         if logical == 1
%             Frequency_force = str2double(strcat(num2str(Frequency_force), num2str(value)));
%         end
%     end
% %------------------- extract torque frequency from txt --------------------
%     frequency_torque = 0;
%     clear value logical;
%     for i = 1:(size(rows(4,:),2)-1)
%         [value, logical] = str2num(rows(4,i));
%         if logical == 1
%             frequency_torque = str2double(strcat(num2str(frequency_torque), num2str(value)));
%         end
%     end
%     
%--------------------------------------------------------------------------    
% % Now we extract the data from the csv
%     
%     if format ~= 'csv'
%         error('ERROR: data format is invalid');
%     end
else
    error('ERROR: yaml file missing');
end
protocol_data = [Test_protocol, Test_time, Frequency];
PiezoVolt2Newton = 4448/10;     % Sensor  has a max volt of 10VDC and a max force of 4448 N]   
tick2degrees = 360/16384;       % Sensor is a 14 Bit encoder. It has 16,384 tick per revolution
Volt2Nm = 500/5;                % [Sensor has a max Volt of 5 VDC and a max torque output of 500 Nm]
if exist(raw_filepath, 'file')
    Data_Matrix = csvread(raw_filepath);
    %--------------------------------------------------------------------------
    % Data_Matrix has the form M = [header;[UTC_Time, Pendulum_position, Force_sensor, Torque_sensor]];
    %--------------------------------------------------------------------------
    if isempty(Data_Matrix)
        error('ERROR: Raw Data Files is empty');
    end
    % The first row of Data matrix is the header, and must cut out
    Raw_Data_Matrix = Data_Matrix(2:end,:);
    %% UTC_Time
    UTC_Time = Raw_Data_Matrix(:,1);
    %% Encoder Pendulum position
    Pendulum_position = tick2degrees * Raw_Data_Matrix(:,2);
    %% Force sensor at the tip
    % force is measured by a piezoelectric sensor. An Adjustement to pass to N
    % has to be done
    Force_sensor = PiezoVolt2Newton * Raw_Data_Matrix(:,3);
    %% Torque sensor
    Torque_sensor = Volt2Nm * Raw_Data_Matrix(:,4);
else
    error('ERROR: Raw Data file is missing');
end

% Some datas, as pendulum_position and torque_sensor, requires a filtering.
% Force will be not filtered because we are interested in the peaks of the
% piezo, and doing that, we will lose it.

%--------------------------- Outliers filtering ---------------------------
% Outliers measures in torques and position of the pendulum are not
% feasible, so a filtering is necessary.
Torque_No_outliers = medfilt1(Torque_sensor,'omitnan');
Position_No_outliers = medfilt1(Pendulum_position, 'omitnan');

%---------------------------- Noise filtering -----------------------------
% Design of a filter - Filter is not symmetric
% Fs = frequency;
% FILTER = designfilt('lowpassfir','FilterOrder',Filter_order, ... 
%     'CutoffFrequency',Cut_off_frequency,'DesignMethod','window', ...
%     'Window',{@kaiser,3},'SampleRate',Fs);
% 
% Torque_filtered = filter(FILTER, Torque_No_outliers);
% Position_filtered = filter(FILTER, Position_No_outliers);

%----------------------- Symmetric Noise filtering ------------------------
Filter_order = 6;
Torque_filtered = smooth(Torque_No_outliers,Filter_order);
Position_filtered = smooth(Position_No_outliers,Filter_order);

%------------------ Pre - Processed Data File Creation --------------------
header =['UTC Time [ms]','Pendulum Angular Position [deg]','Contact Force [N]','Output Torque [Nm]'];
Pre_processed_data_matrix = [header; [UTC_Time, Position_filtered, Force_sensor, Torque_filtered]];

% data must be saved in a specified folder
index = strfind(filename,"datafile") - 1;
Pre_processed_file_name = strcat(filename(1:index),'platformData.csv');

Pre_processed_data_folder = strcat(result_dir,'\Preprocessed_data');

mkdir(Pre_processed_data_folder);
datapath = strcat(Pre_processed_data_folder,'\',Pre_processed_file_name);
writematrix(Pre_processed_data_matrix, datapath);

end

