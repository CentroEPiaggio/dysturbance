function [UTC_Time, Position_filtered, Force_sensor, Torque_filtered, Frequency_force,frequency_torque, frequency_pendulum, name_experiment] = Dysturbance_raw_data_extraction(filename, txtfile, Filter_order, Cut_off_frequency)
%--------------------------------------------------------------------------
% This function get the raw data file in format .csv and extract the raw
% data ready for post processing
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@iit.it
%--------------------------------------------------------------------------
if exist(txtfile,'file')
    filereading = fopen(txtfile,'r');
    % In the txtfile must be written the type of format and the frequency of
    % the system
    DATA = fscanf(filereading, '%c');
    k = 1;
    j = 1;
    for i = 1:size(DATA,2)
        if DATA(1,i) == newline
            k = k + 1;
            j = 1;
        else
            rows(k,j) = DATA(1,i);
            j = j + 1;
        end
    end
    
    name_experiment = rows(1,12:(size(rows(1,:),2)-1));
    format = rows(5,13:15);
    
%------------------ extract pendulum frequency from txt -------------------
    frequency_pendulum = 0;

    for i = 1:(size(rows(2,:),2)-1)
        [value, logical] = str2num(rows(2,i))
        if logical == 1
            frequency_pendulum = str2double(strcat(num2str(frequency_pendulum), num2str(value)));
        end
    end

%-------------------- extract force frequency from txt --------------------
    Frequency_force = 0;
    clear value logical;
    for i = 1:(size(rows(3,:),2)-1)
        [value, logical] = str2num(rows(3,i));
        if logical == 1
            Frequency_force = str2double(strcat(num2str(Frequency_force), num2str(value)));
        end
    end
%------------------- extract torque frequency from txt --------------------
    frequency_torque = 0;
    clear value logical;
    for i = 1:(size(rows(4,:),2)-1)
        [value, logical] = str2num(rows(4,i));
        if logical == 1
            frequency_torque = str2double(strcat(num2str(frequency_torque), num2str(value)));
        end
    end
    
%--------------------------------------------------------------------------    
% Now we extract the data from the csv
    
    if format ~= 'csv'
        error('ERROR: data format is invalid');
    end
else
    error('ERROR: text file missing');
end
PiezoVolt2Newton = 1;
tick2degrees = 1;
Volt2Nm = 1;
if exist(filename, 'file')
    Raw_Data_Matrix = csvread(filename);
    %--------------------------------------------------------------------------
    % Raw_Data_Matrix has the form M = [UTC_Time, Pendulum_position, Force_sensor, Torque_sensor];
    %--------------------------------------------------------------------------
    if isempty(Raw_Data_Matrix)
        error('ERROR: Raw Data Files is empty');
    end
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
% Design of a filter
Fs = frequency;
FILTER = designfilt('lowpassfir','FilterOrder',Filter_order, ... 
    'CutoffFrequency',Cut_off_frequency,'DesignMethod','window', ...
    'Window',{@kaiser,3},'SampleRate',Fs);

Torque_filtered = filter(FILTER, Torque_No_outliers);
Position_filtered = filter(FILTER, Position_No_outliers);

%------------------ Pre - Processed Data File Creation --------------------
header =['UTC Time [ms]','Pendulum Angular Position [deg]','Contact Force [N]','Output Torque [Nm]'];
Pre_processed_data_matrix = [header; [UTC_Time, Position_filtered, Force_sensor, Torque_filtered]];

% data must be saved in a specified folder
Pre_processed_file_name = strcat(filename(1:end-4),'_PP_f_', ...
    num2str(Cut_off_frequency),'_F_o_',num2str(Filter_order),'.csv');

Pre_processed_data_folder = strcat('data_folder\',Pre_processed_file_name(1:end-4));

mkdir(Pre_processed_data_folder);

writematrix(Pre_processed_data_matrix,strcat(Pre_processed_data_folder,'\',Pre_processed_file_name));
end

