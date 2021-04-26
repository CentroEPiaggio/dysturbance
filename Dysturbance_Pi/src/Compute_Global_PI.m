function Compute_Global_PI(data_folder, Protocol, result_folder)
%--------------------------------------------------------------------------
% Computation of the Global PI for Dysturbance Bench test
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

warning('OFF', 'MATLAB:table:ModifiedAndSavedVarnames');

Protocol = str2double(Protocol); % it is parsed as a string

% creation of the folder to store the Global PI data
Protocol_folder = strcat('protocol_',num2str(Protocol));
FILE = cellstr(data_folder);
index = cell2mat(strfind(FILE,filesep));
name_of_subject = FILE{1}(index(end)+1:end);

Global_PI_folder = fullfile(result_folder,name_of_subject,Protocol_folder,'Global_PIs');
if ~exist(Global_PI_folder, 'dir')
    mkdir(Global_PI_folder)
end

% Now we must retrieve the local PI data from each experiment folder
output_folders = fullfile(result_folder,name_of_subject,Protocol_folder);
Experiment_folder = fullfile(data_folder,Protocol_folder);

% in Test_Data there are n folders related to n experiments. Each one of them contains a raw data folder and a local PI folder

% move into the Experiment data directory
Old_Folder = cd(Experiment_folder);

fprintf("Searching for experiments folders ... \n");
% count the number of folders.
all_files = dir;
all_dir = all_files([all_files(:).isdir]);
num_dir = numel(all_dir);

% Removing virtual folders
j = 0;
Tests_folders = {};
for i = 1:num_dir
    if all_dir(i).name ~= "."  &&  all_dir(i).name ~= ".." &&  all_dir(i).name ~= "Global_PIs"
        j = j + 1;
        Tests_folders(j,1) = {fullfile(Experiment_folder,all_dir(i).name)};
    end
end
% Come Back at the starting folder
cd(Old_Folder);

% find subject info
Subject_folder = data_folder;
fprintf("Searching for subject info ... \n");
Oldfolder = cd(Subject_folder);
% count the number of folders.
all_files = dir;
all_data = all_files(~[all_files(:).isdir]);
num_data = numel(all_data);

% Removing virtual folders
j = 0;
for i = 1:num_data
    if contains(all_data(i).name,"subject_")
        j = j + 1;
        subject_yaml_info = fullfile(Subject_folder,all_data.name);
    end
end
% Come Back at the starting folder
cd(Oldfolder);

% Compute local PIs for each experiment
Compute_all_LOCAL_PI(Tests_folders,Protocol, subject_yaml_info, result_folder);

% move into the Experiment data directory
Old_Folder = cd(output_folders);

fprintf("Searching for Local_PI folders ... \n");
% count the number of folders.
all_files = dir;
all_dir = all_files([all_files(:).isdir]);
num_dir = numel(all_dir);

% Removing virtual folders
j = 0;
PI_folders = {};
for i = 1:num_dir
    if all_dir(i).name ~= "."  &&  all_dir(i).name ~= ".." &&  all_dir(i).name ~= "Global_PIs"
        j = j + 1;
        PI_folders(j,1) = {fullfile(output_folders,all_dir(i).name,'Local_PI')};
    end
end
% Come Back at the starting folder
cd(Old_Folder);

%Collect the PI datas from each generated folders
fprintf("Collecting Local PI from experiments folders ... \n");
[Stability_margin_matrix, Second_PI] = Collect_Local_PI(PI_folders, Protocol);
% Second_PI is the matrix of the other PI.

fprintf("Computing Global PI ... \n");

if Protocol == 1
    Global_stability_margins(Stability_margin_matrix, Protocol, Global_PI_folder);
    Absorbed_energy = Second_PI;
    Global_absorbed_Energy_PI(Absorbed_energy, Global_PI_folder);
elseif Protocol == 2 || Protocol == 3
    Global_stability_margins(Stability_margin_matrix, Protocol, Global_PI_folder);
    Excited_impedance = Second_PI;
    Global_excited_impedance(Excited_impedance,Global_PI_folder, Protocol);
elseif Protocol == 4 || Protocol == 5
    Global_stability_margins(Stability_margin_matrix, Protocol, Global_PI_folder);
end

end

