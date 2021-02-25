function Compute_Global_PI(data_folder, Protocol)
%--------------------------------------------------------------------------
% Computation of the Global PI for Dysturbance Bench test
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

% creation of the folder to store the Global PI data
Protocol_folder = strcat("\protocol_",num2str(Protocol));
Global_PI_folder = strcat('tests\', data_folder,Protocol_folder,'\Global_PIs');
mkdir(Global_PI_folder);

% Now we must retrieve the local PI data from each experiment folder
Experiment_folder = strcat("tests\", data_folder,Protocol_folder);
% in Test_Data there are n folders related to n experiments. Each one of them contains a raw data folder and a local PI folder

% move into the Local PI data directory
Old_Folder = cd(Experiment_folder);

fprintf("Searching for experiments folders ... \n");
% count the number of folders.
all_files = dir;
all_dir = all_files([all_files(:).isdir]);
num_dir = numel(all_dir);

% Removing virtual folders
j = 0;
for i = 1:num_dir
    if all_dir(i).name ~= "."  &&  all_dir(i).name ~= ".." &&  all_dir(i).name ~= "Global_PIs"
        j = j + 1;
        Tests_folders(j,1) = strcat(Experiment_folder,"\",all_dir(i).name);
    end
end
% Come Back at the starting folder
cd(Old_Folder);

% Compute local PIs for each experiment
Compute_all_LOCAL_PI(Tests_folders,Protocol);

%Collect the PI datas from each generated folders
fprintf("Collecting Local PI from experiments folders ... \n");
[Stability_margin_matrix, Second_PI] = Collect_Local_PI(Tests_folders, Protocol);
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

