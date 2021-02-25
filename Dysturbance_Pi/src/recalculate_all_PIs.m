function recalculate_all_PIs(data_folder,Protocol)
% This function cancels all the Local PI and GLobal PIs that have been
% computed for a specific subject and protocol

Protocol_folder = strcat("\protocol_",num2str(Protocol));
Global_PI_folder = strcat('tests\', data_folder,Protocol_folder,'\Global_PIs');
status = rmdir(Global_PI_folder,'s');
fprintf("Eliminating Global PIs Folder ... \n");

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

num_folder = size(Tests_folders,1);
fprintf("Eliminating Local PIs Folders ... \n");

for i = 1:num_folder
    Local_PI_folder = strcat(Tests_folders(i),"\Local_PI");
    if exist(Local_PI_folder, 'dir')
        status = rmdir(Local_PI_folder,'s');
    end
end

end

