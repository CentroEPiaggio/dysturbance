function Compute_all_LOCAL_PI(Tests_folders, Protocol, subject_yaml_info, result_folder)
%Compute the Local PIs in each folders the program recognize there are
%experiments runs.
num_folder = size(Tests_folders,1);

for i = 1:num_folder
    % Searching for raw datas inside each tests_folder
    folder = Tests_folders(i);
    raw_data_folder = fullfile(folder{1},'raw_data_input');
    Old_Folder = cd(raw_data_folder);
    clear all_data all_files num_files yaml_file filename;
    %collect the files in which are stored the raw data
    all_data = dir;
    all_files = all_data(~[all_data(:).isdir]);
    num_files = numel(all_files);

    index_folder = strfind(folder{1},"subject_");
    experiment_condition_check = folder{1}(index_folder(2):end);
    % Divide files between yaml and csv
    t = 1;
    for i2 = 1:num_files
        if contains(all_files(i2).name, ".csv") && contains(all_files(i2).name, experiment_condition_check)
            filename{t} = all_files(i2).name;
            t = t + 1;
        elseif contains(all_files(i2).name, ".yaml") && contains(all_files(i2).name, experiment_condition_check)
            yaml_file = all_files(i2).name;
        else
            fprintf("unknown file format found during raw data analysis. The file will not be considered.\n");
        end
    end
    cd(Old_Folder);
    
    num_csv = size(filename,2);
    % FIXME: unused stuff
    % indices = strfind(folder,filesep);
    % folder_cell = cellstr(folder);
    % result_folder = folder_cell{1}((indices(1)+1):(indices(3)-1));
    
    % Computations of all local PI must be done
    for i3 = 1:num_csv
        %searching if local PI is already have been calculated for that
        %experiment
        FILE = cellstr(folder);
        index = cell2mat(strfind(FILE,filesep));
        name_of_output_folder = fullfile(result_folder,FILE{1}(index(4):end));
        folder_local_pi = fullfile(name_of_output_folder,'Local_PI');
        if ~exist(folder_local_pi, 'dir')
            mkdir(folder_local_pi)
        end
        cd(folder_local_pi);
        computed_csv = filename{i3};
        local_data = dir;
        local_data_files = local_data(~[local_data(:).isdir]);
        cd(Old_Folder);
        File_csv = cellstr(computed_csv);
        index = cell2mat(strfind(File_csv,"platformData")) - 1;
        csv_comparison = File_csv{1}(1:index);
        
        check_for_local_PI = 0;
        for i4 = 1:numel(local_data_files)
            if contains(local_data_files(i4).name,csv_comparison)
                % in the local_PI folder, exists more than one file that
                % contains the name of a same experiment. The number of
                % those files must be equal to the number of PI of the
                % protocol
                check_for_local_PI = check_for_local_PI + 1;
            else
                check_for_local_PI = check_for_local_PI;
            end
        end

        NO_need_for_computation = 0;
        switch Protocol
            case 1
                if check_for_local_PI == 3
                	NO_need_for_computation = 1;
                else
                    NO_need_for_computation = 0;
                end
            case 2
                if check_for_local_PI == 3
                	NO_need_for_computation = 1;
                else
                    NO_need_for_computation = 0;
                end
            case 3
                if check_for_local_PI == 3
                	NO_need_for_computation = 1;
                else
                    NO_need_for_computation = 0;
                end
            case 4
                if check_for_local_PI == 2
                	NO_need_for_computation = 1;
                else
                    NO_need_for_computation = 0;
                end
            case 5
                if check_for_local_PI == 2
                	NO_need_for_computation = 1;
                else
                    NO_need_for_computation = 0;
                end
        end
        % computation of the missing PI
        if NO_need_for_computation == 0
            FILE_res = cellstr(folder_local_pi);
            index_res = cell2mat(strfind(FILE_res,filesep)) - 1;
            %folder_of_results = folder_local_pi{1}(1:index_res(4));
            filename_comp = fullfile(raw_data_folder, computed_csv);
            yaml_filepath = fullfile(raw_data_folder, yaml_file);

            subject_yaml = subject_yaml_info;
            Compute_Local_PI(filename_comp,yaml_filepath, subject_yaml, result_folder);%computed_csv, yaml_file, result_folder);
        end
    end
end

end

