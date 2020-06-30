function [Stability_margin_matrix, Second_PI] = Collect_Local_PI(Tests_folders, Protocol)
%--------------------------------------------------------------------------
% Collect Local PI for the Dysturbance Tests
%
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@iit.it
%--------------------------------------------------------------------------

num_folder = numel(Tests_folders);
num = 0;
for i = 1:num_folder
    local_folder = strcat('.\', Test_folders(i).name);
    cd local_folder;
    Protocol_matrix = csvread('protocol_check.csv');
    Protocol_number = str2double(Protocol_matrix(2));
    if Protocol_number == Protocol
        num = num + 1;
        cd .\Local_PI
        fileinfo = dir;
        all_files = fileinfo(~[fileinfo(:).isdir]);
        if Protocol == 1
            for j = 1:numel(all_files)
                clear Data Data_2;
                if contains(all_files.name(j), "Stability_margin_")
                    Data = csvread(all_files.name(j));
                    Stability_margin_mat(num,:) = str2double(Data(2,:));
                elseif contains(all_files.name(j), "Absorbed_energy_")
                    Data_2 = csvread(all_files.name(j));
                    Sec_PI(num,:) = str2double(Data_2(2,:));
                elseif contains(all_files.name(j), "Fall_check_")
                    Check = csvread(all_files.name(j));
                    Check_Fall(num,:) = str2double(Check(2,:));
                end
            end
            Stability_margin_matrix = [Stability_margin_mat,Check_Fall];
            Second_PI = [Sec_PI, Check_Fall];
        elseif Protocol == 2
            for j = 1:numel(all_files)
                if contains(all_files.name(j), "Stability_margin_")
                    Data = csvread(all_files.name(j));
                    Stability_margin_mat(num,:) = str2double(Data(2,:));
                elseif contains(all_files.name(j), "Equivalent_impedance_")
                    Data_2 = csvread(all_files.name(j));
                    Sec_PI(num,:) = str2double(Data_2(2,:));
                elseif contains(all_files.name(j), "Fall_check_")
                    Check = csvread(all_files.name(j));
                    Check_Fall(num,:) = str2double(Check(2,:));
                end
            end
            Stability_margin_matrix = [Stability_margin_mat,Check_Fall];
            Second_PI = [Sec_PI, Check_Fall];
        elseif Protocol == 3
            for j = 1:numel(all_files)
                if contains(all_files.name(j), "Stability_margin_")
                    Data = csvread(all_files.name(j));
                    Stability_margin_mat(num,:) = str2double(Data(2,:));
                elseif contains(all_files.name(j), "Equivalent_impedance_")
                    Data_2 = csvread(all_files.name(j));
                    Sec_PI(num,:) = str2double(Data_2(2,:));
                elseif contains(all_files.name(j), "Fall_check_")
                    Check = csvread(all_files.name(j));
                    Check_Fall(num,:) = str2double(Check(2,:));
                end
            end
            Stability_margin_matrix = [Stability_margin_mat,Check_Fall];
            Second_PI = [Sec_PI, Check_Fall];
        elseif Protocol == 4
            for j = 1:numel(all_files)
                if contains(all_files.name(j), "Stability_margin_")
                    Data = csvread(all_files.name(j));
                    Stability_margin_mat(num,:) = str2double(Data(2,:));
                elseif contains(all_files.name(j), "Fall_check_")
                    Check = csvread(all_files.name(j));
                    Check_Fall(num,:) = str2double(Check(2,:));
                end
                
            end
            Sec_PI(num,:) = 0;
            Stability_margin_matrix = [Stability_margin_mat,Check_Fall];
            Second_PI = [Sec_PI, Check_Fall];
        end
        cd ..;
        cd ..;
    else
        error("Experiments data refers to the wrong protocol dataset");
    end
end


end

