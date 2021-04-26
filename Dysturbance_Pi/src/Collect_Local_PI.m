function [Stability_margin_matrix, Second_PI] = Collect_Local_PI(PI_folders, Protocol)
%--------------------------------------------------------------------------
% Collect Local PI for the Dysturbance Tests
%
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

num_folder = size(PI_folders,1);
num1 = 0;
num2 = 0;
num3 = 0;
for i = 1:num_folder
    local_folder = PI_folders(i);
    
    FILE_check = cellstr(local_folder);
    index_check = cell2mat(strfind(FILE_check,"\Local_"));
    check_folder = FILE_check{1}(1:index_check);

    Protocol_matrix = readtable(strcat(check_folder,"Preprocessed_data\","protocol_check.csv"));
    OLD_FOLDER = cd(FILE_check{1}(1:index_check(end)));
    Protocol_number = Protocol_matrix.Protocol_number;
    if Protocol_number == Protocol
        cd("Local_PI");
        fileinfo = dir;
        all_files = fileinfo(~[fileinfo(:).isdir]);
        switch Protocol
            case 1
                for j = 1:numel(all_files)
                    clear Data Data_2;
                    if contains(all_files(j).name, "Stability_margin_")
                        num1 = num1 + 1;
                        % read from yaml
                        Data = ReadYaml(all_files(j).name);
                        Stability_margin_mat(num1,:) = Data.value;
                        %Data = readtable(all_files(j).name);
                        %Stability_margin_mat(num1,:) = table2array(Data);
                    elseif contains(all_files(j).name, "Absorbed_energy_")
                        num2 = num2 + 1;
                        % read from yaml
                        Data_2 = ReadYaml(all_files(j).name);
                        Sec_PI(num2,:) = Data_2.value;
%                         Data_2 = readtable(all_files(j).name);
%                         Sec_PI(num2,:) = table2array(Data_2);
                    elseif contains(all_files(j).name, "Fall_check_")
                        num3 = num3 + 1;
                        Check = ReadYaml(all_files(j).name);% readtable(all_files(j).name);
                        Check_Fall(num3,:) = Check.value;%checkForFall;
                    end
                end
                Stability_margin_matrix = [Stability_margin_mat,Check_Fall];
                Second_PI = [Sec_PI, Check_Fall];
            case 2
                for j = 1:numel(all_files)
                    if contains(all_files(j).name, "Stability_margin_")
                        num1 = num1 + 1;
                        % read from yaml
                        Data = ReadYaml(all_files(j).name);
                        Stability_margin_mat(num1,:) = Data.value;
%                         Data = readtable(all_files(j).name);
%                         Stability_margin_mat(num1,:) = table2array(Data);
                    elseif contains(all_files(j).name, "Equivalent_impedance_")
                        num2 = num2 + 1;
                        Data_2 = ReadYaml(all_files(j).name);
                        Sec_PI(num2,:) = Data_2.value;
%                         Data_2 = readtable(all_files(j).name);
%                         Sec_PI(num2,:) = table2array(Data_2);
                    elseif contains(all_files(j).name, "Fall_check_")
                        num3 = num3 + 1;
                        Check = ReadYaml(all_files(j).name);% readtable(all_files(j).name);
                        Check_Fall(num3,:) = Check.value;
                    end
                end
                Stability_margin_matrix = [Stability_margin_mat,Check_Fall];
                Second_PI = [Sec_PI, Check_Fall];
            case 3
                for j = 1:numel(all_files)
                    if contains(all_files(j).name, "Stability_margin_")
                        num1 = num1 + 1;
                        % read from yaml
                        Data = ReadYaml(all_files(j).name);
                        Stability_margin_mat(num1,:) = Data.value;
%                         Data = readtable(all_files(j).name);
%                         Stability_margin_mat(num1,:) = table2array(Data);
                    elseif contains(all_files(j).name, "Equivalent_impedance_")
                        num2 = num2 + 1;
                        Data_2 = ReadYaml(all_files(j).name);
                        Sec_PI(num2,:) = Data_2.value;
%                         Data_2 = readtable(all_files(j).name);
%                         Sec_PI(num2,:) = table2array(Data_2);
                    elseif contains(all_files(j).name, "Fall_check_")
                        num3 = num3 + 1;
                        Check = ReadYaml(all_files(j).name);% readtable(all_files(j).name);
                        Check_Fall(num3,:) = Check.value;
                    end
                end
                Stability_margin_matrix = [Stability_margin_mat,Check_Fall];
                Second_PI = [Sec_PI, Check_Fall];
            case 4
                for j = 1:numel(all_files)
                    if contains(all_files(j).name, "Stability_margin_")
                        num1 = num1 + 1;
                        % read from yaml
                        Data = ReadYaml(all_files(j).name);
                        Stability_margin_mat(num1,:) = Data.value;
%                         Data = readtable(all_files(j).name);
%                         Stability_margin_mat(num1,:) = table2array(Data);
                    elseif contains(all_files(j).name, "Fall_check_")
                        num3 = num3 + 1;
                        Check = ReadYaml(all_files(j).name);% readtable(all_files(j).name);
                        Check_Fall(num3,:) = Check.value;
                    end
                    
                end
                Sec_PI(num1,:) = 0;
                Stability_margin_matrix = [Stability_margin_mat,Check_Fall];
                Second_PI = [Sec_PI, Check_Fall];
            case 5
                for j = 1:numel(all_files)
                    if contains(all_files(j).name, "Stability_margin_")
                        num1 = num1 + 1;
                        % read from yaml
                        Data = ReadYaml(all_files(j).name);
                        Stability_margin_mat(num1,:) = Data.value;
%                         Data = readtable(all_files(j).name);
%                         Stability_margin_mat(num1,:) = table2array(Data);
                    elseif contains(all_files(j).name, "Fall_check_")
                        num3 = num3 + 1;
                        Check = ReadYaml(all_files(j).name);% readtable(all_files(j).name);
                        Check_Fall(num3,:) = Check.value;
                    end
                    
                end
                Sec_PI(num1,:) = 0;
                Stability_margin_matrix = [Stability_margin_mat,Check_Fall];
                Second_PI = [Sec_PI, Check_Fall];
        end
    end
    cd(OLD_FOLDER);
end
end
