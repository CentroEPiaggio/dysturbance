function check_isfall(isfall, filename, Data_local_folder)
%--------------------------------------------------------------------------
% This function check if during the tests, the robot is fallen or not
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------
if isfall == "true"
    check_fall = 1;
elseif isfall == "false"
    check_fall = 0;
else 
    error("boolean isfall is not true nor false. Please correct it in %s, folder %s \n", filename, Data_local_folder);
end

header = "check for fall";
Fall_matrix = [header; check_fall];
Fall_file_name = strcat("Fall_check_",filename);
writematrix(Fall_matrix,fullfile(Data_local_folder,Fall_file_name));

end

