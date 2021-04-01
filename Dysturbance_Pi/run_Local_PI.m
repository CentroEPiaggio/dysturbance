function run_Local_PI(filename,yaml_file, result_folder)
%--------------------------------------------------------------------------
% Local PI for the Dysturbance Test Bench
%
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------
% Print information on PI computation
fprintf("current directory: %s\n", pwd());
fprintf ("csv file: %s\n", filename);
fprintf ("structure file: %s\n", yaml_file);
fprintf ("result dir: %s\n", result_folder);
%add the path for function folders
addpath("src");

Compute_Local_PI(filename,yaml_file, result_folder);
end