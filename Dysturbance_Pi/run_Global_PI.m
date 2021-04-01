function run_Global_PI(data_folder, Protocol)
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
fprintf("data folder: %s\n", data_folder);
fprintf("Protocol: %f \n", Protocol)
%add the path for function folders
addpath("src");

Compute_Global_PI(data_folder, Protocol);
end