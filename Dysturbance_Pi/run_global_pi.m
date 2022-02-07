function run_global_pi(data_folder, protocol, result_folder)
  addpath("src");
  fprintf("Starting global PI computation...\n");
  Compute_Global_PI(data_folder, protocol, result_folder);
end