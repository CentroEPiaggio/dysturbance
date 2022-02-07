function run_local_pi(filename, yaml_file, subject_yaml, result_folder)
  addpath("src");
  fprintf("Starting local PI computation...\n");
  Compute_Local_PI(filename, yaml_file, subject_yaml, result_folder);
end