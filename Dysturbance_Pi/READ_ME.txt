The equivalent matlab function for these runs are:
- run_local_pi --> Compute_Local_PI(filename,yaml_file, result_folder)
- run_global_pi --> Compute_Global_PI(data_folder, Protocol)

%-----------------------------------------------------------------------
% Compute the Local PI
% Compute_Local_PI(filename,yaml_file, result_folder)
%-----------------------------------------------------------------------

1. Open matlab, and set the working directory to Dysturbance_pi;

2. add the path src to find functions

addpath("src");

3. Argument of the function Compute_Local_PI(filename,yaml_file, result_folder)
 -  filename is the name of the csv in which are stored the raw datas.
Ex:
filename = "subject_2_cond_1000000000_run_0_platformData.csv";

 -  yaml_file is the name of the yaml in which it is stored the information of the experiment(testbed, subject, etc);
Ex:
yaml_file = "subject_2_cond_1000000000_testbed.yaml";

 -  result_folder is the folder in which we store all the raw datas and information from experiments. Only subject and protocol folder must be specified;
Ex:
result_folder = "subject_2\protocol_1";

4. run the function for local pi:
Ex:
Compute_Local_PI(filename,yaml_file, result_folder);

These three entry are sufficient for the code to compute the local Pi of the single experiment.

%-----------------------------------------------------------------------
% Compute the Global PI
% Compute_Global_PI(data_folder, Protocol)
%-----------------------------------------------------------------------

1. Open matlab, and set the working directory to Dysturbance_pi;

2. add the path src to find functions

addpath("src");

3. Argument of the function Compute_Global_PI(data_folder, Protocol)
 -  data_folder is the folder in which it is store all the data of the tested subject
Ex:
data_folder = "subject_2";

 -  Protocol is the variable (double) that identifies the protocol executed.
Ex:
Protocol = 1;

Protocols numbers:
1. Impulsive Disturbance
2. Sinusiodal Displacement Disturbance
3. Sinusoidal Force Disturbance
4. Quasi_static Displacement Disturbance
5. Quasi-static Force Disturbance

%-----------------------------------------------------------------------
NOTE:
For an easier management of the large amount of different csv and yaml data, during the acquisition we divide and store the data in folders with the following structure:

tests -->
	subject_i --> 
		Protocol_j --> 
			subject_i_cond_jxxxxxxxxx --> 
					raw_data_input --> 
							yaml_file (subject_i_cond_jxxxxxxxxx_testbed.yaml);
							csv file, one per each n run (subject_i_cond_jxxxxxxxxx_run_n_platformData.csv);


During the Computations, the code will create additionals folders containing:
- Preprocessed datas: 
	subject_i_cond_jxxxxxxxxx -->
			 Preprocessed_data-->
				csv file, one per each n run (subject_i_cond_jxxxxxxxxx_run_n_pp_platformData.csv)
- Local PIs:
	subject_i_cond_jxxxxxxxxx -->
			 Local_PI -->
				csv files, one per each PI
- Global PIs: 
tests --> 
	subject_i --> 
		Protocol_j --> 
			Global_PI