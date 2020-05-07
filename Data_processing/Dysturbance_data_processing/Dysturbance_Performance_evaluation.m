%--------------------------------------------------------------------------
% Dysturbance Data Post-Processing and Performance Indicators Evaluation
%
% Eurobench Consortium
% 
% Created By: Monteleone Simone
%--------------------------------------------------------------------------
% 
% https://github.com/aremazeilles/eurobench_documentation/blob/master/data_format.adoc#raw-data-files
% 
% This script processes the datas given from the benchmarking of the system
% and proceed to evaluate the KPI of the humanoid robot that is under
% testing.
%
% In the script, datas must be given, in accordance with eurobench
% consortium indications:
% - Raw data is not defined, as long as it is given with a single files per
% run. rosbag, .txt, .csv are taken into consideration.
% - URDF file for the description of the robot model: kinematics, dynamics
% and sensors.
% Considering pre-processed data files we have(N = subject number, R = run 
% number) (For further details, go to the link at the top of the comment 
% section):
% 
% - subject_N_jointAngles_R: .csv format containing the time-series of all
% measured joint angles, expressed in YXZ Cardan Angles [degrees]. 
% 
% - subject_N_jointTorques_R: .csv format containing the time-series of all
% measured joint torques [Nm].
% 
% - subject_N_jointTrajectories_R: .csv format, containing the time-series 
% of all the measured trajectories of the joints [mm]. 
% 
% - subject_N_grf_R: .csv format, contain the Ground Reaction Forces
% measured by force platforms. In order there are forces, centers of 
% pressure and moments [N, m, Nm]
% 
