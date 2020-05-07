function [Normalization_Factors] = normalization_factor(robotdatafile)
%--------------------------------------------------------------------------
% This function get the robot data dimensions, in
% order to be used in the computation of normalization factor
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@iit.it
%--------------------------------------------------------------------------

%% Extract robot data from file

%robotdadtafiles contains the major measurements needed to compute the
%normalization factor for this specific robot.

% read data from yaml
datastructure = ReadYaml(robotdatafile);

Robot_height = str2double(datastructure.height);
Robot_mass = str2double(datastructure.mass);
Base_width = str2double(datastructure.base_width);
Base_depth = str2double(datastructure.base_depth);
Robot_CoM_height = str2double(datastructure.center_of_mass_height);

%% Medium size human dimensions
%those data has been taken from ""

%bodyparts_weight (Kg)
m_head = 4.2;
m_neck = 1.1;
m_thorax = 24.9;
m_abdomen = 2.4;
m_pelvis = 11.8;
m_upperarm = 2.0;
m_forearm = 1.4;
m_hand = 0.5;
m_thigh = 9.8;
m_calf = 3.8;
m_foot = 1.0;

%bodyparts_height (cm)
h_head = 167.9;
h_neck = 154.5;
h_thorax = 130.8;
h_abdomen = 109.9;
h_pelvis = 98.3;
h_upperarm = 146.2 - 17.7;      % measure is given from Acromion
h_forearm = 146.2 - 43.5;       % measure is given from Acromion
h_hand = 146.2 - 67;            % measure is given from Acromion
h_thigh = 75.0;
h_calf = 33.0;
h_foot = 2.8;


d_frontal_human = 0.111;              % [m]
d_lateral_human = 0.333/2;            % [m]
m_human = m_head + m_neck + m_thorax + m_abdomen + m_pelvis + 2 * (m_upperarm + m_forearm + m_hand + m_thigh + m_calf + m_foot);
h_CoM_human = 0.01 *(m_head*h_head + m_neck*h_neck + m_thorax*h_thorax + m_abdomen*h_abdomen + m_pelvis*h_pelvis + 2*(m_upperarm*h_upperarm + m_forearm*h_forearm  + m_hand*h_hand + m_thigh*h_thigh + m_calf*h_calf + m_foot*h_foot))/m_human;

%% Robot_normalization factor
K_factor_frontal = Robot_mass/m_human * Base_depth/d_frontal_human * h_CoM_human/Robot_CoM_height;
K_factor_lateral = Robot_mass/m_human * Base_width/d_lateral_human * h_CoM_human/Robot_CoM_height;

Normalization_Factors = [K_factor_frontal, K_factor_lateral];

end

