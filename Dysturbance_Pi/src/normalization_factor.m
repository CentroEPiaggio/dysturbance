function Norm_factor = normalization_factor(robotdatafile, Frontal_or_lateral)
%--------------------------------------------------------------------------
% This function get the robot data dimensions, in
% order to be used in the computation of normalization factor
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

%% Extract robot data from file

%robotdadtafiles contains the major measurements needed to compute the
%normalization factor for this specific robot.

% read data from yaml
datastructure = ReadYaml(robotdatafile);

Robot_height = datastructure.subject.height;
Robot_mass = datastructure.subject.mass;
Base_width = datastructure.subject.base_width;
Base_depth = datastructure.subject.base_depth;
Robot_CoM_height = datastructure.subject.com_height;

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

%% Robot_normalization factor Force
K_factor_Force_frontal = Robot_mass/m_human * Base_depth/d_frontal_human * h_CoM_human/Robot_CoM_height;
K_factor_Force_lateral = Robot_mass/m_human * Base_width/d_lateral_human * h_CoM_human/Robot_CoM_height;

Normalization_Factors_Force = [K_factor_Force_frontal, K_factor_Force_lateral];

%% Robot_normalization factor Energy
K_factor_Energy_frontal = Robot_mass/m_human * (sqrt(Base_depth^2 + Robot_CoM_height^2) - Robot_CoM_height)/(sqrt(d_frontal_human^2 + h_CoM_human^2) - h_CoM_human);
K_factor_Energy_lateral = Robot_mass/m_human * (sqrt(Base_width^2 + Robot_CoM_height^2) - Robot_CoM_height)/(sqrt(d_lateral_human^2 + h_CoM_human^2) - h_CoM_human);

Normalization_Factors_Energy = [K_factor_Energy_frontal, K_factor_Energy_lateral];

%% Robot_normalization factor Displacement

K_factor_displacement = Robot_CoM_height/h_CoM_human;

if Frontal_or_lateral == "Frontal"
	Norm_factor(1) = Normalization_Factors_Force(1);
	Norm_factor(2) = Normalization_Factors_Energy(1);
    Norm_factor(3) = K_factor_displacement;
elseif Frontal_or_lateral == "Lateral"
	Norm_factor(1) = Normalization_Factors_Force(2);
	Norm_factor(2) = Normalization_Factors_Energy(2);
    Norm_factor(3) = K_factor_displacement;
else
	error('Experiment can be only Frontal or Lateral');
end

end

