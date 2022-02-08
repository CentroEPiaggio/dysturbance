function [Stability_Margin] = Stability_margin(datafile, Data_folder, Pendulum_data, Protocol_data, Norm_factor)

%--------------------------------------------------------------------------
% This function computes the Stability Margin of a test, depending on the Protocol of test.
% - If it is impulsive, it returns only the test margin (aka the maximum
%   exerted force). The impulsive stability margin for impulsive tests is
%   the maximum force exerted before going into the instability area.
% - If it is sinusoidal, it returns only the test margin (aka the maximum
%   exerted amplitude) and the frequency the test has been done. The stability margin is the maximum amplitude
%   before the robot falls.
% - If it is linear, the function returns the stability margin of the
%   robot, as the maximum exerted force before the system falls.
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@@phd.unipi.it
%--------------------------------------------------------------------------


%% Pendulum_data
Pendulum_length = Pendulum_data(1);
Pendulum_height = Pendulum_data(2);
Pendulum_mass = Pendulum_data(3);
frequency_acquisition = Pendulum_data(4);
dt = 1/frequency_acquisition;
delta = 4.13;
g = 9.81;
%% Protocol_data
Protocol_number = Protocol_data(1);
switch Protocol_number
    case 1
        Initial_position = Protocol_data(2);
        init_E = (Pendulum_mass + delta * Pendulum_length/2)*g*Pendulum_length*(1-cosd(Initial_position));
        exp_I = sqrt(2*0.67*init_E*(Pendulum_mass*Pendulum_length^2 + delta*Pendulum_length^3/3))/Pendulum_length;
        %init_E = Protocol_data(4);
        %exp_F = Protocol_data(5);
    case 2
        Displ_amplitude = Protocol_data(2);
        frequency_pendulum = Protocol_data(3);
        number_cycles = Protocol_data(4);
    case 3
        Force_amplitude = Protocol_data(2);
        frequency_pendulum = Protocol_data(3);
        number_cycles = Protocol_data(4);
    case 4
        ramp_slope = Protocol_data(2);
    case 5
        ramp_slope = Protocol_data(2);
end
%% data extraction
% in the datafile must not be specified also the data folder

if exist(datafile)
    Data_matrix = readtable(datafile);
else
    error('ERROR: datafile does not exist');
end

Time = Data_matrix.Time_s_;
Pendulum_position = Data_matrix.PendulumAngularPosition_deg_;
Force_sensor = Data_matrix.ContactForce_N_;
Torque_sensor = Data_matrix.OutputTorque_Nm_;

initial_pendulum_position = mean(Pendulum_position(1:100));
if Protocol_number == 1
    i_fmax= find(Force_sensor == max(Force_sensor),1);
    Force_start = mean(Force_sensor(i_fmax-500:i_fmax-250));
    force_max = Force_sensor(i_fmax);
    found_start = 0;
    found_stop = 0;
    thres= max(4,force_max/50);
    thres = min(thres,15);
    for steps = (i_fmax-1000):i_fmax
        if Force_sensor(steps) >= Force_start + thres && found_start == 0
            step_start = steps;
            found_start = 1;
        end
    end
%     for steps_ = i_fmax:(i_fmax+1000)
%         if Force_sensor(steps_) <= force_max/2 && found_stop == 0
%             step_stop = steps_ + (steps_-i_fmax);
%             found_stop = 1;
%         end
%     end
%     if found_stop == 0 || found_start == 0
%         fprintf("Error: %s file has a strange impulse behavior.... \n", datafile);
%     end
    step_stop = i_fmax + (i_fmax -step_start);
    Impulse_time = Time(step_stop)- Time(step_start);

    Impulse = 0;

    for i_imp = step_start:step_stop
        Impulse = Impulse + dt*Force_sensor(i_imp);
    end
    Medium_force = Impulse/Impulse_time;
    if abs(initial_pendulum_position - Initial_position) > 5
        fprintf("Initial Pendulum position is different than expected...\n Computing with new Initial position... \n");
    end


    %initial velocity
    Position_one_start = pi/180*mean(Pendulum_position((step_start-220):(step_start-200)));
    Position_two_start = pi/180*mean(Pendulum_position((step_start-120):(step_start-100)));

    velocity_start = (Position_two_start-Position_one_start)/0.0100;
%     velocity_start = pi/180*(Position_filtered(step_start-100)-Position_filtered(step_start-200))/0.01;

    %final velocity
    Position_one_stop = pi/180*mean(Pendulum_position((step_stop+100):(step_stop+120)));
    Position_two_stop = pi/180*mean(Pendulum_position((step_stop+200):(step_stop+220)));

    velocity_stop = (Position_two_stop-Position_one_stop)/0.0100;
%     velocity_stop = pi/180*(Position_filtered(step_stop+100)-Position_filtered(step_stop))/0.01;

    Impulse_vel = (Pendulum_mass *Pendulum_length^2 + 4.13*Pendulum_length^3/3)*(velocity_start-velocity_stop)/Pendulum_length;
end



%%
% Torque sensor can be used to compute the force for constant or sinusoidal
% patterns, since piezoelectric systems may have drifts
Force_indirect = Torque_sensor./Pendulum_length;

Norm_force = Norm_factor(1);
Norm_displacement = Norm_factor(3);

switch Protocol_number
    case 1
        %% Protocol number 1: Impulsive disturbances test
        header = "[Local Stability Margin [N],Normalized Local Stability Margin [N],Initial Pendulum Position [deg],Pendulum_mass [Kg], Pendulum_length [m], Expected Impulse [Ns], Expected Energy [J], Impulse Time [s], Impulse Medium Force[N], Impulse [Ns], Impulse thorugh velocity [Ns]]";
        Stability_Margin = [max(Force_sensor), max(Force_sensor)/Norm_force, initial_pendulum_position, Pendulum_mass, Pendulum_length,exp_I, init_E,Impulse_time,Medium_force, Impulse,Impulse_vel];
    case 3
        %% Protocol number 3: Sinusoidal force disturbances test
        data_number = size(Force_indirect,1);

        Minimi = mink(Force_indirect(floor(data_number/number_cycles):floor(data_number*2/number_cycles)),10);
        i_minmin = find(Minimi == min(Minimi),1);
        Minimi(i_minmin) = [];
        Minimum = mean(Minimi);

        Maximi = maxk(Force_indirect(floor(data_number/number_cycles):floor(data_number*2/number_cycles)),10);
        i_maxmax = find(Maximi == max(Maximi),1);
        Maximi(i_maxmax) = [];
        Maximum = mean(Maximi);

        header = "[Local Stability Margin [N], Normalized Local Stability Margin [N], Test_frequency [Hz], Expected Force Amplitude [N]]";
        Stability_Margin = [abs(Maximum - Minimum)/2,abs(Maximum - Minimum)/2/Norm_force , frequency_pendulum, Force_amplitude];

    case 2
        %% Protocol number 2: Sinusoidal displacements disturbances test
        data_number = size(Pendulum_position,1);
        Minimi = mink(Pendulum_position(floor(data_number/number_cycles):floor(data_number*2/number_cycles)),10);
        i_minmin = find(Minimi == min(Minimi),1);
        Minimi(i_minmin) = [];
        Minimum = mean(Minimi);

        Maximi = maxk(Pendulum_position(floor(data_number/number_cycles):floor(data_number*2/number_cycles)),10);
        i_maxmax = find(Maximi == max(Maximi),1);
        Maximi(i_maxmax) = [];
        Maximum = mean(Maximi);
        header = "[Local Stability Margin [m], Normalized Local Stability Margin [m], Test_frequency [Hz], Expected Displacement [degrees]]";
        Stability_Margin = [(Pendulum_length*sind(Maximum - Minimum))/2,(Pendulum_length*sind(Maximum - Minimum))/2/Norm_displacement, frequency_pendulum, Displ_amplitude];
    case 5
        %% Protocol number 5: Quasi-static force disturbances test
        header = "[Local Stability Margin [N], Normalized Local Stability Margin [N], empty]";
        Stability_Margin = [max(Force_indirect), max(Force_indirect)/Norm_force,0];
    case 4
         %% Protocol number 4: Quasi-static displacement disturbances test
         Displacement = Pendulum_length * sind(max(Pendulum_position)) - Pendulum_length * sind(min(initial_pendulum_position));
        header = "[Local Stability Margin [N], Normalized Local Stability Margin [N], empty]";
        Stability_Margin = [ Displacement, Displacement/Norm_displacement,0];
    otherwise
        error('ERROR: Wrong Protocol Number Inserted');
end


% data must be saved in a specified folder
Data_local_stability_folder = Data_folder;

% find name from datafile
FILE = cellstr(datafile);
index = cell2mat(strfind(FILE,"Preprocessed_data")) + 18;


% Stability_file_name = strcat("Stability_margin_",FILE{1}(index:end-4),".yaml");
Stability_file_name = strcat("Stability_margin.yaml");

type = find_type(Stability_Margin);

fileID = fopen(fullfile(Data_local_stability_folder,Stability_file_name),'w');
fprintf(fileID,'type: %s \n',type);
fprintf(fileID, 'label: %s \n',header);
fmt = ['value: [', repmat('%g, ', 1, numel(Stability_Margin)-1), '%g]\n'];
fprintf(fileID, fmt, Stability_Margin);
fclose(fileID);

end

