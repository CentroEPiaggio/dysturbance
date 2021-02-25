function [Exp_vector,Not_feasible] = Impulsive_experiments_set(Energy,Impulse,Pendulum_length)

delta = 4.13;
Dt = 0.01;
L = Pendulum_length;
E = Energy;
I = Impulse;
g = 9.81;
Length = [1.0, 0.5, 1.4];
L_min = Length(2);
L_max = Length(3);
M_max = 15;
%% Check for feasibility
check_1 = (3*I^2)/(2*0.67*delta*L);
check_2 = I^2/(2 * 0.67 * (M_max + delta * L_max));
check_3 = delta*L_min*g*L_min*(1-cosd(10));
% L is the length we prefer to use, not the only one available
check_L = 0;
for l = 1:size(Length,2)
    if L == Length(l)
        check_L = 1;
        Length(l) = [];
        break;
    end
end

if check_L == 0
    error("ERROR: chosen length is not available between the given ones.");
end

if E > check_1 || E < check_2|| E < check_3 || E <= 0 || I <= 0
    error("ERROR: chosen Energy and Impulse are not feasible for test in this structure");
end
%% Since they are feasible
Vector_Impulse = [0.5, 0.8, 1.1, 1.4, 1.7, 2.0]*I;
Vector_energy = [0.5, 0.8, 1.1, 1.4, 1.7, 2.0]*E;
k = 1;
t = 1;
Not_feasible = [];
for j = 1:size(Vector_energy,2)
    for i = 1:size(Vector_Impulse,2)
        
        check_1 = (3*Vector_Impulse(i)^2)/(2*0.67*delta*L);
        check_2 = Vector_Impulse(i)^2/(2 * 0.67 * (M_max + delta * L_max));
        check_3 = delta*L_min*g*L_min*(1-cosd(10));
        
        if Vector_energy(j) < check_1 && Vector_energy(j) > check_2 && Vector_energy(j) > check_3
            M = (L^2*Vector_Impulse(i)^2/(2*Vector_energy(j)) - delta*L^3/3)/L^2;
            theta = - acosd(1 - Vector_energy(j)/((M +delta*L/2)*g*L));
            chosen_length = L;
            if M > 15 || round(theta) < -90
                M = NaN;
                theta = NaN;
            end
        else
            for l2 = 1:size(Length,2)
                check_1 = (3*Vector_Impulse(i)^2)/(2*0.67*delta*Length(l2));
                check_2 = Vector_Impulse(i)^2/(2 * 0.67 * (M_max + delta * L_max));
                check_3 = delta*L_min*g*L_min*(1-cosd(10));
                if Vector_energy(j) < check_1 && Vector_energy(j) > check_2 && Vector_energy(j) > check_3
                    M = (Length(l2)^2*Vector_Impulse(i)^2/(2*Vector_energy(j)) - delta*Length(l2)^3/3)/Length(l2)^2;
                    theta = - acosd(1 - Vector_energy(j)/((M +delta*Length(l2)/2)*g*Length(l2)));
                    chosen_length = Length(l2);
                    if M > 15 || round(theta) < -90
                        M = NaN;
                        theta = NaN;
                    else
                        break;
                    end
                else
                    M = NaN;
                    theta = NaN;
                end
            end
        end
        if isnan(M) || isnan(theta)
            Not_feasible(t,:) = [Vector_energy(j),Vector_Impulse(i)];
            fprintf("Values of Energy = %f and Force = %f are not feasible for this benchmark with any Length. Not considered. \n",Vector_energy(j),Vector_Impulse(i));
            t = t+1;
        end
        
        Experiment_vec(k,:) = [round(2*M)/2, round(theta), Vector_energy(j),Vector_Impulse(i), chosen_length];
        k = k+1;
    end
end

Experiment_vector = sortrows(Experiment_vec,1,'MissingPlacement','last');
Experiment_vector = Experiment_vector(1:(end-t),:);

mass_check = NaN;
prev_angle = NaN;
Exp_vector = [];
a = 1;
for p = 1:size(Experiment_vector,1)
    if Experiment_vector(p,1) == mass_check && (Experiment_vector(p,2) < prev_angle + 1 && Experiment_vector(p,2) > prev_angle - 1)
    else
        Exp_vector(a,:) = Experiment_vector(p,:);
        a = a + 1;
        mass_check = Experiment_vector(p,1);
        prev_angle = Experiment_vector(p,2);
    end
    
end
I_check = linspace(0,(max(Vector_Impulse)+0.5),101);
E_check_1 = 3.*I_check.^2 ./ (2*delta*L_min*0.67);
E_check_2 = I_check.^2./(2 * 0.67 * (10 + delta * L_max));
E_check_3 = delta*L_min*g*L_min*(1-cosd(10)).*ones(size(I_check));

figure('defaultAxesFontSize',36);
plot(Exp_vector(:,4),Exp_vector(:,3),'rx','LineWidth',3);
hold on;
xlim([0,(max(Vector_Impulse)+0.5)]);
ylim([0,(max(Vector_energy)+0.5)]);
plot(I_check,E_check_1,'k','LineWidth',3);
plot(I_check,E_check_2,'k','LineWidth',3);
plot(I_check,E_check_3,'k','LineWidth',3);

xlabel('Impulse [Ns]');
ylabel('Initial Energy [J]');
title('Protocol 1 Stability Margin Plot');

end

