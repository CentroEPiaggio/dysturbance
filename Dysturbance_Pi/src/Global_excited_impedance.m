function Global_excited_impedance(Equivalent_excited_Impedence_matrix, Global_PI_folder,Protocol)
%--------------------------------------------------------------------------
% This function computes the Global KPI for Excited_impedance for Protocol 2/3.
% Global KPI must be stored in a Matrix of this form
% KPI_value = [I_eq1, D_eq1, K_eq1, frequency_pendulum1;
%              I_eq2, D_eq2, K_eq2, frequency_pendulum2;
%                       ........
%              I_eqn, D_eqn, K_eqn, frequency_pendulumn];
% 
% Impedance is a intrinsic property of a system, so that it should not vary
% between experiments at the same frequency. However, different frequency
% may be affected the equivalent properties, so a general matrix showing
% mean values of impedance at different frequencies is built, in order to
% extrapolated a plot.
%
%
% Dysturbance - EuroBench Consortium
% Created By: Simone Monteleone
% mail: simone.monteleone@phd.unipi.it
%--------------------------------------------------------------------------

% sistemo i dati in matrice in modo che le righe siano ordinati per valore
% crescente. In questo modo, le frequenze uguali sono in sequenza
Data_Matrix = sortrows(Equivalent_excited_Impedence_matrix,4);

frequency_check =Data_Matrix(1,4);
K = 1;
j = 0;
Mean_value = [0, 0, 0, 0];

for i = 1:size(Data_Matrix,1)
    if (Data_Matrix(i,4) >= frequency_check - 0.01) && (Data_Matrix(i,4) <= frequency_check + 0.01)
        j = j + 1;
        Data_structure(K,1) = (Mean_value(1,1) * (j-1) + Data_Matrix(i,1))/j;
        Data_structure(K,2) = (Mean_value(1,2) * (j-1) + Data_Matrix(i,2))/j;
        Data_structure(K,3) = (Mean_value(1,3) * (j-1) + Data_Matrix(i,3))/j;
        Data_structure(K,4) = frequency_check;
        Mean_value = Data_structure(K,:);
    else
        K = K + 1;
        j = 1;
        Mean_Value = Data_Matrix(i,:);
        frequency_check = Data_Matrix(i,4);
        Data_structure(K,1) = Mean_value(1,1);
        Data_structure(K,2) = Mean_value(1,2);
        Data_structure(K,3) = Mean_value(1,3);
        Data_structure(K,4) = frequency_check;
    end
end

%         for i_mas = 1:size(Data_structure,2)
%             clear Matrix_Data;
%             Matrix_Data = sortrows(Data_structure{i_mas},5);
%             check = Matrix_Data(1,5);
%             if Protocol == 2
%                 check_value = 1.1;
%             else
%                 check_value = 0.3;
%             end
%             Kmas = 1;
%             jmas = 0;
%             for i_mas_2 = 1:size(Matrix_Data,1)
%                 if (Matrix_Data(i_mas_2,5) >= check - check_value) && (Matrix_Data(i_mas_2,3) <= check + check_value)
%                     jmas = jmas + 1;
%                     Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
%                 else
%                     Kmas = Kmas + 1;
%                     jmas = 1;
%                     Data_structure_2{i_mas,Kmas}(jmas,:) = Matrix_Data(i_mas_2,:);
%                     check = Matrix_Data(i_mas_2,5);
%                 end
%             end
%         end


figure('defaultAxesFontSize',36);
plot(Data_structure(:,4),Data_structure(:,1)/0.8^2,'b','LineWidth',3);
hold on;
plot(Data_structure(:,4),Data_structure(:,2)/0.8^2,'r','LineWidth',3);
plot(Data_structure(:,4),Data_structure(:,3)/0.8^2,'g','LineWidth',3);
hold off;
grid;
xlabel('Frequency [Hz]');
ylabel('Impedance Values');
legend('Equivalent Inertia [Kgm^2]','Equivalent Damping [Nms/rad]','Equivalent Elasticity [Nm/rad]');
if Protocol == 2
    title('Protocol 2 Equivalent Impedance');
else 
    title('Protocol 3 Equivalent Impedance');
end
I = Data_structure(:,1);
B = Data_structure(:,2);
K = Data_structure(:,3);
eps = B./(2*sqrt(K.*I));
omega_n = sqrt(K./I);

figure('defaultAxesFontSize',36);
plot(Data_structure(:,4),omega_n,'b','LineWidth',3);
hold on;
plot(Data_structure(:,4),eps,'r','LineWidth',3);
hold off;
grid;
xlabel('Frequency [Hz]');
ylabel('Impedance Values');
legend('$\omega_n$','$\epsilon$', 'interpreter','latex');
if Protocol == 2
    title('Protocol 2 Equivalent Impedance');
else 
    title('Protocol 3 Equivalent Impedance');
end

header = "[Equivalent Inertia [Kg], Equivalent Damping coefficient [Ns/m], Equivalent Elasticity [N/m], frequency [Hz]]";
% Equivalence_matrix = [header;Data_structure];


if ~isempty(Data_structure)
    type = find_type(Data_structure);
    
    fileID = fopen(strcat(Global_PI_folder,'\Global_Equivalent_Impedance.yaml'),'w');
    fprintf(fileID,'type: %s \n',type);
    fprintf(fileID, 'label: %s \n',header);
    fmt = ['value: [[', repmat('%g, ', 1, numel(Data_structure(1,:))-1), '%g]'];
    fprintf(fileID, fmt, Data_structure(1,:));
    for pi_i = 2:size(Data_structure,1)
        fmd = [',[', repmat('%g, ', 1, numel(Data_structure(pi_i,:))-1) '%g]'];
        fprintf(fileID, fmd, Data_structure(pi_i,:));
    end
    fprintf(fileID, ']\n');
    fclose(fileID);
end


% writematrix(Equivalence_matrix,strcat(Global_PI_folder,'\Global_Equivalent_Impedance.csv'));
end

