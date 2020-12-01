function Global_excited_impedance(Equivalent_excited_Impedence_matrix, Global_PI_folder)
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
    if (Data_Matrix(i,4) >= frequency_check - 0.2) && (Data_Matrix(i,4) <= frequency_check + 0.2)
        j = j + 1;
        Data_structure(K,1) = (Mean_value(1,1) * (j-1) + Data_Matrix(i,1))/j;
        Data_structure(K,2) = (Mean_value(1,2) * (j-1) + Data_Matrix(i,2))/j;
        Data_structure(K,3) = (Mean_value(1,3) * (j-1) + Data_Matrix(i,3))/j;
        Data_structure(K,4) = (Mean_value(1,4) * (j-1) + Data_Matrix(i,4))/j;
        Mean_value = Data_structure(K,:);
    else
        K = K + 1;
        j = 1;
        Mean_Value = Data_Matrix(i,:);
        frequency_check = Data_matrix(i,4);
    end
end

header = ["Equivalent Inertia [Kg]","Equivalent Damping coefficient [Ns/m]","Equivalent Elasticity [N/m]","frequency [Hz]"];
Equivalence_matrix = [header;Data_structure];
writematrix(Equivalence_matrix,strcat(Global_PI_folder,'\Global_Equivalent_Impedance.csv'));
end

