clc
clear all

g = 9.80665;         % gravity constant
m_r = 0.2948;        % mass of the rod
m_w = 0.0695;        % mass of the inertia wheel
R = 0.05;            % radius of the inertia wheel
r = 0.02;            % cross section radius of the rod
l = 0.13;            % corresponding lengths
l_AD = l;
l_AC = l;            
l_AB = l/2;
I_w_C = 0.5*m_w*R^2; % corresponding inertias
I_w_A = I_w_C + m_w*l_AC^2;
I_r_B = (1/12)*m_r*(3*r^2+l_AD^2);
I_r_A = I_r_B + m_r*l_AB^2;  


% some coefficients for the linearized A matrix
a21 = g * (m_r * l_AB + m_w * l_AC) / (I_w_A + I_r_A);

% A matrix based on the linearization
A = [0, 1, 0;
     a21, 0, 0;
     0, 0, 0];

% coefficients for the B matrix
b2 = -1 / (I_w_A + I_r_A);
b3 = 1 / I_w_C;

% the B matrix
B = [0;
     b2;
     b3];

print('A =', A)


% Checking controllability
controllability_matrix = ctrb(A, B);
rank_of_controllability = rank(controllability_matrix);

if rank_of_controllability < size(A)
    disp('The system is not controllable.');
else
    disp('The system is controllable.');
    
    % Desired pole locations (adjusted to be less aggressive)
    desired_poles = [1, 2, 1];

    % Compute the gain matrix K using pole placement
    K = place(A, B, desired_poles);

    % Display the gain matrix K
    disp('The gain matrix K is:');
    disp(K);
end
