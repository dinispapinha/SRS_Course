%Author : Roxan PONS

clc;
clear;
close all;

%% Forward Kinematcs (Symbolic)
%Rule : Implement forward and inverse kinematics algorithms to determine
%the position and orientation of the manupulator's end-effector in space

%Symbolic parameters
syms alpha0 alpha1 alpha2 a0 a1 a2 d1 d2 d3 theta1 theta2 theta3 real
syms nb_of_joints L_X_EE L_Y_EE real

%Modified Denavit-Hartenberg table
M_D_H = [
    alpha0 a0 d1 theta1
    alpha1 a1 d2 theta2
    alpha2 a2 d3 theta3
    ]

%Computing of transformation matrices
T_0_1 = T_matrix(M_D_H(1,:))
T_1_2 = T_matrix(M_D_H(2,:))
T_2_3 = T_matrix(M_D_H(3,:))

T_0_2 = T_0_1 * T_1_2
T_0_3 = T_0_2 * T_2_3
T_1_3 = T_1_2 * T_2_3

Pos_EE = T_0_3(1:3,4)+ [L_X_EE; L_Y_EE; 0] %I admit that we want the tool position and not the wrist joint position*

T_3_T = [ 
    1, 0, 0, L_X_EE; 
    0, 1, 0, L_Y_EE; 
    0, 0, 1, 0; 
    0, 0, 0, 1;
    ]

%% Forward Kinematics numeric computation

% Robot's parameters definition
alpha_values = [0, 0, 0];
a_values = [0, 1, 1]; 
d_values = [0, 0, 0];
theta_values = deg2rad([60, -120, 60]); 


% Numerical evaluation of T_0_3 and End effector position
T_0_3_numeric = double(subs(T_0_3, [alpha0, alpha1, alpha2, a0, a1, a2, d1, d2, d3, theta1, theta2, theta3], [alpha_values, a_values, d_values, theta_values]))
Pos_EE_numeric = double(subs(Pos_EE, [alpha0, alpha1, alpha2, a0, a1, a2, d1, d2, d3, theta1, theta2, theta3, L_X_EE, L_Y_EE], [alpha_values, a_values, d_values, theta_values, 1, 0]))

disp('End-Effector Position (Numerical):');
disp(Pos_EE_numeric);

%% Inverse Kinematics (Symbolic)
%Symbolic parameters

syms c2 s2 k1 k2 theta1_inv theta2_inv theta3_inv real
syms x y phi real

T_W_B = [
    cos(phi) -sin(phi) 0 x 
    sin(phi) cos(phi) 0 y
    0 0 1 0
    0 0 0 1
]

c2 = (x^2 + y^2 - a1^2 -a2^2) / (2 * a1 * a2)
s2 = [sqrt(1 - c2^2), -sqrt(1 - c2^2)]
theta2_inv = atan2(s2, c2)
k1 = a1 + a2 * c2
k2 = a2 * s2
theta1_inv = atan2(y, x) - atan2(k2, k1)
theta3_inv = atan2(sin(phi), cos(phi)) - theta1_inv - theta2_inv

%% Inverse Kinematics numeric computation

%Position parameters of the robot

Num_pos = [1, 0, 0]

c2_num = double(subs(c2,[x, y, phi, a1, a2], [Num_pos, a_values(2:3)]))

if abs(c2_num) <= 1
    s2_num = double(subs(s2, [x, y, phi, a1, a2], [Num_pos, a_values(2:3)]))
    theta2_inv_num = double(subs(theta2_inv, [x, y, phi, a1, a2], [Num_pos, a_values(2:3)]))
    k1_num = double(subs(k1, [x, y, phi, a1, a2], [Num_pos, a_values(2:3)]))
    k2_num = double(subs(k2, [x, y, phi, a1, a2], [Num_pos, a_values(2:3)]))
    theta1_inv_num = double(subs(theta1_inv, [x, y, phi, a1, a2], [Num_pos, a_values(2:3)]))
    theta3_inv_num = double(subs(theta3_inv, [x, y, phi, a1, a2], [Num_pos, a_values(2:3)]))
    for i = 1:length(theta3_inv_num)
        fprintf('Position number {%d} possible :\n', i);
        disp(rad2deg([theta1_inv_num(i), theta2_inv_num(i), theta3_inv_num(i)]));
    end
else
    disp('ERROR: Robot can not reach the coordinates.');
end

%% Jacobian matrix
% Rule: Calculate the manipulator's Jacobian matrix to relate joint 
% velocities to the end-effector's linear and angular velocities
% syms v_x_EE v_y_EE omega_EE real
syms omega0 omega1 omega2 omega3 real
syms omega_1_0 omega_2_1 omega_3_2 omega_T_3 real
syms theta_dot1 theta_dot2 theta_dot3 theta_dot4 real
syms v0 v1 v2 v3 vT real
syms v_1_0 v_2_1 v_3_2 v_T_3 real
% vel_EE = [v_x_EE; v_y_EE; omega_EE];

J = jacobian(Pos_EE, [theta1, theta2, theta3]);
disp('Jacobian Matrix:');
disp(J);

% if size(J,1) == size(J,2)
%     if abs(det(J)) > 0.01
%         vel_joints = inv(J) * vel_EE;
%     else
%         disp('ERROR: Singularity in Jacobian Matrix');
%     end
% else
%     if size(J,1) < size(J,2)
%         disp('Over-actuated Manipulator (More columns than rows)');
%     elseif size(J,1) > size(J,2)
%         disp('Under-actuated Manipulator (More rows than columns)');
%     end
% end

% v_x_num = 0.1;
% v_y_num = 0.2;
% omega_num = 0.05;
% 
% if size(J,1) == size(J,2) && abs(det(J)) > 0.01
%     vel_joints_num = double(subs(vel_joints, [v_x_EE, v_y_EE, omega_EE, theta1, theta2, theta3], [v_x_num, v_y_num, omega_num, theta_values]));
%     disp('Joint velocities (Numeric):');
%     disp(vel_joints_num);
% else
%     disp('ERROR : Unable to calculate joint velocities due to singular or non-square Jacobian Matrix.');
% end

%Rotation matrices 
R_0_1 = T_0_1(1:3, 1:3)
R_1_2 = T_1_2(1:3, 1:3)
R_2_3 = T_2_3(1:3, 1:3)
R_3_T = T_3_T(1:3, 1:3)

R_0_2 = R_0_1 * R_1_2
R_0_3 = R_0_2 * R_2_3
R_1_3 = R_1_2 * R_2_3

R_1_0 = transpose(R_0_1) 
R_2_1 = transpose(R_1_2)
R_3_1 = transpose(R_1_3)
R_3_2 = transpose(R_2_3)
R_T_3 = transpose(R_3_T)
R_3_0 = transpose(R_0_3)

%Angular velocities
omega0 = sym('omega0', [3, 1]);
omega_1_0 = omega0 + R_0_1 * [0; 0; theta_dot1]
omega_2_1 = omega1 + R_1_2 * [0; 0; theta_dot2]
omega_3_2 = omega2 + R_2_3 * [0; 0; theta_dot3]
omega_2_0 = simplify(R_0_1 * omega_2_1)
omega_3_0 = simplify(R_0_2 * omega_3_2)

omega1 = R_0_1 * omega0 + [0; 0; theta_dot1]
omega2 = R_1_2 * omega1 + [0; 0; theta_dot2]
omega3 = R_2_3 * omega2 + [0; 0; theta_dot3]

%Linear velocities
v0 = [0; 0; 0];

v_1_0 = v0 + cross(omega0, T_0_1(1:3, 4))
v_2_1 = v1 + cross(omega1, T_1_2(1:3, 4))
v_3_2 = v2 + cross(omega2, T_2_3(1:3, 4))
v_T_3 = v3 + cross(omega3, [L_X_EE; L_Y_EE; 0])

v1 = R_0_1 * v_1_0
v2 = R_1_2 * v_2_1
v3 = R_2_3 * v_3_2
vT = R_3_T * v_T_3

%% Dynamics (Symbolic)
%Rule : Derive the dynamical equations governing the motion of the
%manipulator

%Iterative Newton-Euler method

%Outward iterations to compute velocities and accelerations

%Angular accelerations
syms omega_dot0 omega_dot1 omega_dot2 omega_dot3 omega_dotT real
syms theta_dot_dot1 theta_dot_dot2 theta_dot_dot3 theta_dot_dotT real
omega_dot0 = sym('omega_dot0', [3, 1])
omega_dot1 = R_0_1 * omega_dot0 + R_0_1 * cross(omega0, [0; 0; theta_dot1]) + [0; 0; theta_dot_dot1]
omega_dot2 = R_1_2 * omega_dot1 + R_1_2 * cross(omega1, [0; 0; theta_dot2]) + [0; 0; theta_dot_dot2]
omega_dot3 = R_2_3 * omega_dot2 + R_2_3 * cross(omega2, [0; 0; theta_dot3]) + [0; 0; theta_dot_dot3]
omega_dotT = omega_dot3 + cross(omega3, [0; 0; theta_dot_dotT]) + [0; 0; theta_dot_dotT]

%Linear accelerations
syms v_dot0 v_dot1 v_dot2 v_dot3 v_dotT g real

v_dot0 = [0; 0; -g]
v_dot1 = R_0_1 * (cross(omega_dot0, T_0_1(1:3, 4)) + cross(omega0, cross(omega0, T_0_1(1:3, 4))) + v_dot0)
v_dot2 = R_1_2 * (cross(omega_dot1, T_1_2(1:3, 4)) + cross(omega1, cross(omega1, T_1_2(1:3, 4))) + v_dot1)
v_dot3 = R_2_3 * (cross(omega_dot2, T_2_3(1:3, 4)) + cross(omega2, cross(omega2, T_2_3(1:3, 4))) + v_dot2)
v_dotT = R_3_T * (cross(omega_dot3, T_3_T(1:3, 4)) + cross(omega3, cross(omega3, T_3_T(1:3, 4))) + v_dot3)

%Linear accelerations of link's CoM

syms v_dot_c0 v_dot_c1 v_dot_c2 v_dot_c3 v_dot_cT g real

v_dot_c0 = cross(omega_dot0,[0; 0; 0]) + cross(omega0, cross(omega0, [0; 0; 0])) + v_dot0
v_dot_c1 = cross(omega_dot1,[a1; 0; 0]) + cross(omega1, cross(omega1, [a1; 0; 0])) + v_dot1
v_dot_c2 = cross(omega_dot2,[a2; 0; 0]) + cross(omega2, cross(omega2, [a2; 0; 0])) + v_dot2
v_dot_c3 = cross(omega_dot3,[L_X_EE; 0; 0]) + cross(omega3, cross(omega3, [L_X_EE; 0; 0])) + v_dot3
%v_dot_cT ?

%Inertial forces and torques on each link

syms F0 F1 F2 F3 FT N0 N1 N2 N3 NT I0 I1 I2 I3 m0 m1 m2 m3 real

%Inertial forces and torques acting at each link CoM
F0 = m0 * v_dot_c0
F1 = m1 * v_dot_c1
F2 = m2 * v_dot_c2
F3 = m3 * v_dot_c3
FT = 0

N0 = I0 * omega_dot0 + cross(omega0, I0 * omega0)
N1 = I1 * omega_dot1 + cross(omega1, I1 * omega1)
N2 = I2 * omega_dot2 + cross(omega2, I2 * omega2)
N3 = I3 * omega_dot3 + cross(omega3, I3 * omega3)
NT = 0

%Inward iterations to compute forces and torques
syms f0 f1 f2 f3 fT n0 n1 n2 n3 nT real

fT = FT
nT = NT

f3 = R_T_3 * fT + F3
f2 = R_3_2 * f3 + F2
f1 = R_2_1 * f2 + F1
f0 = R_1_0 * f1 + F0

n3 = N3 + R_T_3 * nT + cross([L_X_EE; 0; 0], F3) + cross(T_3_T(1:3, 4), R_T_3 * fT)
n2 = N2 + R_3_2 * n3 + cross([a2; 0; 0], F2) + cross(T_2_3(1:3, 4), R_3_2 * f3)
n1 = N1 + R_2_1 * n2 + cross([a1; 0; 0], F1) + cross(T_1_2(1:3, 4), R_2_1 * f2)
n0 = N0 + R_1_0 * n1 + cross([0; 0; 0], F0) + cross(T_0_1(1:3, 4), R_1_0 * f1)

tau3 =  transpose(n3) * [0; 0; 1]
tau2 = transpose(n2) * [0; 0; 1]
tau1 = transpose(n1) * [0; 0; 1]
tau0 = transpose(n0) * [0; 0; 1]

tau = [tau1; tau2; tau3]; 

%%State-Space representation of the manipulator dynamical equation
% Mass matrix (M) 
M = sym('M', [3, 3]); 
for i = 1:3 
    for j = 1:3 M(i, j) = diff(tau(i), theta_ddot(j)); 
    end 
end 

% Centrifugal and Coriolis forces (C) 
C = sym('C', [3, 3]); 
for i = 1:3 
    for j = 1:3 C(i, j) = 0; 
        for k = 1:3 C(i, j) = C(i, j) + 1/2 * (diff(M(i, j), theta(k)) + diff(M(i, k), theta(j)) - diff(M(j, k), theta(i))) * theta_dot(k); 
        end 
    end 
end 

% Gravity vector (G) 
G = sym('G', [3, 1]);
for i = 1:3 
    G(i) = diff(tau(i), g);
end