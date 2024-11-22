% Author: Dinis Papinha
% Date: 22/11/2024

clc;
clear;
close all;

%% Parameters for 3R planar manipulator variables

% Define symbolic variables for joint angles (theta), lengths (a), and angular velocities/accelerations
syms theta1 theta2 theta3 a1 a2 a3 d1 d2 d3 real
syms theta1_dot theta2_dot theta3_dot theta1_ddot theta2_ddot theta3_ddot real
syms m1 m2 m3 I1 I2 I3 g real

% Modified DH parameters
% [theta, d, a, alpha]
MDH = [
    theta1, 0, a1, 0;
    theta2, 0, a2, 0;
    theta3, 0, a3, 0;
];

%% Forward Kinematics

% Transformation Matrices using MDH
T01 = MDH_to_transformation(MDH(1,:));
T12 = MDH_to_transformation(MDH(2,:));
T23 = MDH_to_transformation(MDH(3,:));
T02 = simplify(T01 * T12);
T03 = simplify(T02 * T23);
disp('Transformation Matrix from 0 to End-Effector:');
disp(T03);

% Outputing End-Effector Position
P0_EE = T03(1:2, 4); % End-effector position in 2D (x, y)
disp('End-Effector Position (Analytical):');
disp(P0_EE);

%% Forward Kinematics with specific geometry

% Example of joint angles and link lengths
theta_vals = [pi/4, pi/6, pi/3]; 
a_vals = [1, 1, 1];              

% Evaluate T03 numerically
T03_numeric = double(subs(T03, [theta1, theta2, theta3, a1, a2, a3], [theta_vals, a_vals]));

disp('End-Effector Position (Numerical):');
disp(T03_numeric(1:2, 4));

%% Inverse Kinematics (Analytical)
% Solve for joint angles given end-effector position (x, y, phi) and link lengths (a1, a2, a3)

% Define symbolic variables
syms x y phi a1 a2 a3 theta1 theta2 theta3 real

% Define equations
Eqns = [
    x == T03(1,4);
    y == T03(2,4);
    tan(phi) == tan(theta1 + theta2 + theta3);
]

% Give values of x, y, phi, and link lengths
% x_vec = [3, 0, pi/4];
% a_vals = [2, 2, 0.5];   
x_vec = [3, 0, pi/4];
a_vals = [2, 2, 0.5];   
Eqns_with_geometry = subs(Eqns, [x, y, phi, a1, a2, a3], [x_vec a_vals])

% Solve symbolically 
[sol_theta1, sol_theta2, sol_theta3] = solve(Eqns_with_geometry, [theta1, theta2, theta3])%,'ReturnConditions', 'true')
% NEEDS TO BE MODIFIED TO OUTPUT ALL SOLUTIONS

% % Convert symbolic solutions to numeric for numerical refinement
% sol_theta1_num = double(sol_theta1);
% sol_theta2_num = double(sol_theta2);
% sol_theta3_num = double(sol_theta3);
% 
% % Display the symbolic solutions
% disp('Symbolic Solutions:');
% disp([sol_theta1_num, sol_theta2_num, sol_theta3_num]);

%% Inverse Kinematics (Numerical)

% use the toolbox


%% Jacobian Matrix
J = jacobian(P0_EE, [theta1, theta2, theta3]);
disp('Jacobian Matrix:');
disp(J);

%% Dynamics using Newton-Euler
% Initialize variables
z0 = [0; 0; 1]; % z-axis for planar manipulator
P01 = T01(1:3, 4); P12 = T12(1:3, 4); P23 = T23(1:3, 4); P03 = T03(1:3, 4);

% Velocities and Accelerations (Linear and Angular)
omega = sym(zeros(3,3));
alpha = sym(zeros(3,3));
a = sym(zeros(3,3));
f = sym(zeros(3,1));
tau = sym(zeros(3,1));

% Base values
omega(:,1) = [0; 0; 0];
alpha(:,1) = [0; 0; 0];
% a(:,1) = [0; -g; 0];
a(:,1) = [0; 0; 0];

% Forward recursion
for i = 1:3
    T_prev = eval(sprintf('T0%d', i));
    R_prev = T_prev(1:3, 1:3);
    omega(:,i+1) = R_prev' * (omega(:,i) + theta1_dot * z0);
    alpha(:,i+1) = R_prev' * (alpha(:,i) + theta1_ddot * z0 + cross(omega(:,i), theta1_dot * z0));
    a(:,i+1) = R_prev' * a(:,i) + cross(alpha(:,i+1), P01) + cross(omega(:,i+1), cross(omega(:,i+1), P01));
end

% Backward recursion for forces and torques
for i = 3:-1:1
    f(:,i) = m1 * a(:,i); % Linear forces
    tau(:,i) = cross(P01, f(:,i)) + I1 * alpha(:,i) + cross(omega(:,i), I1 * omega(:,i)); % Torques
end

disp('Dynamic Equations (Newton-Euler):');
disp(tau);

%% MDH to Transformation Matrix
function T = MDH_to_transformation(params)
    theta = params(1);
    d = params(2);
    a = params(3);
    alpha = params(4);
    T = [
        cos(theta), -sin(theta), 0, a;
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);
        0, 0, 0, 1;
    ];
end
