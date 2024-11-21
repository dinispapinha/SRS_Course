% Forward kinematics implementation of the robot

% Initialisation
nb_of_joints = input('Enter the number of joints of the robot: ');
joints = zeros(nb_of_joints + 1, 4);
joints(1, :) = [0, 0, 0, 0];
T_matrixes = zeros(4, 4, nb_of_joints);

for i = 2:nb_of_joints + 1
    question_type = sprintf('Is joint n°%d revolute or prismatic? [R/P]: ', i-1);
    type_of_joint = input(question_type, 's');

    if type_of_joint == 'P'
        question_offset = sprintf('What is the value of offset between frame %d and frame %d? ', i-1, i-2);
        offset = input(question_offset);

        question_length = sprintf('What is the length of the arm between frame %d and frame %d? ', i-1, i-2);
        L = input(question_length);

        joints(i, :) = [0, L, offset, 0];

    elseif type_of_joint == 'R'
        question_theta = sprintf('What is the value of the angle between frame %d and frame %d? ', i-1, i-2);
        theta = input(question_theta);
        theta = deg2rad(theta);
        question_length = sprintf('What is the length of the arm between frame %d and frame %d? ', i-1, i-2);
        L = input(question_length);

        joints(i, :) = [0, L, 0, theta];

    else
        disp('Invalid input. Please enter R for revolute or P for prismatic.');

    end

    T_matrixes(:, :, i-1) = [
        cos(joints(i, 4)), -sin(joints(i, 4)), 0, joints(i, 2);
        sin(joints(i, 4)) * cos(joints(i-1, 1)), cos(joints(i, 4)) * cos(joints(i-1, 1)), -sin(joints(i-1, 1)), -sin(joints(i-1, 1)) * joints(i, 3);
        sin(joints(i, 4)) * sin(joints(i-1, 1)), cos(joints(i, 4)) * sin(joints(i-1, 1)), cos(joints(i-1, 1)), cos(joints(i-1, 1)) * joints(i, 3);
        0, 0, 0, 1
    ];
end

T_total = eye(4);

for i = 1:nb_of_joints
    T_total = T_total * T_matrixes(:, :, i);
end

disp('Joints:');
disp(joints);
disp('Transformation matrixes:');
for i = 1:nb_of_joints
    fprintf('T_matrixes(%d):\n', i);
    disp(T_matrixes(:, :, i));
end

disp('The position of end-effector is:');
disp(T_total(1:3, 4));

% Implementation of inverse kinematics using initial position

c2 = ((T_total(1, 4))^2 + (T_total(2, 4))^2 - joints(3, 2)^2 - joints(4, 2)^2) / (2 * joints(3, 2) * joints(4, 2)); 
if abs(c2) <= 1
    s2 = [sqrt(1 - c2^2), -sqrt(1 - c2^2)];
    theta2 = atan2(s2, c2);
    k1 = joints(3, 2) + joints(4, 2) * c2;
    k2 = joints(4, 2) * s2;
    theta1 = atan2(T_total(2, 4), T_total(1, 4)) - atan2(k2, k1);
    theta3 = atan2(T_total(2, 1), T_total(1, 1)) - theta1 - theta2;
    for i = 1:length(theta3)
        fprintf('Position number {%d} possible :\n', i);
        disp(rad2deg([theta1(i), theta2(i), theta3(i)]));
    end
else
    disp('ERROR: Robot can not reach the coordinates.');
end

% Implementation of Jacobian Matrices

R_matrixes = zeros(3, 3, nb_of_joints);
theta_dots = zeros(3, nb_of_joints + 1);
theta_dots(:, 1) = [0; 0; 0]; % We assume the base frame with no angular velocity (fixed)
omega_joints = zeros(3, nb_of_joints + 1);
omega_joints(:, 1) = [0; 0; 0];
v_joints = zeros(3, nb_of_joints + 1);
v_joints(:, 1) = [0; 0; 0]; % We assume the base with no linear velocity (fixed)

for i = 1:nb_of_joints
    R_matrixes(:, :, i) = [
        cos(joints(i+1, 4)), -sin(joints(i+1, 4)), 0;
        sin(joints(i+1, 4)), cos(joints(i+1, 4)), 0;
        0, 0, 1
    ];
    theta_dots(:, i+1) = [0; 0; 1]; % We assume 1 here but value to be changed
    omega_joints(:, i+1) = R_matrixes(:, :, i) * omega_joints(:, i) + theta_dots(:, i+1);
    pos_i_plus_1_in_i = T_matrixes(:, :, i) * [0; 0; 0; 1];
    fprintf('Position of joint %d in frame %d :\n', i, i-1);
    v_joints(:, i+1) = R_matrixes(:, :, i) * (v_joints(:, i) + cross(omega_joints(:, i),pos_i_plus_1_in_i));
    disp(pos_i_plus_1_in_i(1:3));
end
