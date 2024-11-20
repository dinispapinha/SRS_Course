%Forward kinematics implementation of the robot

joints = cell(4);
nb_of_joints = input('Enter the number of joints of the robot: ');

for i = 1:nb_of_joints
    question_type = sprintf('Is joint n°%d revolute or prismatic? [R/P]: ', i);
    type_of_joint = input(question_type, 's');
    
    if type_of_joint == 'P'
        question_offset = sprintf('What is the value of offset between frame %d and frame %d? ', i, i-1);
        offset = input(question_offset);
        
        question_length = sprintf('What is the length of the arm between frame %d and frame %d? ', i, i-1);
        L = input(question_length);
        
        joints{i} = [0, L, offset, 0];

    elseif type_of_joint == 'R'
        question_theta = sprintf('What is the value of the angle between frame %d and frame %d? ', i, i-1);
        theta = input(question_theta);
        
        question_length = sprintf('What is the length of the arm between frame %d and frame %d? ', i, i-1);
        L = input(question_length);
        
        joints{i} = [0, L, 0, theta];

    else
        disp('Invalid input. Please enter R for revolute or P for prismatic.');

    end
end