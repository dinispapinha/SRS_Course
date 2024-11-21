%Forward kinematics implementation of the robot

joints = cell(4);
joints{1} = [0,0,0,0];
nb_of_joints = input('Enter the number of joints of the robot: ');
T_matrixes = cell(1, nb_of_joints);

for i = 2:nb_of_joints+1
    question_type = sprintf('Is joint n°%d revolute or prismatic? [R/P]: ', i-1);
    type_of_joint = input(question_type, 's');
    
    if type_of_joint == 'P'
        question_offset = sprintf('What is the value of offset between frame %d and frame %d? ', i-1, i-2);
        offset = input(question_offset);
        
        question_length = sprintf('What is the length of the arm between frame %d and frame %d? ', i-1, i-2);
        L = input(question_length);
        
        joints{i} = [0, L, offset, 0];

    elseif type_of_joint == 'R'
        question_theta = sprintf('What is the value of the angle between frame %d and frame %d? ', i-1, i-2);
        theta = input(question_theta);
        
        question_length = sprintf('What is the length of the arm between frame %d and frame %d? ', i-1, i-2);
        L = input(question_length);
        
        joints{i} = [0, L, 0, theta];

    else
        disp('Invalid input. Please enter R for revolute or P for prismatic.');

    end

T_matrixes{i} = [
    cos(joints{i}(4)), -sin(joints{i}(4)), 0, joints{i-1}(2);
    sin(joints{i}(4)) * cos(joints{i-1}(1)), cos(joints{i}(4)) * cos(joints{i-1}(1)), -sin(joints{i-1}(1)), -sin(joints{i-1}(1)) * joints{i}(3);
    sin(joints{i}(4)) * sin(joints{i-1}(1)), cos(joints{i}(4)) * sin(joints{i-1}(1)), cos(joints{i-1}(1)), cos(joints{i-1}(1)) * joints{i}(3);
    0, 0, 0, 1
];

end

T_total = eye(4); 

for i = 2:nb_of_joints + 1 
    T_total = T_total * T_matrixes{i}; 
    
end  

disp('The position of end-effector is :'); 

disp(T_total(1:3, 4));