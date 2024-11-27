function [T_matrix] = T_matrix(MDH)
%Compute the transformation matrix between two frames thanks to the
%Modified Denavit-Hartenberg table

alpha = MDH(1);
a = MDH(2);
d = MDH(3);
theta = MDH(4);

T_matrix = [
    cos(theta), -sin(theta), 0, a;
    sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d;
    sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d;
    0, 0, 0, 1
];

end

