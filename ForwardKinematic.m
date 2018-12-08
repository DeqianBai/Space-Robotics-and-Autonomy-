% This file is for Forward Kinematic
% inputs: theta1, theta2, theta3, theta4, theta5, theta6
% outputs: X, Y, Z
function [X,Y,Z]=ForwardKinematic(theta1,theta2,theta3,theta4,theta5,theta6)
A1_0_1=DH_matrix(deg2rad(theta1), -pi/2,    0,     0);
A1_1_2=DH_matrix(deg2rad(theta2),     0,  0.5,  0.25);
A1_2_3=DH_matrix(deg2rad(theta3),  pi/2,    0,     0);
A1_3_4=DH_matrix(deg2rad(theta4), -pi/2,    0,     1);
A1_4_5=DH_matrix(deg2rad(theta5),  pi/2,    0,     0);
A1_5_6=DH_matrix(deg2rad(theta6),     0,    0,   0.5);

A1_0_6=A1_0_1*A1_1_2*A1_2_3*A1_3_4*A1_4_5*A1_5_6;
X=A1_0_6(1,4)
Y=A1_0_6(2,4)
Z=A1_0_6(3,4)

end