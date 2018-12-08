% This file aim to validate the accuracy of all the inverse kinematics solutions obtained in Inverse.m using
% forward kinematics. 
A1_0_1=DH_matrix(deg2rad(Degree_1(1)), -pi/2,    0,     0)
A1_1_2=DH_matrix(deg2rad(Degree_1(2)),     0,  0.5,  0.25)
A1_2_3=DH_matrix(deg2rad(Degree_1(3)),  pi/2,    0,     0)
A1_3_4=DH_matrix(deg2rad(Degree_1(4)), -pi/2,    0,     1)
A1_4_5=DH_matrix(deg2rad(Degree_1(5)),  pi/2,    0,     0)
A1_5_6=DH_matrix(deg2rad(Degree_1(6)),     0,    0,   0.5)

A1_0_6=A1_0_1*A1_1_2*A1_2_3*A1_3_4*A1_4_5*A1_5_6


A2_0_1=DH_matrix(deg2rad(Degree_2(1)), -pi/2,    0,     0)
A2_1_2=DH_matrix(deg2rad(Degree_2(2)),     0,  0.5,  0.25)
A2_2_3=DH_matrix(deg2rad(Degree_2(3)),  pi/2,    0,     0)
A2_3_4=DH_matrix(deg2rad(Degree_2(4)), -pi/2,    0,     1)
A2_4_5=DH_matrix(deg2rad(Degree_2(5)),  pi/2,    0,     0)
A2_5_6=DH_matrix(deg2rad(Degree_2(6)),     0,    0,   0.5)

A2_0_6=A2_0_1*A2_1_2*A2_2_3*A2_3_4*A2_4_5*A2_5_6

A3_0_1=DH_matrix(deg2rad(Degree_3(1)), -pi/2,    0,     0)
A3_1_2=DH_matrix(deg2rad(Degree_3(2)),     0,  0.5,  0.25)
A3_2_3=DH_matrix(deg2rad(Degree_3(3)),  pi/2,    0,     0)
A3_3_4=DH_matrix(deg2rad(Degree_3(4)), -pi/2,    0,     1)
A3_4_5=DH_matrix(deg2rad(Degree_3(5)),  pi/2,    0,     0)
A3_5_6=DH_matrix(deg2rad(Degree_3(6)),     0,    0,   0.5)

A3_0_6=A3_0_1*A3_1_2*A3_2_3*A3_3_4*A3_4_5*A3_5_6

A4_0_1=DH_matrix(deg2rad(Degree_4(1)), -pi/2,    0,     0)
A4_1_2=DH_matrix(deg2rad(Degree_4(2)),     0,  0.5,  0.25)
A4_2_3=DH_matrix(deg2rad(Degree_4(3)),  pi/2,    0,     0)
A4_3_4=DH_matrix(deg2rad(Degree_4(4)), -pi/2,    0,     1)
A4_4_5=DH_matrix(deg2rad(Degree_4(5)),  pi/2,    0,     0)
A4_5_6=DH_matrix(deg2rad(Degree_4(6)),     0,    0,   0.5)

A4_0_6=A4_0_1*A4_1_2*A4_2_3*A4_3_4*A4_4_5*A4_5_6