% This file is for Inverse Kinematics
% input : T(transformation matrix)
% outputs:theta1, theta2, theta3, theta4, theta5, theta6
function [theta1,theta2,theta3,theta4,theta5,theta6] = InverseKinematic(T)
% the link parameters
a2=0.5;  % Link length
d2=0.25; % Offset distance
d4=1;    % Offset distance
d6=0.5;  % Offset distance
% use array operations to get "P,a,s,n"
P=T(:,4);
a=T(:,3);
s=T(:,2);
n=T(:,1);

% set the last item to[]
P(4,:)=[];
a(4,:)=[];
s(4,:)=[];
n(4,:)=[];
% compute P_wrist and P_arm
P_wrist=d6*a;
P_arm=P-P_wrist;

% compute theta 1
theta1_1=atan2(P_arm(2),P_arm(1))-atan2(d2,sqrt(P_arm(1)*P_arm(1)+P_arm(2)*P_arm(2)-d2*d2));
% prepare for theta 2
A1=cos(theta1_1)*P_arm(1)+sin(theta1_1)*P_arm(2);
B1=(A1*A1+P_arm(3)*P_arm(3)+a2*a2-d4*d4)/(2*a2);
% compute theta 2
theta2_1_1=atan2(A1,P_arm(3))-atan2(B1,sqrt(A1*A1+P_arm(3)*P_arm(3)-B1*B1));
% compute theta 3
theta3_1_1=atan2(A1-a2*cos(theta2_1_1),P_arm(3)+a2*sin(theta2_1_1))-theta2_1_1;
% compute theta 4
theta4_1_1=atan2(-sin(theta1_1)*a(1)+cos(theta1_1)*a(2),...
    cos(theta2_1_1+theta3_1_1)*(cos(theta1_1)*a(1)+sin(theta1_1)*a(2))-sin(theta2_1_1+theta3_1_1)*a(3));
% compute theta 5
theta5_1_1=atan2(sqrt((cos(theta1_1)*cos(theta2_1_1+theta3_1_1)*a(1)+sin(theta1_1)*cos(theta2_1_1+theta3_1_1)*a(2)-sin(theta2_1_1+theta3_1_1)*a(3))^2+(-sin(theta1_1)*a(1)+cos(theta1_1)*a(2))^2),...
    sin(theta2_1_1+theta3_1_1)*(cos(theta1_1)*a(1)+sin(theta1_1)*a(2))+cos(theta2_1_1+theta3_1_1)*a(3));
% compute theta 6
theta6_1_1=atan2(sin(theta2_1_1+theta3_1_1)*(cos(theta1_1)*s(1)+sin(theta1_1)*s(2))+cos(theta2_1_1+theta3_1_1)*s(3),...
    -(sin(theta2_1_1+theta3_1_1)*(cos(theta1_1)*n(1)+sin(theta1_1)*n(2))+cos(theta2_1_1+theta3_1_1)*n(3)));


theta1=rad2deg(theta1_1);
theta2=rad2deg(theta2_1_1);
theta3=rad2deg(theta3_1_1);
theta4=rad2deg(theta4_1_1);
theta5=rad2deg(theta5_1_1);
theta6=rad2deg(theta6_1_1);

end