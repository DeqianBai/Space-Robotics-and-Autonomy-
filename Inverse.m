% This file aim to compute the inverse kinematic solutions for all possible arm configurations

% the link parameters
a2=0.5;  % Link length
d2=0.25; % Offset distance
d4=1;    % Offset distance
d6=0.5;  % Offset distance

% Fmatrix used to get n,s,a,p
Fmatrix=[-1/sqrt(2) 0 1/sqrt(2) 1;
    0 -1 0 1;
    1/sqrt(2) 0 1/sqrt(2) 0;
    0 0 0 1];

% use array operations to get the [n s a] rotational submatrix
P=Fmatrix(:,4);
a=Fmatrix(:,3);
s=Fmatrix(:,2);
n=Fmatrix(:,1);

% set the last item to[]
P(4,:)=[];
a(4,:)=[];
s(4,:)=[];
n(4,:)=[];

% compute P_wrist and P_arm
P_wrist=d6*a;
P_arm=P-P_wrist;

% compute  Joint angle:theta
% theta 1
theta1_1=atan2(P_arm(2),P_arm(1))-atan2(d2,sqrt(P_arm(1)*P_arm(1)+P_arm(2)*P_arm(2)-d2*d2));
theta1_2=atan2(P_arm(2),P_arm(1))-atan2(d2,-sqrt(P_arm(1)*P_arm(1)+P_arm(2)*P_arm(2)-d2*d2));

% prepare for theta 2 and theat 3
A1=cos(theta1_1)*P_arm(1)+sin(theta1_1)*P_arm(2);
B1=(A1*A1+P_arm(3)*P_arm(3)+a2*a2-d4*d4)/(2*a2);
A2=cos(theta1_2)*P_arm(1)+sin(theta1_2)*P_arm(2);
B2=(A2*A2+P_arm(3)*P_arm(3)+a2*a2-d4*d4)/(2*a2);
% theta 2
theta2_1_1=atan2(A1,P_arm(3))-atan2(B1,sqrt(A1*A1+P_arm(3)*P_arm(3)-B1*B1));
theta2_1_2=atan2(A1,P_arm(3))-atan2(B1,-sqrt(A1*A1+P_arm(3)*P_arm(3)-B1*B1));
theta2_2_1=atan2(A2,P_arm(3))-atan2(B2,sqrt(A2*A2+P_arm(3)*P_arm(3)-B2*B2));
theta2_2_2=atan2(A2,P_arm(3))-atan2(B2,-sqrt(A2*A2+P_arm(3)*P_arm(3)-B2*B2));
% theta 3
theta3_1_1=atan2(A1-a2*cos(theta2_1_1),P_arm(3)+a2*sin(theta2_1_1))-theta2_1_1;
theta3_1_2=atan2(A1-a2*cos(theta2_1_2),P_arm(3)+a2*sin(theta2_1_2))-theta2_1_2;
theta3_2_1=atan2(A2-a2*cos(theta2_2_1),P_arm(3)+a2*sin(theta2_2_1))-theta2_2_1;
theta3_2_2=atan2(A2-a2*cos(theta2_2_2),P_arm(3)+a2*sin(theta2_2_2))-theta2_2_2;
% theta 4
theta4_1_1=atan2(-sin(theta1_1)*a(1)+cos(theta1_1)*a(2),...
                cos(theta2_1_1+theta3_1_1)*(cos(theta1_1)*a(1)+sin(theta1_1)*a(2))-sin(theta2_1_1+theta3_1_1)*a(3));
theta4_1_2=atan2(-sin(theta1_1)*a(1)+cos(theta1_1)*a(2),...
                cos(theta2_1_2+theta3_1_2)*(cos(theta1_1)*a(1)+sin(theta1_1)*a(2))-sin(theta2_1_2+theta3_1_2)*a(3));
theta4_2_1=atan2(-sin(theta1_2)*a(1)+cos(theta1_2)*a(2),...
                cos(theta2_2_1+theta3_2_1)*(cos(theta1_2)*a(1)+sin(theta1_2)*a(2))-sin(theta2_2_1+theta3_2_1)*a(3));
theta4_2_2=atan2(-sin(theta1_2)*a(1)+cos(theta1_2)*a(2),...
                cos(theta2_2_2+theta3_2_2)*(cos(theta1_2)*a(1)+sin(theta1_2)*a(2))-sin(theta2_2_2+theta3_2_2)*a(3));
% theta 5
theta5_1_1=atan2(sqrt((cos(theta1_1)*cos(theta2_1_1+theta3_1_1)*a(1)+sin(theta1_1)*cos(theta2_1_1+theta3_1_1)*a(2)-sin(theta2_1_1+theta3_1_1)*a(3))^2+(-sin(theta1_1)*a(1)+cos(theta1_1)*a(2))^2),...
                sin(theta2_1_1+theta3_1_1)*(cos(theta1_1)*a(1)+sin(theta1_1)*a(2))+cos(theta2_1_1+theta3_1_1)*a(3));
theta5_1_2=atan2(sqrt((cos(theta1_1)*cos(theta2_1_2+theta3_1_2)*a(1)+sin(theta1_1)*cos(theta2_1_2+theta3_1_2)*a(2)-sin(theta2_1_2+theta3_1_2)*a(3))^2+(-sin(theta1_1)*a(1)+cos(theta1_1)*a(2))^2),...
                sin(theta2_1_2+theta3_1_2)*(cos(theta1_1)*a(1)+sin(theta1_1)*a(2))+cos(theta2_1_2+theta3_1_2)*a(3));
theta5_2_1=atan2(sqrt((cos(theta1_2)*cos(theta2_2_1+theta3_2_1)*a(1)+sin(theta1_2)*cos(theta2_2_1+theta3_2_1)*a(2)-sin(theta2_2_1+theta3_2_1)*a(3))^2+(-sin(theta1_2)*a(1)+cos(theta1_2)*a(2))^2),...
                sin(theta2_2_1+theta3_2_1)*(cos(theta1_2)*a(1)+sin(theta1_2)*a(2))+cos(theta2_2_1+theta3_2_1)*a(3));        
theta5_2_2=atan2(sqrt((cos(theta1_2)*cos(theta2_2_2+theta3_2_2)*a(1)+sin(theta1_2)*cos(theta2_2_2+theta3_2_2)*a(2)-sin(theta2_2_2+theta3_2_2)*a(3))^2+(-sin(theta1_2)*a(1)+cos(theta1_2)*a(2))^2),...
                sin(theta2_2_2+theta3_2_2)*(cos(theta1_2)*a(1)+sin(theta1_2)*a(2))+cos(theta2_2_2+theta3_2_2)*a(3));           
% theta 6            
theta6_1_1=atan2(sin(theta2_1_1+theta3_1_1)*(cos(theta1_1)*s(1)+sin(theta1_1)*s(2))+cos(theta2_1_1+theta3_1_1)*s(3),...
                -(sin(theta2_1_1+theta3_1_1)*(cos(theta1_1)*n(1)+sin(theta1_1)*n(2))+cos(theta2_1_1+theta3_1_1)*n(3)));            
theta6_1_2=atan2(sin(theta2_1_2+theta3_1_2)*(cos(theta1_1)*s(1)+sin(theta1_1)*s(2))+cos(theta2_1_2+theta3_1_2)*s(3),...
                -(sin(theta2_1_2+theta3_1_2)*(cos(theta1_1)*n(1)+sin(theta1_1)*n(2))+cos(theta2_1_2+theta3_1_2)*n(3)));            
theta6_2_1=atan2(sin(theta2_2_1+theta3_2_1)*(cos(theta1_2)*s(1)+sin(theta1_2)*s(2))+cos(theta2_2_1+theta3_2_1)*s(3),...
                -(sin(theta2_2_1+theta3_2_1)*(cos(theta1_2)*n(1)+sin(theta1_2)*n(2))+cos(theta2_2_1+theta3_2_1)*n(3)));            
theta6_2_2=atan2(sin(theta2_2_2+theta3_2_2)*(cos(theta1_2)*s(1)+sin(theta1_2)*s(2))+cos(theta2_2_2+theta3_2_2)*s(3),...
                -(sin(theta2_2_2+theta3_2_2)*(cos(theta1_2)*n(1)+sin(theta1_2)*n(2))+cos(theta2_2_2+theta3_2_2)*n(3)));
            
            
Degree_1=[rad2deg(theta1_1);rad2deg(theta2_1_1);rad2deg(theta3_1_1);rad2deg(theta4_1_1);rad2deg(theta5_1_1);rad2deg(theta6_1_1)];
Degree_2=[rad2deg(theta1_1);rad2deg(theta2_1_2);rad2deg(theta3_1_2);rad2deg(theta4_1_2);rad2deg(theta5_1_2);rad2deg(theta6_1_2)];
Degree_3=[rad2deg(theta1_2);rad2deg(theta2_2_1);rad2deg(theta3_2_1);rad2deg(theta4_2_1);rad2deg(theta5_2_1);rad2deg(theta6_2_1)];
Degree_4=[rad2deg(theta1_2);rad2deg(theta2_2_2);rad2deg(theta3_2_2);rad2deg(theta4_2_2);rad2deg(theta5_2_2);rad2deg(theta6_2_2)];

plot(Degree_1)
hold on
plot(Degree_2)
hold on
plot(Degree_3)
hold on
plot(Degree_4)
legend('Degree_1','Degree_2','Degree_3','Degree_4');
Degree_1
Degree_2
Degree_3
Degree_4

