% This has two function:
% 1. Plot the joint torques, accelerations, velocities and joint angles using the ¡°scope¡± blocks in
% Simulink.
% 2. plot the 3D trajectory using the instruction ¡°plot3(X,Y,Z)¡±
pos_d=zeros(50,3);
for i=1:1:50
[pos_d(i,1),pos_d(i,2),pos_d(i,3)]=ForwardKinematic(theta_d(i+1,1),theta_d(i+1,2),theta_d(i+1,3),theta_d(i+1,4),theta_d(i+1,5),theta_d(i+1,6));

end
pos_real=zeros(50,3);
for i=1:1:50
[pos_real(i,1),pos_real(i,2),pos_real(i,3)]=ForwardKinematic(theta_real(i+1,1),theta_real(i+1,2),theta_real(i+1,3),theta_real(i+1,4),theta_real(i+1,5),theta_real(i+1,6));

end

figure(1);
plot3(pos_d(:,1),pos_d(:,2),pos_d(:,3),'g');
hold on; 
plot3(pos_real(:,1),pos_real(:,2),pos_real(:,3),'r');
% axis([0 1.5 0 1.5 0 1.5]);
grid on;
