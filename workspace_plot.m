clear
clc
close
data = readtable('work_space_minus_50.txt', 'Delimiter', '\t');

ins1 = data.Ins1;
rot1 = data.Rot1;
ins2 = data.Ins2;
rot2 = data.Rot2;
ins3 = data.Ins3;
rot3 = data.Rot3;
X = data.X;
Y = data.Y;
Z = data.Z;

figure;
scatter3(X, Y, Z, 20, ins3, 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Workspace del tubo 3');
xlim([-50 50]); ylim([-50 50]); zlim([0 200])
grid on;
colormap jet; 
colorbar;