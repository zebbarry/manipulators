close all; clear; format long; clc;

origin = [4.71, 0, 144.76];
reference = [-577.06, -446.46, 341.38];
vector = reference - origin;


origin_norm = origin/norm(origin);
reference_norm = reference/norm(reference);
axis_norm = vector / norm(vector);

% silvia_angle = acos(dot(silvia_y_norm, origin));

xa = [1 0 0];
ya = [0 1 0];
za = [0 0 1];
yb = axis_norm;
% angle = acos(dot(yb, ya))
angle = deg2rad(-90);

transformX = [1 0 0;
              0 cos(angle) -sin(angle);
              0 sin(angle) cos(angle)];
transformY = [cos(angle) 0 sin(angle);
              0 1 0;
              -sin(angle) 0 cos(angle)]
transformZ = [cos(angle) -sin(angle) 0;
              sin(angle) cos(angle) 0;
              0 0 1];
norm(transformY)
         