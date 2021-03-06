close all; clear; format long; clc;

origin = [482.29, -433.74, 314.13];
reference = [370.66, -321.55, 66.86];
vector = reference - origin;


origin_norm = origin/norm(origin);
reference_norm = reference/norm(reference);
axis_norm = vector / norm(vector);

% silvia_angle = acos(dot(silvia_y_norm, origin));

xa = [1 0 0];
ya = [0 1 0];
za = [0 0 1];
xb = axis_norm;
angle = acos(dot(xb, xa));
rad2deg(angle)
angle = deg2rad(-180);

transformX = [1 0 0;
              0 cos(angle) -sin(angle);
              0 sin(angle) cos(angle)];
transformY = [cos(angle) 0 sin(angle);
              0 1 0;
              -sin(angle) 0 cos(angle)];
transformZ = [cos(angle) -sin(angle) 0;
              sin(angle) cos(angle) 0;
              0 0 1];
transform = transformY
% norm(transform)
