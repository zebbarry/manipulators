close all; clear; clc;

origin = [4.71, 0, 144.76];
reference = [660, 68.77, 156.07];
vector = flip(reference - origin);


origin_norm = origin/norm(origin);
reference_norm = reference/norm(reference);
axis_norm = vector / norm(vector);

% silvia_angle = acos(dot(silvia_y_norm, origin));

xa = [1 0 0];
ya = [0 1 0];
za = [0 0 1];
xb = axis_norm;
angle = acos(dot(xb, xa))
angle = deg2rad(-7.5);

transform = [cos(angle) 0 -sin(angle);
             sin(angle) 1 0;
             0 0 cos(angle)]
         