function [phi] = normalize_angle_vec(phi)
% Normalize phi to be between -pi and pi
% phi can also be a vector of angles

phi = mod(phi + pi, 2*pi) - pi;

end
