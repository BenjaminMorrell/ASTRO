function [Rot_Mat] = computeCylinderRotationMatrix(x1,x2)
% Takes the input of the start (x1) and end (x2) of a cylinder and computes the
% rotation matrix to define the orientation of the end caps (and to use to
% rotate from being aligned with the z axis to being angled.

a = x2-x1;
ahat = a/norm(a);   %Unit vector
zhat = [0;0;1];     % z unit vector

% Compute the rotation matrix for the end-caps
rotVec = cross(ahat,zhat); % khat * sin(theta)
rotNorm = norm(rotVec); % sin(theta)
rotDot = dot(ahat,zhat); %cos(theta)
% Vector of rotation
if (rotNorm > 1e-10)
    k = rotVec/rotNorm; %normalize to get rotation axis (khat)
else
    k = [0;1;0] * sign(rotDot);
end

% Angle of rotation
th = atan2(rotNorm,rotDot);% theta= atan(sin(theta)/cos(theta))

%Find rotation matrix with Rodrigues's Rotation formula
kcross = [0, -k(3), k(2); k(3), 0, -k(1); -k(2), k(1), 0]; % Cross product matrix for the rotation vector
Rot_Mat = eye(3) + sin(th)*kcross + (1-cos(th))*(kcross*kcross); % Rodrigues's Rotation formula - fixed 20160502
