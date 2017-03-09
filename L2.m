function [L2] = L2(theta)
%> @file L3.m
% ======================================================================
%> @brief This function is a single axis transformation about a y axis. 
%>
%> @param theta the input angle to create the rotation matrix. In radians.
%>
%> @param L2 the output rotation matrix.
%>
%> Updated: 06/06/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L2 = [cos(theta),0,-sin(theta);0,1,0; sin(theta),0,cos(theta)];
end