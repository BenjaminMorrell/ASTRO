function [L1] = L1(phi)
%> @file L1.m
% ======================================================================
%> @brief This function is a single axis transformation about an x axis. 
%>
%> @param phi the input angle to create the rotation matrix. In radians.
%>
%> @param L1 the output rotation matrix.
%>
%> Updated: 06/06/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L1 = [1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)];
end
