function [L3] = L3(psi)
%> @file L3.m
% ======================================================================
%> @brief This function is a single axis transformation about a z axis. 
%>
%> @param psi the input angle to create the rotation matrix. In radians.
%>
%> @param L3 the output rotation matrix.
%>
%> Updated: 06/06/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L3 = [cos(psi),sin(psi),0;-sin(psi),cos(psi),0;0,0,1];
end