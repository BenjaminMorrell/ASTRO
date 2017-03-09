function C = DCM_be(Eul)
%> @file DCM_be.m
% ======================================================================
%> @brief Creates a DCM from Euler angles with the 3,2,1 rotation scheme. 
%>
%> @param Eul The Euler angles (3x1) for the rotation
%> @param C The output rotation Direction Cosine Matrix (DCM), 3x3

C = L1(Eul(1))*L2(Eul(2))*L3(Eul(3));